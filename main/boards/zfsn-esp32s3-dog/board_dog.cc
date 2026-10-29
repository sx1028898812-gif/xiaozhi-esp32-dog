#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include "pet_dog.h"
#include "ws2812_led.h"
#include "dog_display.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_sh1106.h>
#include <cJSON.h>

#define TAG "ZfsnDogBoard"

class DogStatusLed : public SingleLed {
public:
    DogStatusLed(gpio_num_t gpio, DogDisplay** display_ptr)
        : SingleLed(gpio), display_ptr_(display_ptr) {}

    void OnStateChanged() override {
        SingleLed::OnStateChanged();

        auto& app = Application::GetInstance();
        auto& dog = PetDog::GetInstance();
        DogDisplay* disp = display_ptr_ ? *display_ptr_ : nullptr;
        DeviceState state = app.GetDeviceState();

        switch (state) {
            case kDeviceStateIdle:
                if (disp) {
                    disp->EnterSleepingEyes();
                }
                if (!dog.IsBusy() && dog.GetCurrentPosture() != DogActionType::kSleep) {
                    dog.Sleep();
                }
                break;

            case kDeviceStateConnecting:
            case kDeviceStateListening:
            case kDeviceStateSpeaking:
                if (disp) {
                    disp->EnterAnimatedEyes();
                }
                if (dog.GetCurrentPosture() == DogActionType::kSleep) {
                    dog.Stand();
                }
                break;

            case kDeviceStateWifiConfiguring:
            case kDeviceStateActivating:
            case kDeviceStateUpgrading:
                if (disp) {
                    disp->EnterNormalUI();
                }
                break;

            default:
                break;
        }
    }

private:
    DogDisplay** display_ptr_ = nullptr;
};

class ZfsnDogBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_ = nullptr;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    DogDisplay* dog_display_ = nullptr;
    Display* fallback_display_ = nullptr;

    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;

    esp_timer_handle_t explore_timer_ = nullptr;
    int saved_normal_volume_ = 70;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSh1106Display() {
        esp_lcd_panel_io_i2c_config_t io_config = {};
        io_config.dev_addr = 0x3C;
        io_config.scl_speed_hz = 400 * 1000;
        io_config.control_phase_bytes = 1;
        io_config.dc_bit_offset = 6;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        io_config.on_color_trans_done = nullptr;
        io_config.user_ctx = nullptr;
        io_config.flags.dc_low_on_data = 0;
        io_config.flags.disable_control_phase = 0;

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SH1106 OLED driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize SH1106 display");
            fallback_display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        dog_display_ = new DogDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                      DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

    void InitializeButtons() {
        // GPIO 0: Shared by Head Touch Sensor and Boot Button
        // 1. Single Click / Short Touch:
        //    - Starting -> Enter WiFi Config Mode
        //    - Idle     -> Wake up, Stand up, Open Animated Eyes, Start Chat
        //    - Speaking -> Interrupt AI speech immediately
        //    - Listening-> Close audio channel & rest
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            auto state = app.GetDeviceState();
            if (state == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            if (state == kDeviceStateIdle) {
                PetDog::GetInstance().Stand();
                if (dog_display_) {
                    dog_display_->EnterAnimatedEyes();
                }
            }
            app.ToggleChatState();
        });

        // 2. Long Press Head Touch:
        //    - Trigger random cute action + happy eyes (does NOT force WS2812 LED on)
        boot_button_.OnLongPress([this]() {
            ESP_LOGI(TAG, "Head touch long-press: triggering random pet interaction");
            if (dog_display_) {
                dog_display_->EnterAnimatedEyes();
                dog_display_->SetEmotion("happy");
            }
            PetDog::GetInstance().ExecuteRandomAction();
        });

        // 3. Double Click Head Touch:
        //    - Toggle Free Explore mode on/off
        boot_button_.OnDoubleClick([this]() {
            bool enabled = PetDog::GetInstance().ToggleFreeExplore();
            GetDisplay()->ShowNotification(enabled ? "自由探索: 开启" : "自由探索: 关闭");
        });

        // GPIO 47: Secondary Touch Button (Push-to-Talk)
        touch_button_.OnPressDown([this]() {
            PetDog::GetInstance().Stand();
            if (dog_display_) {
                dog_display_->EnterAnimatedEyes();
            }
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        // GPIO 40: Volume Up Button
        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = std::min(100, codec->output_volume() + 10);
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });
        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        // GPIO 39: Volume Down Button
        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = std::max(0, codec->output_volume() - 10);
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });
        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    void InitializeFreeExploreTimer() {
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                auto& app = Application::GetInstance();
                auto& dog = PetDog::GetInstance();
                if (app.GetDeviceState() == kDeviceStateIdle &&
                    dog.GetFreeExplore() && !dog.IsBusy()) {
                    dog.ExecuteRandomAction();
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "dog_free_explore",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &explore_timer_));
        // Check every 45 seconds for idle free exploration
        ESP_ERROR_CHECK(esp_timer_start_periodic(explore_timer_, 45ULL * 1000 * 1000));
    }

    void InitializeMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        // 1. Core 13-Action Tool (100% compatible with seller 6.0 schema + voiceprint awareness)
        mcp_server.AddTool(
            "self.dog.action",
            "机器狗动作控制。支持以下动作：\n"
            "walk_forward: 向前走（可指定 steps 步数）\n"
            "walk_back: 向后退（可指定 steps 步数）\n"
            "turn_left: 向左转（可指定 steps 步数）\n"
            "turn_right: 向右转（可指定 steps 步数）\n"
            "sitdown: 坐下/蹲下\n"
            "stand: 站立/立正\n"
            "sleep: 趴下睡觉休息\n"
            "stretch: 伸懒腰\n"
            "scratching: 挠痒痒\n"
            "swing: 摇摆身体/开心摇晃（可指定 steps 次数和 angle 幅度）\n"
            "petwave: 招手打招呼/握手/作揖\n"
            "jump: 欢快跳跃\n"
            "stop: 立即停止当前动作并恢复站立。\n"
            "提示：当通过声纹识别出说话人是'小主人'时，打招呼可主动搭配 petwave 或 swing；识别出'爸爸'时可搭配 stand 或 petwave。",
            PropertyList({
                Property("action", kPropertyTypeString),
                Property("steps", kPropertyTypeInteger, 4, 1, 100),
                Property("angle", kPropertyTypeInteger, 30, 10, 60),
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string action = properties["action"].value<std::string>();
                int steps = properties["steps"].value<int>();
                int angle = properties["angle"].value<int>();
                auto& dog = PetDog::GetInstance();

                if (action == "walk_forward") {
                    dog.WalkForward(steps);
                } else if (action == "walk_back") {
                    dog.WalkBack(steps);
                } else if (action == "turn_left") {
                    dog.TurnLeft(steps);
                } else if (action == "turn_right") {
                    dog.TurnRight(steps);
                } else if (action == "sitdown") {
                    dog.Sitdown();
                } else if (action == "stand") {
                    dog.Stand();
                } else if (action == "sleep") {
                    dog.Sleep();
                } else if (action == "stretch") {
                    dog.Stretch();
                } else if (action == "scratching") {
                    dog.Scratching();
                } else if (action == "swing") {
                    dog.Swing(steps, angle);
                } else if (action == "petwave") {
                    dog.PetWave();
                } else if (action == "jump") {
                    dog.Jump();
                } else if (action == "stop") {
                    dog.Stop();
                } else {
                    return std::string("Error: unknown action '") + action + "'";
                }
                return true;
            });

        // 2. Persistent Feature Settings (free_explore, boot_animation, emotion_sync)
        mcp_server.AddTool(
            "self.dog.setting",
            "机器狗功能设置管理（持久化保存到NVS）。\n"
            "feature 可选值：\n"
            "- free_explore: 自由探索模式（待机时偶尔自己动一动）\n"
            "- boot_animation: 开机招手动画\n"
            "- emotion_sync: 情绪-动作-灯光三端共情联动\n"
            "operation 可选值：get（查询）、set（设置）、toggle（切换）",
            PropertyList({
                Property("feature", kPropertyTypeString),
                Property("operation", kPropertyTypeString, std::string("set")),
                Property("enable", kPropertyTypeBoolean, true),
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string feature = properties["feature"].value<std::string>();
                std::string op = properties["operation"].value<std::string>();
                bool enable = properties["enable"].value<bool>();
                auto& dog = PetDog::GetInstance();

                bool result_val = false;
                if (feature == "free_explore") {
                    if (op == "get") result_val = dog.GetFreeExplore();
                    else if (op == "toggle") result_val = dog.ToggleFreeExplore();
                    else { dog.SetFreeExplore(enable); result_val = enable; }
                } else if (feature == "boot_animation") {
                    if (op == "get") result_val = dog.GetBootAnimation();
                    else if (op == "toggle") result_val = dog.ToggleBootAnimation();
                    else { dog.SetBootAnimation(enable); result_val = enable; }
                } else if (feature == "emotion_sync") {
                    if (op == "get") result_val = dog.GetEmotionSync();
                    else if (op == "toggle") result_val = dog.ToggleEmotionSync();
                    else { dog.SetEmotionSync(enable); result_val = enable; }
                } else {
                    return std::string("Unknown feature: ") + feature;
                }

                cJSON* json = cJSON_CreateObject();
                cJSON_AddStringToObject(json, "feature", feature.c_str());
                cJSON_AddBoolToObject(json, "enabled", result_val);
                return json;
            });

        // 3. Servo Trim Calibration (Official v2.5.3 Robot Feature)
        mcp_server.AddTool(
            "self.dog.set_trim",
            "校准单个舵机的零位微调角度并永久保存到NVS，解决机器狗站立歪斜或走路偏航。\n"
            "servo_type 可选：left_front(左前腿)、right_front(右前腿)、left_back(左后腿)、right_back(右后腿)、reset(全部重置为0)。\n"
            "trim_value 范围：-30 到 30 度。",
            PropertyList({
                Property("servo_type", kPropertyTypeString),
                Property("trim_value", kPropertyTypeInteger, 0, -30, 30),
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string servo_type = properties["servo_type"].value<std::string>();
                int trim_value = properties["trim_value"].value<int>();
                if (!PetDog::GetInstance().SetTrim(servo_type, trim_value)) {
                    return std::string("Invalid servo_type. Use left_front, right_front, left_back, right_back, or reset.");
                }
                return std::string("Trim updated for ") + servo_type + ": " + std::to_string(trim_value);
            });

        mcp_server.AddTool(
            "self.dog.get_trims",
            "获取机器狗四条腿当前的舵机微调校准值（NVS持久化配置）。",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                auto trims = PetDog::GetInstance().GetTrims();
                cJSON* json = cJSON_CreateObject();
                for (const auto& kv : trims) {
                    cJSON_AddNumberToObject(json, kv.first.c_str(), kv.second);
                }
                return json;
            });

        // 4. Custom AI Servo Choreography (Official v2.5.3 Robot Feature)
        mcp_server.AddTool(
            "self.dog.servo_sequences",
            "AI自定义动作编排（复合机械舞/特技动作）。传入JSON数组字符串，每个元素包含四条腿目标角度(0-180, 90为中位站立)与延时毫秒d(40-3000)：\n"
            "示例：[{\"lf\":90,\"rf\":160,\"lb\":40,\"rb\":40,\"d\":300},{\"lf\":90,\"rf\":90,\"lb\":90,\"rb\":90,\"d\":300}]",
            PropertyList({
                Property("sequence_json", kPropertyTypeString),
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string seq_json = properties["sequence_json"].value<std::string>();
                std::string err;
                if (!PetDog::GetInstance().ExecuteServoSequence(seq_json, err)) {
                    return std::string("Error: ") + err;
                }
                return true;
            });

        // 5. Child Whisper / Bedtime Mode (Optional Feature #1)
        mcp_server.AddTool(
            "self.dog.whisper_mode",
            "儿童睡前故事/悄悄话模式。开启(enable=true)时自动将音量调低到轻柔音量、切换为乖巧坐姿(sitdown)，并将身体氛围灯调为柔和暖光呼吸或关闭；关闭(enable=false)时恢复正常音量与站姿。",
            PropertyList({
                Property("enable", kPropertyTypeBoolean, true),
                Property("volume", kPropertyTypeInteger, 25, 10, 50),
                Property("warm_light", kPropertyTypeBoolean, true),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                bool enable = properties["enable"].value<bool>();
                int whisper_vol = properties["volume"].value<int>();
                bool warm_light = properties["warm_light"].value<bool>();

                auto codec = GetAudioCodec();
                auto& dog = PetDog::GetInstance();
                auto& led = WS2812Led::GetInstance();

                if (enable) {
                    if (codec->output_volume() > whisper_vol) {
                        saved_normal_volume_ = codec->output_volume();
                    }
                    codec->SetOutputVolume(whisper_vol);
                    dog.Sitdown();
                    if (warm_light) {
                        led.StartBreathe("warm", 35);
                    } else {
                        led.TurnOff();
                    }
                    GetDisplay()->ShowNotification("悄悄话模式: 开启");
                } else {
                    codec->SetOutputVolume(saved_normal_volume_);
                    dog.Stand();
                    led.TurnOff();
                    GetDisplay()->ShowNotification("悄悄话模式: 关闭");
                }
                return true;
            });

        // 6. Dual WS2812 RGB LED Strip Control (100% compatible with seller 6.0 schema)
        mcp_server.AddTool(
            "self.ws2812_led.control",
            "控制机器狗身体两侧的WS2812 RGB彩灯带。\n"
            "action 可选值：\n"
            "- turn_on_off: 开关常亮灯光（color 设为 off 表示关闭）\n"
            "- flash_light: 闪烁灯效\n"
            "- water_light: 流水灯效\n"
            "- breathe_light: 呼吸灯效\n"
            "- rainbow_light: 七彩彩虹灯效\n"
            "color 可选值：red, green, blue, yellow, cyan, magenta, purple, orange, warm, white, off\n"
            "brightness 亮度范围：0 到 255",
            PropertyList({
                Property("action", kPropertyTypeString),
                Property("color", kPropertyTypeString, std::string("white")),
                Property("brightness", kPropertyTypeInteger, 128, 0, 255),
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string action = properties["action"].value<std::string>();
                std::string color = properties["color"].value<std::string>();
                int brightness = properties["brightness"].value<int>();
                return WS2812Led::GetInstance().Control(action, color, brightness);
            });

        // 7. Display Emotion & Celebration Firework Tools (Optional Feature #3)
        mcp_server.AddTool(
            "self.display.set_emotion",
            "设置机器狗屏幕眼神表情（如 happy, laughing, cool, surprised, sad, sleepy, neutral）。",
            PropertyList({
                Property("emotion", kPropertyTypeString),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string emotion = properties["emotion"].value<std::string>();
                GetDisplay()->SetEmotion(emotion.c_str());
                return true;
            });

        mcp_server.AddTool(
            "self.display.firework",
            "在机器狗OLED屏幕上播放绚丽的烟花粒子绽放动画。当表扬小主人（如算对数学题、背出古诗、讲礼貌、祝贺生日）时调用，可联动欢快跳跃和彩虹灯庆祝！",
            PropertyList({
                Property("celebrate_action", kPropertyTypeBoolean, true),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                bool celebrate = properties["celebrate_action"].value<bool>();
                if (dog_display_) {
                    dog_display_->ShowFirework();
                }
                if (celebrate) {
                    PetDog::GetInstance().Jump();
                    PetDog::GetInstance().Swing(4, 30);
                }
                return true;
            });
    }

public:
    ZfsnDogBoard()
        : boot_button_(BOOT_BUTTON_GPIO),
          touch_button_(TOUCH_BUTTON_GPIO),
          volume_up_button_(VOLUME_UP_BUTTON_GPIO),
          volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeSh1106Display();

        // Trigger power-on green water-fill effect (1s then off) & initial posture
        WS2812Led::GetInstance().PlayPowerOnEffect();
        PetDog::GetInstance().SetActionCallback([this](DogActionType action) {
            if (dog_display_ && PetDog::GetInstance().GetEmotionSync()) {
                dog_display_->SyncWithAction(action);
            }
        });

        InitializeButtons();
        InitializeFreeExploreTimer();
        InitializeMcpTools();
    }

    virtual Led* GetLed() override {
        static DogStatusLed led(BUILTIN_LED_GPIO, &dog_display_);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        if (dog_display_) {
            return dog_display_;
        }
        return fallback_display_;
    }
};

DECLARE_BOARD(ZfsnDogBoard);

