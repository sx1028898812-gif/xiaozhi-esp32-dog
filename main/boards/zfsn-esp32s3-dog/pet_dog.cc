#include "pet_dog.h"
#include "config.h"
#include "settings.h"

#include <esp_log.h>
#include <esp_random.h>
#include <cJSON.h>
#include <algorithm>
#include <cmath>

#define TAG "PetDog"

#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180
#define SERVO_PWM_FREQ   50
#define SERVO_MIN_DUTY   205   // 0.5ms pulse at 50Hz, 13-bit (8191)
#define SERVO_MAX_DUTY   1065  // 2.5ms pulse at 50Hz, 13-bit (8191)

#define STOP_ACTION_BIT  (1 << 0)

static uint32_t AngleToDuty(int angle) {
    int clamped = std::clamp(angle, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE);
    return SERVO_MIN_DUTY + ((SERVO_MAX_DUTY - SERVO_MIN_DUTY) * clamped) / SERVO_MAX_DEGREE;
}

PetDog::PetDog() {
    event_group_ = xEventGroupCreate();
    action_queue_ = xQueueCreate(24, sizeof(DogActionParams));
    LoadSettings();
    InitPwm();

    xTaskCreate(ActionTaskTrampoline, "pet_dog_task", 4096, this, 5, &action_task_handle_);

    // Initial posture on boot
    Stand();
    if (boot_animation_) {
        PetWave();
    }
}

PetDog::~PetDog() {
    if (action_task_handle_) {
        vTaskDelete(action_task_handle_);
    }
    if (action_queue_) {
        vQueueDelete(action_queue_);
    }
    if (event_group_) {
        vEventGroupDelete(event_group_);
    }
}

void PetDog::LoadSettings() {
    Settings trim_settings("dog_trims", false);
    trim_lf_ = trim_settings.GetInt("left_front", 0);
    trim_rf_ = trim_settings.GetInt("right_front", 0);
    trim_lb_ = trim_settings.GetInt("left_back", 0);
    trim_rb_ = trim_settings.GetInt("right_back", 0);
    ESP_LOGI(TAG, "Loaded servo trims: LF=%d, RF=%d, LB=%d, RB=%d",
             trim_lf_, trim_rf_, trim_lb_, trim_rb_);

    Settings dog_settings("petdog", false);
    free_explore_ = dog_settings.GetBool("free_explore", false);
    boot_animation_ = dog_settings.GetBool("boot_animation", true);
    emotion_sync_ = dog_settings.GetBool("emotion_sync", true);
}

void PetDog::InitPwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_1,
        .freq_hz          = SERVO_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    const int gpios[4] = {
        SERVO_RIGHT_BACK_GPIO,  // LEDC_CHANNEL_0 -> GPIO 14
        SERVO_LEFT_FRONT_GPIO,  // LEDC_CHANNEL_1 -> GPIO 17
        SERVO_RIGHT_FRONT_GPIO, // LEDC_CHANNEL_2 -> GPIO 13
        SERVO_LEFT_BACK_GPIO,   // LEDC_CHANNEL_3 -> GPIO 18
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num   = gpios[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = static_cast<ledc_channel_t>(LEDC_CHANNEL_0 + i),
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_1,
            .duty       = 0,
            .hpoint     = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags      = { .output_invert = 0 },
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
}

// Channel 0: Right Back (GPIO 14) - physically mirrored (180 - angle)
void PetDog::SetRightBackAngle(int angle) {
    cur_rb_ = angle;
    int adjusted = std::clamp(angle + trim_rb_, 0, 180);
    uint32_t duty = AngleToDuty(180 - adjusted);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Channel 1: Left Front (GPIO 17) - direct angle
void PetDog::SetLeftFrontAngle(int angle) {
    cur_lf_ = angle;
    int adjusted = std::clamp(angle + trim_lf_, 0, 180);
    uint32_t duty = AngleToDuty(adjusted);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// Channel 2: Right Front (GPIO 13) - physically mirrored (180 - angle)
void PetDog::SetRightFrontAngle(int angle) {
    cur_rf_ = angle;
    int adjusted = std::clamp(angle + trim_rf_, 0, 180);
    uint32_t duty = AngleToDuty(180 - adjusted);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

// Channel 3: Left Back (GPIO 18) - direct angle
void PetDog::SetLeftBackAngle(int angle) {
    cur_lb_ = angle;
    int adjusted = std::clamp(angle + trim_lb_, 0, 180);
    uint32_t duty = AngleToDuty(adjusted);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void PetDog::ReleaseServos() {
    for (int i = 0; i < 4; i++) {
        ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(LEDC_CHANNEL_0 + i), 0);
    }
}

bool PetDog::WaitOrStop(int duration_ms) {
    if (duration_ms <= 0) {
        return (xEventGroupGetBits(event_group_) & STOP_ACTION_BIT) != 0;
    }
    EventBits_t bits = xEventGroupWaitBits(
        event_group_, STOP_ACTION_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(duration_ms));
    return (bits & STOP_ACTION_BIT) != 0;
}

void PetDog::QueueAction(const DogActionParams& params, bool clear_existing) {
    if (clear_existing) {
        xEventGroupSetBits(event_group_, STOP_ACTION_BIT);
        xQueueReset(action_queue_);
    }
    xQueueSend(action_queue_, &params, 0);
}

void PetDog::Stand() {
    QueueAction({DogActionType::kStand, 1, 0, 90, 90, 90, 90, 0, true}, false);
}

void PetDog::Sitdown() {
    QueueAction({DogActionType::kSitdown, 1, 0, 90, 90, 30, 30, 0, true}, false);
}

void PetDog::Sleep() {
    QueueAction({DogActionType::kSleep, 1, 0, 180, 180, 0, 0, 0, false}, false);
}

void PetDog::WalkForward(int steps) {
    QueueAction({DogActionType::kWalkForward, std::max(1, steps), 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::WalkBack(int steps) {
    QueueAction({DogActionType::kWalkBack, std::max(1, steps), 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::TurnLeft(int steps) {
    QueueAction({DogActionType::kTurnLeft, std::max(1, steps), 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::TurnRight(int steps) {
    QueueAction({DogActionType::kTurnRight, std::max(1, steps), 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Stretch() {
    QueueAction({DogActionType::kStretch, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Scratching() {
    QueueAction({DogActionType::kScratching, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Swing(int steps, int angle) {
    QueueAction({DogActionType::kSwing, std::clamp(steps, 1, 50), std::clamp(angle, 10, 60), 0, 0, 0, 0, 0, true}, false);
}

void PetDog::PetWave() {
    QueueAction({DogActionType::kPetWave, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Jump() {
    QueueAction({DogActionType::kJump, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Surprised() {
    QueueAction({DogActionType::kSurprised, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Sleepy() {
    QueueAction({DogActionType::kSleepy, 1, 0, 0, 0, 0, 0, 0, false}, false);
}

void PetDog::PlayBow() {
    QueueAction({DogActionType::kPlayBow, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Crying() {
    QueueAction({DogActionType::kCrying, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::AngryStomp() {
    QueueAction({DogActionType::kAngryStomp, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::PushUp(int count) {
    QueueAction({DogActionType::kPushUp, std::clamp(count, 1, 10), 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::ActCute() {
    QueueAction({DogActionType::kActCute, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Pee() {
    QueueAction({DogActionType::kPee, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::HeadTilt() {
    QueueAction({DogActionType::kHeadTilt, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Moonwalk(int steps) {
    QueueAction({DogActionType::kMoonwalk, std::clamp(steps, 1, 10), 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Bow() {
    QueueAction({DogActionType::kBow, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::TippyTaps() {
    QueueAction({DogActionType::kTippyTaps, 1, 0, 0, 0, 0, 0, 0, true}, false);
}

void PetDog::Stop() {
    xEventGroupSetBits(event_group_, STOP_ACTION_BIT);
    xQueueReset(action_queue_);
    QueueAction({DogActionType::kStand, 1, 0, 90, 90, 90, 90, 0, true}, false);
}

void PetDog::ExecuteRandomAction() {
    static const DogActionType kRandomActions[] = {
        DogActionType::kPetWave,
        DogActionType::kSwing,
        DogActionType::kStretch,
        DogActionType::kScratching,
        DogActionType::kPlayBow,
        DogActionType::kSurprised,
        DogActionType::kJump,
        DogActionType::kPushUp,
        DogActionType::kActCute,
        DogActionType::kPee,
        DogActionType::kHeadTilt,
        DogActionType::kMoonwalk,
        DogActionType::kBow,
        DogActionType::kTippyTaps,
    };
    uint32_t idx = esp_random() % (sizeof(kRandomActions) / sizeof(kRandomActions[0]));
    switch (kRandomActions[idx]) {
        case DogActionType::kPetWave:     PetWave(); break;
        case DogActionType::kSwing:       Swing(4, 30); break;
        case DogActionType::kStretch:     Stretch(); break;
        case DogActionType::kScratching:  Scratching(); break;
        case DogActionType::kPlayBow:     PlayBow(); break;
        case DogActionType::kSurprised:   Surprised(); break;
        case DogActionType::kJump:        Jump(); break;
        case DogActionType::kPushUp:      PushUp(3); break;
        case DogActionType::kActCute:     ActCute(); break;
        case DogActionType::kPee:         Pee(); break;
        case DogActionType::kHeadTilt:    HeadTilt(); break;
        case DogActionType::kMoonwalk:    Moonwalk(3); break;
        case DogActionType::kBow:         Bow(); break;
        case DogActionType::kTippyTaps:   TippyTaps(); break;
        default:                          Swing(4, 30); break;
    }
}

bool PetDog::ExecuteServoSequence(const std::string& sequence_json, std::string& error_msg) {
    cJSON* root = cJSON_Parse(sequence_json.c_str());
    if (!root) {
        error_msg = "Failed to parse JSON sequence";
        return false;
    }

    cJSON* seq_array = root;
    if (cJSON_IsObject(root)) {
        seq_array = cJSON_GetObjectItem(root, "sequence");
    }
    if (!cJSON_IsArray(seq_array)) {
        cJSON_Delete(root);
        error_msg = "Expected a JSON array of keyframes or object with 'sequence' array";
        return false;
    }

    int count = cJSON_GetArraySize(seq_array);
    if (count <= 0 || count > 20) {
        cJSON_Delete(root);
        error_msg = "Sequence length must be between 1 and 20 keyframes";
        return false;
    }

    for (int i = 0; i < count; i++) {
        cJSON* item = cJSON_GetArrayItem(seq_array, i);
        if (!cJSON_IsObject(item)) {
            continue;
        }
        auto get_int = [&](const char* key, int def_val) -> int {
            cJSON* node = cJSON_GetObjectItem(item, key);
            return cJSON_IsNumber(node) ? node->valueint : def_val;
        };

        DogActionParams frame = {};
        frame.type = DogActionType::kCustomKeyframe;
        frame.lf = std::clamp(get_int("lf", 90), 0, 180);
        frame.rf = std::clamp(get_int("rf", 90), 0, 180);
        frame.lb = std::clamp(get_int("lb", 90), 0, 180);
        frame.rb = std::clamp(get_int("rb", 90), 0, 180);
        frame.delay_ms = std::clamp(get_int("d", get_int("delay_ms", 250)), 40, 3000);
        cJSON* hold_node = cJSON_GetObjectItem(item, "hold");
        frame.hold = cJSON_IsBool(hold_node) ? cJSON_IsTrue(hold_node) : true;

        QueueAction(frame, false);
    }

    cJSON_Delete(root);
    return true;
}

bool PetDog::SetTrim(const std::string& servo_type, int trim_value) {
    int clamped = std::clamp(trim_value, -30, 30);
    Settings settings("dog_trims", true);

    if (servo_type == "left_front" || servo_type == "lf") {
        trim_lf_ = clamped;
        settings.SetInt("left_front", clamped);
    } else if (servo_type == "right_front" || servo_type == "rf") {
        trim_rf_ = clamped;
        settings.SetInt("right_front", clamped);
    } else if (servo_type == "left_back" || servo_type == "lb") {
        trim_lb_ = clamped;
        settings.SetInt("left_back", clamped);
    } else if (servo_type == "right_back" || servo_type == "rb") {
        trim_rb_ = clamped;
        settings.SetInt("right_back", clamped);
    } else if (servo_type == "reset" || servo_type == "all") {
        trim_lf_ = trim_rf_ = trim_lb_ = trim_rb_ = 0;
        settings.SetInt("left_front", 0);
        settings.SetInt("right_front", 0);
        settings.SetInt("left_back", 0);
        settings.SetInt("right_back", 0);
    } else {
        return false;
    }

    // Apply immediately in stand posture so user can visually verify calibration
    Stand();
    return true;
}

std::map<std::string, int> PetDog::GetTrims() const {
    return {
        {"left_front", trim_lf_},
        {"right_front", trim_rf_},
        {"left_back", trim_lb_},
        {"right_back", trim_rb_},
    };
}

void PetDog::SetFreeExplore(bool enable) {
    free_explore_ = enable;
    Settings settings("petdog", true);
    settings.SetBool("free_explore", enable);
}

bool PetDog::ToggleFreeExplore() {
    SetFreeExplore(!free_explore_);
    return free_explore_;
}

void PetDog::SetBootAnimation(bool enable) {
    boot_animation_ = enable;
    Settings settings("petdog", true);
    settings.SetBool("boot_animation", enable);
}

bool PetDog::ToggleBootAnimation() {
    SetBootAnimation(!boot_animation_);
    return boot_animation_;
}

void PetDog::SetEmotionSync(bool enable) {
    emotion_sync_ = enable;
    Settings settings("petdog", true);
    settings.SetBool("emotion_sync", enable);
}

bool PetDog::ToggleEmotionSync() {
    SetEmotionSync(!emotion_sync_);
    return emotion_sync_;
}

// 50Hz Cosine Ease-in-out Smooth Motion Engine
bool PetDog::MoveTo(int target_lf, int target_rf, int target_lb, int target_rb, int duration_ms) {
    if (duration_ms <= 0) duration_ms = 20;
    int steps = duration_ms / 20;
    if (steps < 1) steps = 1;

    int start_lf = cur_lf_;
    int start_rf = cur_rf_;
    int start_lb = cur_lb_;
    int start_rb = cur_rb_;

    for (int i = 1; i <= steps; ++i) {
        float progress = static_cast<float>(i) / steps;
        float ease = 0.5f * (1.0f - cosf(progress * 3.14159265f));

        int lf = std::round(start_lf + (target_lf - start_lf) * ease);
        int rf = std::round(start_rf + (target_rf - start_rf) * ease);
        int lb = std::round(start_lb + (target_lb - start_lb) * ease);
        int rb = std::round(start_rb + (target_rb - start_rb) * ease);

        SetLeftFrontAngle(lf);
        SetRightFrontAngle(rf);
        SetLeftBackAngle(lb);
        SetRightBackAngle(rb);

        if (WaitOrStop(20)) {
            return false;
        }
    }
    return true;
}

void PetDog::ActionTaskTrampoline(void* arg) {
    static_cast<PetDog*>(arg)->ActionTaskLoop();
}

void PetDog::ActionTaskLoop() {
    DogActionParams params;
    while (true) {
        if (xQueueReceive(action_queue_, &params, portMAX_DELAY) == pdTRUE) {
            xEventGroupClearBits(event_group_, STOP_ACTION_BIT);
            is_busy_ = true;

            if (action_callback_) {
                action_callback_(params.type);
            }

            switch (params.type) {
                case DogActionType::kStand:
                    RunStand();
                    break;
                case DogActionType::kSitdown:
                    RunSitdown();
                    break;
                case DogActionType::kSleep:
                    RunSleep();
                    break;
                case DogActionType::kWalkForward:
                    RunWalkForward(params.steps);
                    break;
                case DogActionType::kWalkBack:
                    RunWalkBack(params.steps);
                    break;
                case DogActionType::kTurnLeft:
                    RunTurnLeft(params.steps);
                    break;
                case DogActionType::kTurnRight:
                    RunTurnRight(params.steps);
                    break;
                case DogActionType::kStretch:
                    RunStretch();
                    break;
                case DogActionType::kScratching:
                    RunScratching();
                    break;
                case DogActionType::kSwing:
                    RunSwing(params.steps, params.angle);
                    break;
                case DogActionType::kPetWave:
                    RunPetWave();
                    break;
                case DogActionType::kJump:
                    RunJump();
                    break;
                case DogActionType::kSurprised:
                    RunSurprised();
                    break;
                case DogActionType::kSleepy:
                    RunSleepy();
                    break;
                case DogActionType::kPlayBow:
                    RunPlayBow();
                    break;
                case DogActionType::kCrying:
                    RunCrying();
                    break;
                case DogActionType::kAngryStomp:
                    RunAngryStomp();
                    break;
                case DogActionType::kPushUp:
                    RunPushUp(params.steps);
                    break;
                case DogActionType::kActCute:
                    RunActCute();
                    break;
                case DogActionType::kPee:
                    RunPee();
                    break;
                case DogActionType::kHeadTilt:
                    RunHeadTilt();
                    break;
                case DogActionType::kMoonwalk:
                    RunMoonwalk(params.steps);
                    break;
                case DogActionType::kBow:
                    RunBow();
                    break;
                case DogActionType::kTippyTaps:
                    RunTippyTaps();
                    break;
                case DogActionType::kCustomKeyframe:
                    SetLeftFrontAngle(params.lf);
                    SetRightFrontAngle(params.rf);
                    SetLeftBackAngle(params.lb);
                    SetRightBackAngle(params.rb);
                    WaitOrStop(params.delay_ms);
                    if (!params.hold) {
                        ReleaseServos();
                    }
                    break;
            }

            is_busy_ = (uxQueueMessagesWaiting(action_queue_) > 0);
            if (!is_busy_ && current_posture_ == DogActionType::kStand &&
                params.type != DogActionType::kStand) {
                if (action_callback_) {
                    action_callback_(DogActionType::kStand);
                }
            }
        }
    }
}

void PetDog::RunStand() {
    if (!MoveTo(90, 90, 90, 90, 1000)) return;
    current_posture_ = DogActionType::kStand;
}

void PetDog::RunSitdown() {
    SetLeftFrontAngle(90);
    SetRightFrontAngle(90);
    SetLeftBackAngle(30);
    SetRightBackAngle(30);
    current_posture_ = DogActionType::kSitdown;
}

void PetDog::RunSleep() {
    SetLeftFrontAngle(180);
    SetRightFrontAngle(180);
    SetLeftBackAngle(0);
    SetRightBackAngle(0);
    current_posture_ = DogActionType::kSleep;
    // Wait up to 3 seconds then release servo PWM torque to prevent heating and buzzing
    if (!WaitOrStop(3000)) {
        ReleaseServos();
    }
}

// Option A: Silky Smooth Melt into Sleep
void PetDog::RunSleepy() {
    // Stage 1: Slight forward droop / drowsy nod
    if (!MoveTo(108, 108, 90, 90, 500)) return;
    // Stage 2: Rear butt sinks forward first
    if (!MoveTo(108, 108, 0, 0, 600)) return;
    // Stage 3: Front legs fold backward and melt flat onto ground
    if (!MoveTo(180, 180, 0, 0, 700)) return;
    current_posture_ = DogActionType::kSleep;
    // Rest for 3 seconds before torque release
    if (WaitOrStop(3000)) return;
    ReleaseServos();
}

// Surprised: 45° outward splay fast drop -> shock freeze -> smooth recovery
void PetDog::RunSurprised() {
    if (!MoveTo(45, 45, 135, 135, 260)) return;
    if (WaitOrStop(1400)) return;
    MoveTo(90, 90, 90, 90, 800);
    current_posture_ = DogActionType::kStand;
}

// Play Bow (嬉笑/邀玩下犬式): Chest down, hips high, wiggle butt
void PetDog::RunPlayBow() {
    if (!MoveTo(30, 30, 95, 95, 550)) return;
    if (WaitOrStop(250)) return;
    for (int i = 0; i < 3; ++i) {
        if (!MoveTo(30, 30, 80, 110, 300)) return;
        if (!MoveTo(30, 30, 110, 80, 300)) return;
    }
    MoveTo(90, 90, 90, 90, 750);
    current_posture_ = DogActionType::kStand;
}

// Crying (委屈抽泣): Inward slump, shivering tremor, slow rise
void PetDog::RunCrying() {
    if (!MoveTo(125, 125, 30, 30, 500)) return;
    for (int i = 0; i < 3; ++i) {
        if (!MoveTo(120, 120, 35, 35, 120)) return;
        if (!MoveTo(125, 125, 30, 30, 120)) return;
    }
    MoveTo(90, 90, 90, 90, 900);
    current_posture_ = DogActionType::kStand;
}

// Angry Stomp (生气跺脚): alternate front paw stomping
void PetDog::RunAngryStomp() {
    for (int i = 0; i < 2; ++i) {
        if (!MoveTo(50, 90, 90, 90, 180)) return;
        if (!MoveTo(90, 90, 90, 90, 140)) return;
        if (!MoveTo(90, 50, 90, 90, 180)) return;
        if (!MoveTo(90, 90, 90, 90, 140)) return;
    }
    current_posture_ = DogActionType::kStand;
}

// Push Up (猛男俯卧撑): Hind legs braced, front legs smoothly press down & up with realistic cadence
void PetDog::RunPushUp(int count) {
    // Smoothly lower into plank/brace stance first
    if (!MoveTo(85, 85, 55, 55, 650)) return;
    if (WaitOrStop(250)) return;

    int reps = std::clamp(count, 1, 10);
    for (int i = 0; i < reps; ++i) {
        // Controlled descent under gravity
        if (!MoveTo(150, 150, 50, 50, 550)) return;
        if (WaitOrStop(180)) return;
        // Smooth power push back up
        if (!MoveTo(80, 80, 58, 58, 600)) return;
        if (WaitOrStop(220)) return;
    }

    // Smooth recovery to neutral stand
    MoveTo(90, 90, 90, 90, 850);
    current_posture_ = DogActionType::kStand;
}

// Act Cute (撒娇求抱抱): Smooth sit -> gentle alternating upper-body sway -> front paws reach out
void PetDog::RunActCute() {
    // 1. Gently sit down
    if (!MoveTo(90, 90, 30, 30, 650)) return;
    if (WaitOrStop(250)) return;

    // 2. Softly sway shoulders left and right (3 cycles)
    for (int i = 0; i < 3; ++i) {
        if (!MoveTo(60, 115, 30, 35, 480)) return;
        if (!MoveTo(115, 60, 35, 30, 480)) return;
    }

    // 3. Reach both front paws forward pleadingly ("hug me!")
    if (!MoveTo(45, 45, 30, 30, 550)) return;
    if (WaitOrStop(600)) return;

    // 4. Gentle little paw flutter
    if (!MoveTo(35, 55, 30, 30, 360)) return;
    if (!MoveTo(55, 35, 30, 30, 360)) return;
    if (WaitOrStop(250)) return;

    // 5. Smoothly rise back to standing
    MoveTo(90, 90, 90, 90, 850);
    current_posture_ = DogActionType::kStand;
}

// Pee (搞笑翘腿标记领地): Shift center of gravity first -> slowly lift right hind leg -> gentle shake -> lower
void PetDog::RunPee() {
    // 1. Shift weight onto left legs & right front leg so the body stays balanced
    if (!MoveTo(82, 78, 72, 95, 600)) return;
    if (WaitOrStop(250)) return;

    // 2. Slowly and smoothly raise the right hind leg high backward
    if (!MoveTo(82, 78, 70, 172, 680)) return;
    if (WaitOrStop(400)) return;

    // 3. Three realistic, unhurried shakes at the top
    for (int i = 0; i < 3; ++i) {
        if (!MoveTo(82, 78, 70, 152, 340)) return;
        if (!MoveTo(82, 78, 70, 172, 340)) return;
    }
    if (WaitOrStop(300)) return;

    // 4. Slowly lower the leg and restore center of gravity
    MoveTo(90, 90, 90, 90, 800);
    current_posture_ = DogActionType::kStand;
}

// Head Tilt (好奇歪头杀): Smooth lateral body lean left -> pause & ponder -> lean right -> pause -> recover
void PetDog::RunHeadTilt() {
    // 1. Lean left side down, right side up (curious tilt left)
    if (!MoveTo(122, 68, 118, 72, 620)) return;
    if (WaitOrStop(700)) return;

    // 2. Smoothly transition across center to tilt right
    if (!MoveTo(68, 122, 72, 118, 720)) return;
    if (WaitOrStop(700)) return;

    // 3. Slight second tilt left for extra expressiveness
    if (!MoveTo(115, 75, 112, 78, 600)) return;
    if (WaitOrStop(500)) return;

    // 4. Smoothly return to upright stand
    MoveTo(90, 90, 90, 90, 680);
    current_posture_ = DogActionType::kStand;
}

// Moonwalk (太空步/机械滑步): Fluid 4-phase wave gait with smooth weight transfer
void PetDog::RunMoonwalk(int steps) {
    if (!MoveTo(90, 90, 90, 90, 500)) return;
    int count = std::clamp(steps, 1, 10);

    for (int s = 0; s < count; ++s) {
        // Phase 1: Left front glides back while right hind slides forward
        if (!MoveTo(125, 75, 70, 120, 420)) return;
        // Phase 2: Body wave crest through mid-stance
        if (!MoveTo(105, 105, 75, 75, 360)) return;
        // Phase 3: Right front glides back while left hind slides forward
        if (!MoveTo(75, 125, 120, 70, 420)) return;
        // Phase 4: Body wave trough back to neutral
        if (!MoveTo(75, 75, 105, 105, 360)) return;
    }

    MoveTo(90, 90, 90, 90, 700);
    current_posture_ = DogActionType::kStand;
}

// Bow (恭敬鞠躬/拜年作揖): Deep respectful front bow -> gentle nod twice -> dignified rise
void PetDog::RunBow() {
    // 1. Smoothly fold front legs back to bow chest low while hind legs stay upright
    if (!MoveTo(152, 152, 85, 85, 720)) return;
    if (WaitOrStop(400)) return;

    // 2. Two polite, unhurried nods at the bottom of the bow
    for (int i = 0; i < 2; ++i) {
        if (!MoveTo(135, 135, 85, 85, 380)) return;
        if (!MoveTo(155, 155, 85, 85, 380)) return;
    }
    if (WaitOrStop(350)) return;

    // 3. Dignified, smooth rise back to standing
    MoveTo(90, 90, 90, 90, 850);
    current_posture_ = DogActionType::kStand;
}

// Tippy Taps (兴奋踏步): Rhythmic, elastic diagonal prancing steps followed by a happy little hop
void PetDog::RunTippyTaps() {
    // 1. Elastic diagonal prancing (4 cycles, 260ms per beat so servos move smoothly)
    for (int i = 0; i < 4; ++i) {
        if (!MoveTo(72, 108, 108, 72, 260)) return;
        if (!MoveTo(108, 72, 72, 108, 260)) return;
    }

    // 2. Settle briefly then do a soft happy mini-hop
    if (!MoveTo(70, 70, 60, 60, 320)) return;
    if (WaitOrStop(150)) return;
    if (!MoveTo(125, 125, 120, 120, 300)) return;
    if (WaitOrStop(200)) return;

    // 3. Smooth landing back to stand
    MoveTo(90, 90, 90, 90, 750);
    current_posture_ = DogActionType::kStand;
}

// Original 6.0 Gaits (Unchanged)
void PetDog::RunWalkForward(int steps) {
    RunStand();
    if (WaitOrStop(80)) return;
    for (int s = 0; s < steps; s++) {
        for (int i = 0; i < 45; i += 3) {
            SetLeftFrontAngle(90 + i);
            SetRightBackAngle(90 + i);
            SetRightFrontAngle(90 - i);
            SetLeftBackAngle(90 - i);
            if (WaitOrStop(8)) return;
        }
        for (int i = 45; i >= -45; i -= 3) {
            SetLeftFrontAngle(90 + i);
            SetRightBackAngle(90 + i);
            SetRightFrontAngle(90 - i);
            SetLeftBackAngle(90 - i);
            if (WaitOrStop(8)) return;
        }
        for (int i = -45; i <= 0; i += 3) {
            SetLeftFrontAngle(90 + i);
            SetRightBackAngle(90 + i);
            SetRightFrontAngle(90 - i);
            SetLeftBackAngle(90 - i);
            if (WaitOrStop(8)) return;
        }
    }
    RunStand();
}

void PetDog::RunWalkBack(int steps) {
    RunStand();
    if (WaitOrStop(80)) return;
    for (int s = 0; s < steps; s++) {
        for (int i = 0; i > -45; i -= 3) {
            SetLeftFrontAngle(90 + i);
            SetRightBackAngle(90 + i);
            SetRightFrontAngle(90 - i);
            SetLeftBackAngle(90 - i);
            if (WaitOrStop(8)) return;
        }
        for (int i = -45; i <= 45; i += 3) {
            SetLeftFrontAngle(90 + i);
            SetRightBackAngle(90 + i);
            SetRightFrontAngle(90 - i);
            SetLeftBackAngle(90 - i);
            if (WaitOrStop(8)) return;
        }
        for (int i = 45; i >= 0; i -= 3) {
            SetLeftFrontAngle(90 + i);
            SetRightBackAngle(90 + i);
            SetRightFrontAngle(90 - i);
            SetLeftBackAngle(90 - i);
            if (WaitOrStop(8)) return;
        }
    }
    RunStand();
}

void PetDog::RunTurnLeft(int steps) {
    RunStand();
    if (WaitOrStop(80)) return;
    for (int s = 0; s < steps * 2; s++) {
        SetLeftFrontAngle(45);
        SetRightBackAngle(45);
        if (WaitOrStop(120)) return;
        SetRightFrontAngle(135);
        SetLeftBackAngle(135);
        if (WaitOrStop(120)) return;
        SetLeftFrontAngle(90);
        SetRightBackAngle(90);
        if (WaitOrStop(120)) return;
        SetRightFrontAngle(90);
        SetLeftBackAngle(90);
        if (WaitOrStop(120)) return;
    }
    RunStand();
}

void PetDog::RunTurnRight(int steps) {
    RunStand();
    if (WaitOrStop(80)) return;
    for (int s = 0; s < steps * 2; s++) {
        SetRightFrontAngle(45);
        SetLeftBackAngle(45);
        if (WaitOrStop(120)) return;
        SetLeftFrontAngle(135);
        SetRightBackAngle(135);
        if (WaitOrStop(120)) return;
        SetRightFrontAngle(90);
        SetLeftBackAngle(90);
        if (WaitOrStop(120)) return;
        SetLeftFrontAngle(90);
        SetRightBackAngle(90);
        if (WaitOrStop(120)) return;
    }
    RunStand();
}

void PetDog::RunStretch() {
    RunStand();
    if (WaitOrStop(80)) return;
    for (int i = 90; i < 165; i += 2) {
        SetLeftFrontAngle(i);
        SetRightFrontAngle(i);
        if (WaitOrStop(15)) return;
    }
    for (int i = 165; i > 90; i -= 2) {
        SetLeftFrontAngle(i);
        SetRightFrontAngle(i);
        if (WaitOrStop(15)) return;
    }
    for (int i = 90; i > 15; i -= 2) {
        SetLeftBackAngle(i);
        SetRightBackAngle(i);
        if (WaitOrStop(15)) return;
    }
    for (int i = 15; i < 90; i += 2) {
        SetLeftBackAngle(i);
        SetRightBackAngle(i);
        if (WaitOrStop(15)) return;
    }
    RunStand();
}

void PetDog::RunScratching() {
    SetLeftFrontAngle(90);
    SetRightFrontAngle(90);
    SetLeftBackAngle(30);
    if (WaitOrStop(80)) return;
    for (int count = 0; count < 8; count++) {
        for (int i = 90; i <= 180; i += 5) {
            SetRightBackAngle(i);
            if (WaitOrStop(8)) return;
        }
        for (int i = 180; i >= 90; i -= 5) {
            SetRightBackAngle(i);
            if (WaitOrStop(8)) return;
        }
    }
    RunStand();
}

void PetDog::RunSwing(int steps, int angle) {
    RunStand();
    if (WaitOrStop(80)) return;

    int amp = std::clamp(angle, 10, 60);
    for (int i = 90; i > 90 - amp; i -= 2) {
        SetLeftFrontAngle(i);
        SetRightFrontAngle(i);
        SetLeftBackAngle(i);
        SetRightBackAngle(i);
        if (WaitOrStop(15)) return;
    }
    for (int temp = 0; temp < steps; temp++) {
        for (int i = 90 - amp; i < 90 + amp; i += 2) {
            SetLeftFrontAngle(i);
            SetRightFrontAngle(i);
            SetLeftBackAngle(i);
            SetRightBackAngle(i);
            if (WaitOrStop(15)) return;
        }
        for (int i = 90 + amp; i > 90 - amp; i -= 2) {
            SetLeftFrontAngle(i);
            SetRightFrontAngle(i);
            SetLeftBackAngle(i);
            SetRightBackAngle(i);
            if (WaitOrStop(15)) return;
        }
    }
    RunStand();
}

void PetDog::RunPetWave() {
    RunSitdown();
    // The logical joint convention is 0°=forward, 90°=down, 180°=back.
    // Preserve the legacy right-paw timing and mirror its old backward range forward.
    if (WaitOrStop(150)) return;
    for (int count = 0; count < 4; count++) {
        for (int i = 45; i >= 5; i -= 2) {
            SetRightFrontAngle(i);
            if (WaitOrStop(10)) return;
        }
        for (int i = 5; i <= 45; i += 2) {
            SetRightFrontAngle(i);
            if (WaitOrStop(10)) return;
        }
    }
    RunStand();
}

void PetDog::RunJump() {
    if (!MoveTo(60, 60, 45, 45, 350)) return;
    if (WaitOrStop(200)) return;

    if (!MoveTo(145, 145, 135, 135, 300)) return;
    if (WaitOrStop(250)) return;

    if (!MoveTo(90, 90, 90, 90, 700)) return;
    current_posture_ = DogActionType::kStand;
}
