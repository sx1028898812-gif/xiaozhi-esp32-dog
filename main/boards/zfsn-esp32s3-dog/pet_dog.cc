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
    int adjusted = std::clamp(angle + trim_rb_, 0, 180);
    uint32_t duty = AngleToDuty(180 - adjusted);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Channel 1: Left Front (GPIO 17) - direct angle
void PetDog::SetLeftFrontAngle(int angle) {
    int adjusted = std::clamp(angle + trim_lf_, 0, 180);
    uint32_t duty = AngleToDuty(adjusted);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// Channel 2: Right Front (GPIO 13) - physically mirrored (180 - angle)
void PetDog::SetRightFrontAngle(int angle) {
    int adjusted = std::clamp(angle + trim_rf_, 0, 180);
    uint32_t duty = AngleToDuty(180 - adjusted);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

// Channel 3: Left Back (GPIO 18) - direct angle
void PetDog::SetLeftBackAngle(int angle) {
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
        DogActionType::kJump,
        DogActionType::kSitdown,
        DogActionType::kWalkForward,
    };
    uint32_t idx = esp_random() % (sizeof(kRandomActions) / sizeof(kRandomActions[0]));
    switch (kRandomActions[idx]) {
        case DogActionType::kPetWave:     PetWave(); break;
        case DogActionType::kSwing:       Swing(4, 30); break;
        case DogActionType::kStretch:     Stretch(); break;
        case DogActionType::kScratching:  Scratching(); break;
        case DogActionType::kJump:        Jump(); break;
        case DogActionType::kSitdown:     Sitdown(); break;
        case DogActionType::kWalkForward: WalkForward(2); break;
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
    SetLeftFrontAngle(90);
    SetRightFrontAngle(90);
    SetLeftBackAngle(90);
    SetRightBackAngle(90);
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
        if (WaitOrStop(6)) return;
    }
    // Fixed 2.1 bug: actually use 'steps' for the number of swing cycles
    for (int temp = 0; temp < steps; temp++) {
        for (int i = 90 - amp; i < 90 + amp; i += 2) {
            SetLeftFrontAngle(i);
            SetRightFrontAngle(i);
            SetLeftBackAngle(i);
            SetRightBackAngle(i);
            if (WaitOrStop(6)) return;
        }
        for (int i = 90 + amp; i > 90 - amp; i -= 2) {
            SetLeftFrontAngle(i);
            SetRightFrontAngle(i);
            SetLeftBackAngle(i);
            SetRightBackAngle(i);
            if (WaitOrStop(6)) return;
        }
    }
    RunStand();
}

void PetDog::RunPetWave() {
    RunSitdown();
    if (WaitOrStop(150)) return;
    for (int count = 0; count < 4; count++) {
        for (int i = 135; i <= 175; i += 2) {
            SetRightFrontAngle(i);
            if (WaitOrStop(10)) return;
        }
        for (int i = 175; i >= 135; i -= 2) {
            SetRightFrontAngle(i);
            if (WaitOrStop(10)) return;
        }
    }
    RunStand();
}

void PetDog::RunJump() {
    // Prepare crouch -> explosive extension -> land -> stand
    SetLeftFrontAngle(60);
    SetRightFrontAngle(60);
    SetLeftBackAngle(45);
    SetRightBackAngle(45);
    if (WaitOrStop(200)) return;

    SetLeftFrontAngle(145);
    SetRightFrontAngle(145);
    SetLeftBackAngle(135);
    SetRightBackAngle(135);
    if (WaitOrStop(220)) return;

    RunStand();
    if (WaitOrStop(150)) return;
}

