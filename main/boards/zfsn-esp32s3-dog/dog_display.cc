#include "dog_display.h"
#include "pet_dog.h"
#include "ws2812_led.h"

#include <esp_log.h>
#include <esp_random.h>
#include <algorithm>
#include <cmath>
#include <cstring>

#define TAG "DogDisplay"
#define EYE_MODE_CHANGED_BIT (1 << 0)

static int BresenhamLine(int x1, int y1, int x2, int y2, int* out_pts, int max_Floats) {
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    int count = 0;

    while (count + 2 <= max_Floats) {
        out_pts[count] = x1;
        out_pts[count + 1] = y1;
        count += 2;
        if (x1 == x2 && y1 == y2) {
            break;
        }
        int e2 = err * 2;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    return count;
}

DogDisplay::DogDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                       int width, int height, bool mirror_x, bool mirror_y)
    : OledDisplay(panel_io, panel, width, height, mirror_x, mirror_y) {
    event_group_ = xEventGroupCreate();
}

DogDisplay::~DogDisplay() {
    if (anim_task_handle_) {
        vTaskDelete(anim_task_handle_);
    }
    if (event_group_) {
        vEventGroupDelete(event_group_);
    }
}

void DogDisplay::SetupUI() {
    if (setup_ui_called_) {
        return;
    }
    // Initialize standard 128x64 OLED UI first
    OledDisplay::SetupUI();

    {
        DisplayLockGuard lock(this);
        auto screen = lv_screen_active();

        // Full-screen overlay covering OledDisplay's default UI when in eye/firework mode
        eye_overlay_ = lv_obj_create(screen);
        lv_obj_set_size(eye_overlay_, width_, height_);
        lv_obj_set_style_radius(eye_overlay_, 0, 0);
        lv_obj_set_style_bg_color(eye_overlay_, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(eye_overlay_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(eye_overlay_, 0, 0);
        lv_obj_set_style_pad_all(eye_overlay_, 0, 0);
        lv_obj_set_scrollbar_mode(eye_overlay_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_align(eye_overlay_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(eye_overlay_, LV_OBJ_FLAG_HIDDEN);

        // Create twin rounded-square eyes (LVGL 9 monochrome: lv_color_black() = lit pixel)
        eye_right_ = lv_obj_create(eye_overlay_);
        lv_obj_set_size(eye_right_, kEyeSize, kEyeSize);
        lv_obj_set_style_radius(eye_right_, kEyeRadius, 0);
        lv_obj_set_style_bg_color(eye_right_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(eye_right_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(eye_right_, 0, 0);
        lv_obj_set_style_pad_all(eye_right_, 0, 0);
        lv_obj_set_scrollbar_mode(eye_right_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_align(eye_right_, LV_ALIGN_CENTER, kEyeGap, 0);
        lv_obj_add_flag(eye_right_, LV_OBJ_FLAG_HIDDEN);

        eye_left_ = lv_obj_create(eye_overlay_);
        lv_obj_set_size(eye_left_, kEyeSize, kEyeSize);
        lv_obj_set_style_radius(eye_left_, kEyeRadius, 0);
        lv_obj_set_style_bg_color(eye_left_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(eye_left_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(eye_left_, 0, 0);
        lv_obj_set_style_pad_all(eye_left_, 0, 0);
        lv_obj_set_scrollbar_mode(eye_left_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_align(eye_left_, LV_ALIGN_CENTER, -kEyeGap, 0);
        lv_obj_add_flag(eye_left_, LV_OBJ_FLAG_HIDDEN);

        // Create firework particle dots
        for (int i = 0; i < kFireworkParticles; i++) {
            firework_dots_[i] = lv_obj_create(eye_overlay_);
            lv_obj_set_size(firework_dots_[i], 4, 4);
            lv_obj_set_style_radius(firework_dots_[i], 2, 0);
            lv_obj_set_style_bg_color(firework_dots_[i], lv_color_black(), 0);
            lv_obj_set_style_bg_opa(firework_dots_[i], LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(firework_dots_[i], 0, 0);
            lv_obj_set_style_pad_all(firework_dots_[i], 0, 0);
            lv_obj_set_scrollbar_mode(firework_dots_[i], LV_SCROLLBAR_MODE_OFF);
            lv_obj_align(firework_dots_[i], LV_ALIGN_CENTER, 0, 0);
            lv_obj_add_flag(firework_dots_[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    xTaskCreate(AnimTaskTrampoline, "dog_eye_task", 4096, this, 4, &anim_task_handle_);
}

void DogDisplay::ApplyVisibilityLocked(bool show_eyes, bool show_ui, bool show_fireworks) {
    if (eye_overlay_) {
        if (show_ui && !show_eyes && !show_fireworks) {
            lv_obj_add_flag(eye_overlay_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_remove_flag(eye_overlay_, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (eye_right_ && eye_left_) {
        if (show_eyes) {
            lv_obj_remove_flag(eye_right_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(eye_left_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(eye_right_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(eye_left_, LV_OBJ_FLAG_HIDDEN);
        }
    }
    for (int i = 0; i < kFireworkParticles; i++) {
        if (firework_dots_[i]) {
            if (show_fireworks) {
                lv_obj_remove_flag(firework_dots_[i], LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(firework_dots_[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

void DogDisplay::EnterAnimatedEyes() {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        current_mode_ = DogEyeMode::kAnimatedEyes;
    }
    xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
}

void DogDisplay::EnterSleepingEyes() {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        current_mode_ = DogEyeMode::kSleepingEyes;
    }
    xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
}

void DogDisplay::EnterNormalUI() {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        current_mode_ = DogEyeMode::kNormalUI;
    }
    xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
}

void DogDisplay::ShowFirework() {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (current_mode_ != DogEyeMode::kFireworkEffect) {
            return_mode_after_firework_ = current_mode_ == DogEyeMode::kNormalUI
                                              ? DogEyeMode::kAnimatedEyes
                                              : current_mode_;
        }
        current_mode_ = DogEyeMode::kFireworkEffect;
    }
    xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
}

void DogDisplay::SetChatMessage(const char* role, const char* content) {
    OledDisplay::SetChatMessage(role, content);
    // If system alert / activation message arrives with content, make sure Normal UI is visible
    if (role && strcmp(role, "system") == 0 && content && content[0] != '\0') {
        EnterNormalUI();
    }
}

void DogDisplay::ShowNotification(const char* notification, int duration_ms) {
    {
        DisplayLockGuard lock(this);
        if (notification_label_) {
            lv_obj_t* status_bar = lv_obj_get_parent(notification_label_);
            if (status_bar) {
                lv_obj_move_foreground(status_bar);
            }
        }
    }
    OledDisplay::ShowNotification(notification, duration_ms);
}

void DogDisplay::SyncWithAction(DogActionType action) {
    if (action == DogActionType::kSleep) {
        EnterSleepingEyes();
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (current_mode_ == DogEyeMode::kSleepingEyes) {
            current_mode_ = DogEyeMode::kAnimatedEyes;
        }
        switch (action) {
            case DogActionType::kTurnLeft:
                forced_gaze_dx_ = -15;
                forced_gaze_dy_ = 0;
                has_forced_gaze_ = true;
                current_emotion_ = "neutral";
                break;
            case DogActionType::kTurnRight:
                forced_gaze_dx_ = 15;
                forced_gaze_dy_ = 0;
                has_forced_gaze_ = true;
                current_emotion_ = "neutral";
                break;
            case DogActionType::kWalkForward:
                forced_gaze_dx_ = 0;
                forced_gaze_dy_ = -4;
                has_forced_gaze_ = true;
                current_emotion_ = "neutral";
                break;
            case DogActionType::kWalkBack:
                forced_gaze_dx_ = 0;
                forced_gaze_dy_ = 8;
                has_forced_gaze_ = true;
                current_emotion_ = "neutral";
                break;
            case DogActionType::kSitdown:
                forced_gaze_dx_ = 0;
                forced_gaze_dy_ = -6;
                has_forced_gaze_ = true;
                break;
            case DogActionType::kPetWave:
            case DogActionType::kSwing:
                has_forced_gaze_ = false;
                current_emotion_ = "happy";
                break;
            case DogActionType::kStretch:
            case DogActionType::kScratching:
                forced_gaze_dx_ = 0;
                forced_gaze_dy_ = 2;
                has_forced_gaze_ = true;
                current_emotion_ = "happy";
                break;
            case DogActionType::kJump:
                forced_gaze_dx_ = 0;
                forced_gaze_dy_ = -8;
                has_forced_gaze_ = true;
                current_emotion_ = "surprised";
                break;
            case DogActionType::kStand:
            default:
                has_forced_gaze_ = false;
                current_emotion_ = "neutral";
                break;
        }
    }
    xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
}

bool DogDisplay::SetEmotion(const char* emotion) {
    bool res = OledDisplay::SetEmotion(emotion);
    if (!emotion) {
        return res;
    }

    std::string emo(emotion);
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        current_emotion_ = emo;
        has_forced_gaze_ = false;
    }
    // Immediately wake up eye animation loop so eye shape updates without delay
    xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);

    // Emotion -> Limb Action Linkage (WS2812 LED is NOT auto-triggered here; LED is strictly voice-controlled)
    auto& dog = PetDog::GetInstance();
    if (dog.GetEmotionSync() && !dog.IsBusy()) {
        if (emo == "happy" || emo == "laughing" || emo == "funny" || emo == "loving" || emo == "winking") {
            dog.Swing(3, 25);
        } else if (emo == "cool" || emo == "confident" || emo == "kissy") {
            dog.PetWave();
        } else if (emo == "surprised" || emo == "shocked") {
            dog.Jump();
        } else if (emo == "sad" || emo == "crying" || emo == "embarrassed") {
            dog.Sitdown();
        } else if (emo == "sleepy") {
            dog.Sleep();
        }
    }

    return res;
}

bool DogDisplay::WaitInterruptible(int ms) {
    EventBits_t bits = xEventGroupWaitBits(
        event_group_, EYE_MODE_CHANGED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(ms));
    return (bits & EYE_MODE_CHANGED_BIT) != 0;
}

void DogDisplay::MoveEyesSmooth(int target_x1, int target_y1, int target_x2, int target_y2) {
    int pts1[200];
    int pts2[200];
    int n1 = BresenhamLine(cur_x1_, cur_y1_, target_x1, target_y1, pts1, 200);
    int n2 = BresenhamLine(cur_x2_, cur_y2_, target_x2, target_y2, pts2, 200);
    int n = std::min(n1, n2);

    for (int i = 0; i < n; i += 2) {
        if (xEventGroupGetBits(event_group_) & EYE_MODE_CHANGED_BIT) {
            return;
        }
        {
            DisplayLockGuard lock(this);
            if (eye_right_ && eye_left_) {
                lv_obj_align(eye_right_, LV_ALIGN_CENTER, pts1[i], pts1[i + 1]);
                lv_obj_align(eye_left_, LV_ALIGN_CENTER, pts2[i], pts2[i + 1]);
            }
        }
        if (WaitInterruptible(12)) {
            return;
        }
    }
    cur_x1_ = target_x1;
    cur_y1_ = target_y1;
    cur_x2_ = target_x2;
    cur_y2_ = target_y2;
}

void DogDisplay::CloseEyesSmooth() {
    for (int h = kEyeSize; h >= 10; h -= 3) {
        if (xEventGroupGetBits(event_group_) & EYE_MODE_CHANGED_BIT) {
            return;
        }
        {
            DisplayLockGuard lock(this);
            if (eye_right_ && eye_left_) {
                int w = kEyeSize + static_cast<int>(h * 0.1f);
                lv_obj_set_size(eye_right_, w, h);
                lv_obj_set_size(eye_left_, w, h);
            }
        }
        if (WaitInterruptible(15)) {
            return;
        }
    }
}

void DogDisplay::OpenEyesSmooth(int target_height, int target_width, int radius) {
    for (int h = 10; h <= target_height; h += 3) {
        if (xEventGroupGetBits(event_group_) & EYE_MODE_CHANGED_BIT) {
            return;
        }
        {
            DisplayLockGuard lock(this);
            if (eye_right_ && eye_left_) {
                lv_obj_set_style_radius(eye_right_, radius, 0);
                lv_obj_set_style_radius(eye_left_, radius, 0);
                lv_obj_set_size(eye_right_, target_width, h);
                lv_obj_set_size(eye_left_, target_width, h);
            }
        }
        if (WaitInterruptible(15)) {
            return;
        }
    }
}

void DogDisplay::BlinkOnce() {
    CloseEyesSmooth();
    OpenEyesSmooth();
}

void DogDisplay::RunFireworkAnimation() {
    static const float kAngles[kFireworkParticles] = {
        0.0f, 0.5236f, 1.0472f, 1.5708f, 2.0944f, 2.6180f,
        3.1416f, 3.6652f, 4.1888f, 4.7124f, 5.2360f, 5.7596f
    };
    static const int kCenterOffsets[3][2] = {
        {0, 0}, {-28, -8}, {28, -6}
    };

    {
        DisplayLockGuard lock(this);
        ApplyVisibilityLocked(false, false, true);
    }

    for (int burst = 0; burst < 3; burst++) {
        int cx = kCenterOffsets[burst][0];
        int cy = kCenterOffsets[burst][1];
        for (int r = 2; r <= 28; r += 3) {
            if (xEventGroupGetBits(event_group_) & EYE_MODE_CHANGED_BIT) {
                return;
            }
            {
                DisplayLockGuard lock(this);
                for (int i = 0; i < kFireworkParticles; i++) {
                    int px = cx + static_cast<int>(r * std::cos(kAngles[i]));
                    int py = cy + static_cast<int>(r * std::sin(kAngles[i]));
                    if (firework_dots_[i]) {
                        lv_obj_align(firework_dots_[i], LV_ALIGN_CENTER, px, py);
                    }
                }
            }
            if (WaitInterruptible(35)) {
                return;
            }
        }
    }

    // Restore previous eye mode after firework finishes
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (current_mode_ == DogEyeMode::kFireworkEffect) {
            current_mode_ = return_mode_after_firework_;
        }
    }
}

void DogDisplay::AnimTaskTrampoline(void* arg) {
    static_cast<DogDisplay*>(arg)->AnimTaskLoop();
}

void DogDisplay::AnimTaskLoop() {
    DogEyeMode last_mode = DogEyeMode::kNormalUI;

    while (true) {
        DogEyeMode mode;
        std::string emotion;
        bool forced_gaze = false;
        int forced_dx = 0;
        int forced_dy = 0;
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            mode = current_mode_;
            emotion = current_emotion_;
            forced_gaze = has_forced_gaze_;
            forced_dx = forced_gaze_dx_;
            forced_dy = forced_gaze_dy_;
        }
        xEventGroupClearBits(event_group_, EYE_MODE_CHANGED_BIT);

        if (mode == DogEyeMode::kNormalUI) {
            {
                DisplayLockGuard lock(this);
                ApplyVisibilityLocked(false, true, false);
            }
            last_mode = mode;
            xEventGroupWaitBits(event_group_, EYE_MODE_CHANGED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
            continue;
        }

        if (mode == DogEyeMode::kFireworkEffect) {
            RunFireworkAnimation();
            last_mode = DogEyeMode::kFireworkEffect;
            continue;
        }

        if (mode == DogEyeMode::kSleepingEyes) {
            if (last_mode != DogEyeMode::kSleepingEyes) {
                {
                    DisplayLockGuard lock(this);
                    ApplyVisibilityLocked(true, false, false);
                }
                MoveEyesSmooth(kEyeGap, 4, -kEyeGap, 4);
                CloseEyesSmooth();
                last_mode = DogEyeMode::kSleepingEyes;
            }
            xEventGroupWaitBits(event_group_, EYE_MODE_CHANGED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
            continue;
        }

        // DogEyeMode::kAnimatedEyes
        if (last_mode != DogEyeMode::kAnimatedEyes) {
            {
                DisplayLockGuard lock(this);
                ApplyVisibilityLocked(true, false, false);
            }
            OpenEyesSmooth();
            last_mode = DogEyeMode::kAnimatedEyes;
        }

        // Shape adjustment based on emotion
        if (emotion == "happy" || emotion == "laughing" || emotion == "loving" || emotion == "funny") {
            OpenEyesSmooth(20, kEyeSize + 4, 14);
        } else if (emotion == "surprised" || emotion == "shocked") {
            OpenEyesSmooth(kEyeSize + 6, kEyeSize + 4, 18);
        } else if (emotion == "sad" || emotion == "crying" || emotion == "embarrassed") {
            OpenEyesSmooth(22, kEyeSize - 4, 6);
        } else if (emotion == "cool" || emotion == "confident" || emotion == "winking") {
            OpenEyesSmooth(28, kEyeSize + 2, 8);
        } else {
            OpenEyesSmooth(kEyeSize, kEyeSize, kEyeRadius);
        }

        // Gaze offset: follow action direction if forced, else random (-14..+14 in X, -12..+12 in Y)
        int dx = forced_gaze ? forced_dx : (static_cast<int>(esp_random() % 29) - 14);
        int dy = forced_gaze ? forced_dy : (static_cast<int>(esp_random() % 25) - 12);
        MoveEyesSmooth(kEyeGap + dx, dy, -kEyeGap + dx, dy);

        int wait_sec = 1 + static_cast<int>(esp_random() % 3);
        if (WaitInterruptible(wait_sec * 1000)) {
            continue;
        }

        if ((esp_random() % 100) < 80) {
            BlinkOnce();
        }
    }
}

