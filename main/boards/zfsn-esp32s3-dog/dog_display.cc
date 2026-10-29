#include "dog_display.h"
#include "pet_dog.h"
#include "ws2812_led.h"

#include <esp_log.h>
#include <esp_random.h>
#include <cstring>
#include <vector>

#define TAG "DogDisplay"
#define EYE_MODE_CHANGED_BIT (1 << 0)

DogDisplay::DogDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                       int width, int height, bool mirror_x, bool mirror_y)
    : OledDisplay(panel_io, panel, width, height, mirror_x, mirror_y) {
    event_group_ = xEventGroupCreate();
}

DogDisplay::~DogDisplay() {
    if (anim_task_handle_) {
        vTaskDelete(anim_task_handle_);
        anim_task_handle_ = nullptr;
    }
    if (event_group_) {
        vEventGroupDelete(event_group_);
        event_group_ = nullptr;
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

        // 128x64 full-screen image object for 6.0 authentic GIF playback
        gif_image_ = lv_image_create(screen);
        lv_obj_set_size(gif_image_, 128, 64);
        lv_obj_align(gif_image_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(gif_image_, LV_OBJ_FLAG_HIDDEN);
    }

    if (!anim_task_handle_) {
        xTaskCreate(AnimTaskTrampoline, "dog_anim_task", 4096, this, 3, &anim_task_handle_);
    }
}

void DogDisplay::ApplyVisibilityLocked(bool show_gif, bool show_ui) {
    if (gif_image_) {
        if (show_gif) {
            lv_obj_remove_flag(gif_image_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(gif_image_, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (container_) {
        if (show_ui) {
            lv_obj_remove_flag(container_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(container_, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void DogDisplay::PlayGifInternalLocked(const DogGifAsset* asset) {
    if (!asset || !asset->data || asset->size == 0) {
        return;
    }

    // Stop current GIF before loading next
    if (gif_controller_) {
        gif_controller_->Stop();
        gif_controller_.reset();
    }

    memset(&current_gif_dsc_, 0, sizeof(current_gif_dsc_));
    current_gif_dsc_.header.magic = LV_IMAGE_HEADER_MAGIC;
    current_gif_dsc_.header.cf = LV_COLOR_FORMAT_RAW;
    current_gif_dsc_.header.w = 128;
    current_gif_dsc_.header.h = 64;
    current_gif_dsc_.data_size = asset->size;
    current_gif_dsc_.data = asset->data;

    gif_controller_ = std::make_unique<LvglGif>(&current_gif_dsc_);
    if (gif_controller_->IsLoaded()) {
        gif_controller_->SetFrameCallback([this]() {
            if (gif_image_ && gif_controller_) {
                lv_image_set_src(gif_image_, gif_controller_->image_dsc());
                lv_obj_invalidate(gif_image_);
            }
        });
        lv_image_set_src(gif_image_, gif_controller_->image_dsc());
        gif_controller_->Start();
        ApplyVisibilityLocked(true, false);
    } else {
        ESP_LOGE(TAG, "Failed to decode GIF: %s", asset->name);
        gif_controller_.reset();
    }
}

bool DogDisplay::PlayGif(const char* gif_name) {
    if (!gif_name) return false;
    const auto* asset = GetDogGif(gif_name);
    if (!asset) {
        ESP_LOGW(TAG, "GIF asset '%s' not found", gif_name);
        return false;
    }
    DisplayLockGuard lock(this);
    PlayGifInternalLocked(asset);
    return true;
}

bool DogDisplay::SetEmotion(const char* emotion) {
    if (!emotion) return false;
    ESP_LOGI(TAG, "DogDisplay::SetEmotion: %s", emotion);

    // Map common emotion strings to 6.0 authentic GIF names
    const char* target_gif = emotion;
    if (strcmp(emotion, "neutral") == 0 || strcmp(emotion, "standby") == 0) {
        target_gif = "blink";
    } else if (strcmp(emotion, "happy") == 0 || strcmp(emotion, "joy") == 0 || strcmp(emotion, "pleased") == 0) {
        target_gif = "happy";
    } else if (strcmp(emotion, "sad") == 0 || strcmp(emotion, "depressed") == 0) {
        target_gif = "sad";
    } else if (strcmp(emotion, "angry") == 0) {
        target_gif = "angry";
    } else if (strcmp(emotion, "sleepy") == 0 || strcmp(emotion, "sleeping") == 0) {
        target_gif = "sleeping";
    } else if (strcmp(emotion, "surprised") == 0 || strcmp(emotion, "shocked") == 0) {
        target_gif = "surprise";
    } else if (strcmp(emotion, "confused") == 0 || strcmp(emotion, "doubt") == 0 || strcmp(emotion, "thinking") == 0) {
        target_gif = "doubt";
    } else if (strcmp(emotion, "aggrieved") == 0 || strcmp(emotion, "wronged") == 0) {
        target_gif = "aggrieved";
    } else if (strcmp(emotion, "blame") == 0) {
        target_gif = "blame";
    } else if (strcmp(emotion, "contempt") == 0) {
        target_gif = "contempt";
    } else if (strcmp(emotion, "firework") == 0 || strcmp(emotion, "celebrate") == 0) {
        ShowFirework();
        return true;
    }

    const auto* asset = GetDogGif(target_gif);
    if (!asset) {
        asset = GetDogGif("blink");
    }

    if (asset) {
        {
            std::lock_guard<std::mutex> lk(mode_mutex_);
            current_emotion_ = asset->name;
            custom_emotion_active_ = (strcmp(asset->name, "blink") != 0);
            if (current_mode_ != DogEyeMode::kFireworkEffect) {
                current_mode_ = (strcmp(asset->name, "sleeping") == 0) ? DogEyeMode::kSleepingEyes : DogEyeMode::kAnimatedEyes;
            }
        }
        if (event_group_) {
            xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
        }
        DisplayLockGuard lock(this);
        PlayGifInternalLocked(asset);
        return true;
    }
    return false;
}

void DogDisplay::SetChatMessage(const char* role, const char* content) {
    OledDisplay::SetChatMessage(role, content);
    std::lock_guard<std::mutex> lock(mode_mutex_);
    if (current_mode_ == DogEyeMode::kSleepingEyes && content && strlen(content) > 0) {
        current_mode_ = DogEyeMode::kAnimatedEyes;
        custom_emotion_active_ = false;
        if (event_group_) {
            xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
        }
    }
}

void DogDisplay::ShowNotification(const char* notification, int duration_ms) {
    OledDisplay::ShowNotification(notification, duration_ms);
}

void DogDisplay::EnterAnimatedEyes() {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        current_mode_ = DogEyeMode::kAnimatedEyes;
        custom_emotion_active_ = false;
    }
    if (event_group_) {
        xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
    }
}

void DogDisplay::EnterSleepingEyes() {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        current_mode_ = DogEyeMode::kSleepingEyes;
        custom_emotion_active_ = false;
    }
    if (event_group_) {
        xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
    }
}

void DogDisplay::EnterNormalUI() {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        current_mode_ = DogEyeMode::kNormalUI;
    }
    if (event_group_) {
        xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
    }
}

void DogDisplay::ShowFirework() {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (current_mode_ != DogEyeMode::kFireworkEffect) {
            return_mode_after_firework_ = current_mode_;
        }
        current_mode_ = DogEyeMode::kFireworkEffect;
    }
    if (event_group_) {
        xEventGroupSetBits(event_group_, EYE_MODE_CHANGED_BIT);
    }
}

void DogDisplay::SyncWithAction(DogActionType action) {
    switch (action) {
        case DogActionType::kSurprised:
            SetEmotion("surprise");
            break;
        case DogActionType::kSleep:
        case DogActionType::kSleepy:
            EnterSleepingEyes();
            break;
        case DogActionType::kPlayBow:
        case DogActionType::kSwing:
        case DogActionType::kJump:
            SetEmotion("happy");
            break;
        case DogActionType::kPetWave:
            SetEmotion("wink");
            break;
        case DogActionType::kCrying:
            SetEmotion("pitiful");
            break;
        case DogActionType::kAngryStomp:
            SetEmotion("angry");
            break;
        case DogActionType::kStand:
            EnterAnimatedEyes();
            break;
        default:
            break;
    }
}

bool DogDisplay::WaitInterruptible(int ms) {
    if (!event_group_) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return false;
    }
    EventBits_t bits = xEventGroupWaitBits(event_group_, EYE_MODE_CHANGED_BIT,
                                           pdTRUE, pdFALSE, pdMS_TO_TICKS(ms));
    return (bits & EYE_MODE_CHANGED_BIT) != 0;
}

void DogDisplay::AnimTaskTrampoline(void* arg) {
    static_cast<DogDisplay*>(arg)->AnimTaskLoop();
}

void DogDisplay::AnimTaskLoop() {
    ESP_LOGI(TAG, "Dog display anim task started (6.0 authentic GIF mode)");
    static const char* kIdlePool[] = {
        "blink", "blink", "move1", "move2", "move3", "move4", "wink"
    };
    constexpr int kIdlePoolSize = sizeof(kIdlePool) / sizeof(kIdlePool[0]);

    while (true) {
        DogEyeMode mode;
        bool has_custom_emotion;
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            mode = current_mode_;
            has_custom_emotion = custom_emotion_active_;
        }

        switch (mode) {
            case DogEyeMode::kNormalUI: {
                {
                    DisplayLockGuard disp_lock(this);
                    if (gif_controller_) {
                        gif_controller_->Stop();
                    }
                    ApplyVisibilityLocked(false, true);
                }
                WaitInterruptible(1000);
                break;
            }

            case DogEyeMode::kSleepingEyes: {
                const auto* sleep_asset = GetDogGif("sleeping");
                {
                    DisplayLockGuard disp_lock(this);
                    if (!gif_controller_ || strcmp(current_emotion_.c_str(), "sleeping") != 0) {
                        current_emotion_ = "sleeping";
                        PlayGifInternalLocked(sleep_asset);
                    }
                }
                WaitInterruptible(3000);
                break;
            }

            case DogEyeMode::kFireworkEffect: {
                const auto* fw_asset = GetDogGif("firework");
                {
                    DisplayLockGuard disp_lock(this);
                    current_emotion_ = "firework";
                    PlayGifInternalLocked(fw_asset);
                }
                // Firework plays for 6 seconds, then returns to previous mode
                WaitInterruptible(6000);
                {
                    std::lock_guard<std::mutex> lock(mode_mutex_);
                    current_mode_ = return_mode_after_firework_;
                }
                break;
            }

            case DogEyeMode::kAnimatedEyes: {
                if (has_custom_emotion) {
                    // Custom emotion stays on screen for 5 seconds, then decays back to idle
                    if (WaitInterruptible(5000)) {
                        break;
                    }
                    std::lock_guard<std::mutex> lock(mode_mutex_);
                    custom_emotion_active_ = false;
                }

                // Authentic 6.0 Idle Glance: randomly pick from blink/move1~4/wink
                uint32_t idx = esp_random() % kIdlePoolSize;
                const char* next_gif = kIdlePool[idx];
                const auto* asset = GetDogGif(next_gif);
                {
                    DisplayLockGuard disp_lock(this);
                    current_emotion_ = next_gif;
                    PlayGifInternalLocked(asset);
                }
                // Hold idle animation for 3.5 ~ 5 seconds
                uint32_t delay_ms = 3500 + (esp_random() % 1500);
                WaitInterruptible(delay_ms);
                break;
            }
        }
    }
}
