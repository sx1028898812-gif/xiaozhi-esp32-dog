#ifndef _DOG_DISPLAY_H_
#define _DOG_DISPLAY_H_

#include "display/oled_display.h"
#include "display/lvgl_display/gif/lvgl_gif.h"
#include "dog_gifs.h"
#include "pet_dog.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>

enum class DogEyeMode {
    kNormalUI = 0,     // Standard OLED UI (Initialization, Activation Code, WiFi Config, Alerts)
    kAnimatedEyes,     // Authentic 6.0 GIF Animation (Listening, Speaking, Idle)
    kSleepingEyes,     // Authentic 6.0 sleeping.gif
    kFireworkEffect,   // Authentic 6.0 firework.gif
};

class DogDisplay : public OledDisplay {
public:
    DogDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
               int width, int height, bool mirror_x, bool mirror_y);
    virtual ~DogDisplay();

    virtual void SetupUI() override;
    virtual bool SetEmotion(const char* emotion) override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void ShowNotification(const char* notification, int duration_ms = 3000) override;

    // Switch eye animation states
    void EnterAnimatedEyes();
    void EnterSleepingEyes();
    void EnterNormalUI();
    void ShowFirework();
    void SyncWithAction(DogActionType action);

    // Play specific 6.0 GIF by name ("blink", "happy", "sleeping", etc.)
    bool PlayGif(const char* gif_name);

private:
    void PlayGifInternalLocked(const DogGifAsset* asset);
    void ApplyVisibilityLocked(bool show_gif, bool show_ui);
    bool WaitInterruptible(int ms);

    static void AnimTaskTrampoline(void* arg);
    void AnimTaskLoop();

    lv_obj_t* gif_image_ = nullptr;
    std::unique_ptr<LvglGif> gif_controller_ = nullptr;
    lv_image_dsc_t current_gif_dsc_ = {};

    std::mutex mode_mutex_;
    DogEyeMode current_mode_ = DogEyeMode::kNormalUI;
    DogEyeMode return_mode_after_firework_ = DogEyeMode::kAnimatedEyes;
    std::string current_emotion_ = "blink";
    bool custom_emotion_active_ = false;

    EventGroupHandle_t event_group_ = nullptr;
    TaskHandle_t anim_task_handle_ = nullptr;
};

#endif // _DOG_DISPLAY_H_
