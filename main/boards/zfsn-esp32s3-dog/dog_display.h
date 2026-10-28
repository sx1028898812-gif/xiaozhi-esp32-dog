#ifndef _DOG_DISPLAY_H_
#define _DOG_DISPLAY_H_

#include "display/oled_display.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <atomic>
#include <mutex>
#include <string>

enum class DogEyeMode {
    kNormalUI = 0,     // Standard OLED UI (Initialization, Activation Code, WiFi Config, Alerts)
    kAnimatedEyes,     // Lively blinking & gazing twin eyes (Listening & Speaking)
    kSleepingEyes,     // Half-closed resting eyes (Idle standby)
    kFireworkEffect,   // Celebration firework particle burst
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

private:
    static constexpr int kEyeSize = 40;
    static constexpr int kEyeGap = 25;
    static constexpr int kEyeRadius = 10;
    static constexpr int kFireworkParticles = 12;

    void ApplyVisibilityLocked(bool show_eyes, bool show_ui, bool show_fireworks = false);
    void MoveEyesSmooth(int target_x1, int target_y1, int target_x2, int target_y2);
    void CloseEyesSmooth();
    void OpenEyesSmooth(int target_height = kEyeSize, int target_width = kEyeSize, int radius = kEyeRadius);
    void BlinkOnce();
    void RunFireworkAnimation();
    bool WaitInterruptible(int ms);

    static void AnimTaskTrampoline(void* arg);
    void AnimTaskLoop();

    lv_obj_t* eye_overlay_ = nullptr; // Full-screen overlay for eyes & fireworks
    lv_obj_t* eye_right_ = nullptr;   // square1 (+kEyeGap)
    lv_obj_t* eye_left_ = nullptr;    // square2 (-kEyeGap)
    lv_obj_t* firework_dots_[kFireworkParticles] = {nullptr};

    int cur_x1_ = kEyeGap;
    int cur_y1_ = 0;
    int cur_x2_ = -kEyeGap;
    int cur_y2_ = 0;

    std::mutex mode_mutex_;
    DogEyeMode current_mode_ = DogEyeMode::kNormalUI;
    DogEyeMode return_mode_after_firework_ = DogEyeMode::kAnimatedEyes;
    std::string current_emotion_ = "neutral";

    EventGroupHandle_t event_group_ = nullptr;
    TaskHandle_t anim_task_handle_ = nullptr;
};

#endif // _DOG_DISPLAY_H_

