#ifndef _WS2812_LED_H_
#define _WS2812_LED_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <led_strip.h>
#include <cstdint>
#include <string>
#include <mutex>

enum class Ws2812Mode {
    kOff = 0,
    kSolid,
    kFlash,
    kWater,
    kBreathe,
    kRainbow,
    kPowerOnOnce,
};

struct RgbColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

class WS2812Led {
public:
    static WS2812Led& GetInstance() {
        static WS2812Led instance;
        return instance;
    }

    WS2812Led(const WS2812Led&) = delete;
    WS2812Led& operator=(const WS2812Led&) = delete;

    // Unified control entry matching MCP self.ws2812_led.control
    bool Control(const std::string& action, const std::string& color_name, int brightness);

    void SetSolidColor(const std::string& color_name, int brightness = 128);
    void TurnOff();
    void StartFlash(const std::string& color_name = "white", int brightness = 128);
    void StartWater(const std::string& color_name = "cyan", int brightness = 128);
    void StartBreathe(const std::string& color_name = "blue", int brightness = 128);
    void StartRainbow(int brightness = 128);
    void PlayPowerOnEffect();

    static RgbColor ParseColor(const std::string& color_name, int brightness = 128);

private:
    WS2812Led();
    ~WS2812Led();

    void InitStrips();
    void SetMode(Ws2812Mode mode, RgbColor base_color, int brightness);
    bool WaitInterruptible(int ms);
    void FillAll(uint8_t r, uint8_t g, uint8_t b);
    void ClearAll();
    void RefreshAll();
    static RgbColor Wheel(uint8_t pos, int brightness);

    static void EffectTaskTrampoline(void* arg);
    void EffectTaskLoop();

    led_strip_handle_t strip1_ = nullptr;
    led_strip_handle_t strip2_ = nullptr;
    EventGroupHandle_t event_group_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    std::mutex mutex_;

    Ws2812Mode current_mode_ = Ws2812Mode::kPowerOnOnce;
    RgbColor current_color_ = {0, 255, 0};
    int current_brightness_ = 128;
};

#endif // _WS2812_LED_H_

