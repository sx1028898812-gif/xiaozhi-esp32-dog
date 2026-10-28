#include "ws2812_led.h"
#include "config.h"

#include <esp_log.h>
#include <algorithm>
#include <cctype>

#define TAG "WS2812Led"
#define MODE_CHANGED_BIT (1 << 0)

WS2812Led::WS2812Led() {
    event_group_ = xEventGroupCreate();
    InitStrips();
    xTaskCreate(EffectTaskTrampoline, "ws2812_task", 3072, this, 3, &task_handle_);
}

WS2812Led::~WS2812Led() {
    if (task_handle_) {
        vTaskDelete(task_handle_);
    }
    if (event_group_) {
        vEventGroupDelete(event_group_);
    }
}

void WS2812Led::InitStrips() {
    led_strip_config_t strip_config1 = {};
    strip_config1.strip_gpio_num = WS2812_STRIP1_GPIO;
    strip_config1.max_leds = WS2812_LED_MAX_NUMBERS;
    strip_config1.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config1.led_model = LED_MODEL_WS2812;

    led_strip_config_t strip_config2 = {};
    strip_config2.strip_gpio_num = WS2812_STRIP2_GPIO;
    strip_config2.max_leds = WS2812_LED_MAX_NUMBERS;
    strip_config2.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config2.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config1, &rmt_config, &strip1_));
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config2, &rmt_config, &strip2_));
    ClearAll();
}

RgbColor WS2812Led::ParseColor(const std::string& color_name, int brightness) {
    int b = std::clamp(brightness, 0, 255);
    std::string lower = color_name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (lower == "off" || lower == "none" || b == 0) {
        return {0, 0, 0};
    } else if (lower == "red") {
        return {static_cast<uint8_t>(b), 0, 0};
    } else if (lower == "green") {
        return {0, static_cast<uint8_t>(b), 0};
    } else if (lower == "blue") {
        return {0, 0, static_cast<uint8_t>(b)};
    } else if (lower == "yellow") {
        return {static_cast<uint8_t>(b), static_cast<uint8_t>(b), 0};
    } else if (lower == "cyan") {
        return {0, static_cast<uint8_t>(b), static_cast<uint8_t>(b)};
    } else if (lower == "magenta" || lower == "purple" || lower == "pink") {
        return {static_cast<uint8_t>(b), 0, static_cast<uint8_t>(b)};
    } else if (lower == "orange" || lower == "warm") {
        return {static_cast<uint8_t>(b), static_cast<uint8_t>(b * 90 / 255), static_cast<uint8_t>(b * 15 / 255)};
    } else {
        // Default to white
        return {static_cast<uint8_t>(b), static_cast<uint8_t>(b), static_cast<uint8_t>(b)};
    }
}

RgbColor WS2812Led::Wheel(uint8_t pos, int brightness) {
    int scale = std::clamp(brightness, 0, 255);
    uint8_t r = 0, g = 0, b = 0;
    pos = 255 - pos;
    if (pos < 85) {
        r = 255 - pos * 3;
        g = 0;
        b = pos * 3;
    } else if (pos < 170) {
        pos -= 85;
        r = 0;
        g = pos * 3;
        b = 255 - pos * 3;
    } else {
        pos -= 170;
        r = pos * 3;
        g = 255 - pos * 3;
        b = 0;
    }
    return {
        static_cast<uint8_t>((r * scale) / 255),
        static_cast<uint8_t>((g * scale) / 255),
        static_cast<uint8_t>((b * scale) / 255),
    };
}

void WS2812Led::FillAll(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < WS2812_LED_MAX_NUMBERS; i++) {
        led_strip_set_pixel(strip1_, i, r, g, b);
        led_strip_set_pixel(strip2_, i, r, g, b);
    }
    RefreshAll();
}

void WS2812Led::ClearAll() {
    led_strip_clear(strip1_);
    led_strip_clear(strip2_);
}

void WS2812Led::RefreshAll() {
    led_strip_refresh(strip1_);
    led_strip_refresh(strip2_);
}

void WS2812Led::SetMode(Ws2812Mode mode, RgbColor base_color, int brightness) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_mode_ = mode;
        current_color_ = base_color;
        current_brightness_ = std::clamp(brightness, 0, 255);
    }
    xEventGroupSetBits(event_group_, MODE_CHANGED_BIT);
}

bool WS2812Led::Control(const std::string& action, const std::string& color_name, int brightness) {
    int b = std::clamp(brightness, 0, 255);
    if (action == "turn_on_off") {
        if (color_name == "off" || color_name == "none" || b == 0) {
            TurnOff();
        } else {
            SetSolidColor(color_name, b);
        }
        return true;
    } else if (action == "flash_light") {
        StartFlash(color_name == "off" ? "white" : color_name, b > 0 ? b : 128);
        return true;
    } else if (action == "water_light") {
        StartWater(color_name == "off" ? "cyan" : color_name, b > 0 ? b : 128);
        return true;
    } else if (action == "breathe_light") {
        StartBreathe(color_name == "off" ? "blue" : color_name, b > 0 ? b : 128);
        return true;
    } else if (action == "rainbow_light") {
        StartRainbow(b > 0 ? b : 128);
        return true;
    }
    return false;
}

void WS2812Led::SetSolidColor(const std::string& color_name, int brightness) {
    RgbColor c = ParseColor(color_name, brightness);
    if (c.r == 0 && c.g == 0 && c.b == 0) {
        SetMode(Ws2812Mode::kOff, c, 0);
    } else {
        SetMode(Ws2812Mode::kSolid, c, brightness);
    }
}

void WS2812Led::TurnOff() {
    SetMode(Ws2812Mode::kOff, {0, 0, 0}, 0);
}

void WS2812Led::StartFlash(const std::string& color_name, int brightness) {
    SetMode(Ws2812Mode::kFlash, ParseColor(color_name, 255), brightness);
}

void WS2812Led::StartWater(const std::string& color_name, int brightness) {
    SetMode(Ws2812Mode::kWater, ParseColor(color_name, brightness), brightness);
}

void WS2812Led::StartBreathe(const std::string& color_name, int brightness) {
    SetMode(Ws2812Mode::kBreathe, ParseColor(color_name, 255), brightness);
}

void WS2812Led::StartRainbow(int brightness) {
    SetMode(Ws2812Mode::kRainbow, {255, 255, 255}, brightness);
}

void WS2812Led::PlayPowerOnEffect() {
    SetMode(Ws2812Mode::kPowerOnOnce, {0, 255, 0}, 128);
}

bool WS2812Led::WaitInterruptible(int ms) {
    EventBits_t bits = xEventGroupWaitBits(
        event_group_, MODE_CHANGED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(ms));
    return (bits & MODE_CHANGED_BIT) != 0;
}

void WS2812Led::EffectTaskTrampoline(void* arg) {
    static_cast<WS2812Led*>(arg)->EffectTaskLoop();
}

void WS2812Led::EffectTaskLoop() {
    uint8_t step = 0;
    while (true) {
        Ws2812Mode mode;
        RgbColor color;
        int brightness;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            mode = current_mode_;
            color = current_color_;
            brightness = current_brightness_;
        }
        xEventGroupClearBits(event_group_, MODE_CHANGED_BIT);

        switch (mode) {
            case Ws2812Mode::kOff:
                ClearAll();
                xEventGroupWaitBits(event_group_, MODE_CHANGED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
                break;

            case Ws2812Mode::kSolid:
                FillAll(color.r, color.g, color.b);
                xEventGroupWaitBits(event_group_, MODE_CHANGED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
                break;

            case Ws2812Mode::kPowerOnOnce:
                ClearAll();
                for (int i = 0; i < WS2812_LED_MAX_NUMBERS; i++) {
                    led_strip_set_pixel(strip1_, i, 0, 128, 0);
                    led_strip_set_pixel(strip2_, i, 0, 128, 0);
                    RefreshAll();
                    if (WaitInterruptible(120)) break;
                }
                if (!(xEventGroupGetBits(event_group_) & MODE_CHANGED_BIT)) {
                    WaitInterruptible(400);
                }
                if (!(xEventGroupGetBits(event_group_) & MODE_CHANGED_BIT)) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (current_mode_ == Ws2812Mode::kPowerOnOnce) {
                        current_mode_ = Ws2812Mode::kOff;
                    }
                }
                break;

            case Ws2812Mode::kFlash: {
                uint8_t r = (color.r * brightness) / 255;
                uint8_t g = (color.g * brightness) / 255;
                uint8_t b = (color.b * brightness) / 255;
                FillAll(r, g, b);
                if (WaitInterruptible(200)) break;
                ClearAll();
                WaitInterruptible(200);
                break;
            }

            case Ws2812Mode::kWater:
                for (int i = 0; i < WS2812_LED_MAX_NUMBERS; i++) {
                    ClearAll();
                    for (int j = 0; j < 3; j++) {
                        int pos = (i + j) % WS2812_LED_MAX_NUMBERS;
                        led_strip_set_pixel(strip1_, pos, color.r, color.g, color.b);
                        led_strip_set_pixel(strip2_, pos, color.r, color.g, color.b);
                    }
                    RefreshAll();
                    if (WaitInterruptible(90)) break;
                }
                break;

            case Ws2812Mode::kBreathe:
                for (int level = 5; level <= brightness; level += std::max(2, brightness / 25)) {
                    FillAll((color.r * level) / 255, (color.g * level) / 255, (color.b * level) / 255);
                    if (WaitInterruptible(25)) break;
                }
                if (xEventGroupGetBits(event_group_) & MODE_CHANGED_BIT) break;
                for (int level = brightness; level >= 5; level -= std::max(2, brightness / 25)) {
                    FillAll((color.r * level) / 255, (color.g * level) / 255, (color.b * level) / 255);
                    if (WaitInterruptible(25)) break;
                }
                break;

            case Ws2812Mode::kRainbow:
                for (int i = 0; i < WS2812_LED_MAX_NUMBERS; i++) {
                    RgbColor rc = Wheel(static_cast<uint8_t>((i * 256 / WS2812_LED_MAX_NUMBERS) + step), brightness);
                    led_strip_set_pixel(strip1_, i, rc.r, rc.g, rc.b);
                    led_strip_set_pixel(strip2_, i, rc.r, rc.g, rc.b);
                }
                RefreshAll();
                step += 5;
                WaitInterruptible(35);
                break;
        }
    }
}

