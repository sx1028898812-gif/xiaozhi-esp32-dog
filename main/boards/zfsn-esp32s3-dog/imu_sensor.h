#ifndef _IMU_SENSOR_H_
#define _IMU_SENSOR_H_

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>
#include <cstdint>
#include <cmath>

enum class ImuModel {
    kNone = 0,
    kMpu6050,
    kQmi8658,
};

struct ImuRawData {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};

class ImuSensor {
public:
    ImuSensor();
    ~ImuSensor();

    ImuSensor(const ImuSensor&) = delete;
    ImuSensor& operator=(const ImuSensor&) = delete;

    // Probe I2C bus for MPU6050 (0x68, 0x69) or QMI8658 (0x6A, 0x6B) and initialize
    bool Initialize(i2c_master_bus_handle_t i2c_bus);

    bool IsPresent() const { return model_ != ImuModel::kNone; }
    ImuModel GetModel() const { return model_; }
    const char* GetModelName() const;

    using DizzyCallback = std::function<void()>;
    void SetOnDizzyCallback(DizzyCallback cb) { dizzy_callback_ = std::move(cb); }

    bool ReadRawData(ImuRawData& data);

private:
    bool ProbeAndInitMpu6050(uint8_t addr);
    bool ProbeAndInitQmi8658(uint8_t addr);

    esp_err_t WriteReg(uint8_t reg, uint8_t val);
    esp_err_t ReadReg(uint8_t reg, uint8_t* val);
    esp_err_t ReadRegs(uint8_t reg, uint8_t* buf, size_t len);

    static void ImuTaskTrampoline(void* arg);
    void ImuTaskLoop();

    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    uint8_t device_address_ = 0;
    ImuModel model_ = ImuModel::kNone;

    TaskHandle_t task_handle_ = nullptr;
    DizzyCallback dizzy_callback_;

    // Shake detection filter state
    int shake_score_ = 0;
    int shake_count_ = 0;
    int64_t last_trigger_ms_ = 0;
    ImuRawData prev_data_ = {};
    bool has_prev_data_ = false;
};

#endif // _IMU_SENSOR_H_
