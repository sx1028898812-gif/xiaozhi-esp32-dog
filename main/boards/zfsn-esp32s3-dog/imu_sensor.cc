#include "imu_sensor.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <algorithm>
#include <cstring>

#define TAG "ImuSensor"

// MPU6050 Registers
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

// QMI8658 Registers
#define QMI8658_REG_WHO_AM_I     0x00
#define QMI8658_REG_CTRL1        0x02
#define QMI8658_REG_CTRL2        0x03
#define QMI8658_REG_CTRL3        0x04
#define QMI8658_REG_CTRL5        0x06
#define QMI8658_REG_CTRL7        0x08
#define QMI8658_REG_CTRL9        0x60
#define QMI8658_REG_DATA_AX_L    0x35

ImuSensor::ImuSensor() = default;

ImuSensor::~ImuSensor() {
    if (task_handle_) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
    if (dev_handle_) {
        i2c_master_bus_rm_device(dev_handle_);
        dev_handle_ = nullptr;
    }
}

const char* ImuSensor::GetModelName() const {
    switch (model_) {
        case ImuModel::kMpu6050:
            return "MPU6050 / MPU6500";
        case ImuModel::kQmi8658:
            return "QMI8658 / QMI8658C";
        default:
            return "None";
    }
}

esp_err_t ImuSensor::WriteReg(uint8_t reg, uint8_t val) {
    if (!dev_handle_) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev_handle_, buf, 2, 100);
}

esp_err_t ImuSensor::ReadReg(uint8_t reg, uint8_t* val) {
    if (!dev_handle_ || !val) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(dev_handle_, &reg, 1, val, 1, 100);
}

esp_err_t ImuSensor::ReadRegs(uint8_t reg, uint8_t* buf, size_t len) {
    if (!dev_handle_ || !buf) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(dev_handle_, &reg, 1, buf, len, 100);
}

bool ImuSensor::ProbeAndInitMpu6050(uint8_t addr) {
    if (i2c_master_probe(i2c_bus_, addr, 50) != ESP_OK) {
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400 * 1000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev_handle_) != ESP_OK) {
        return false;
    }

    uint8_t who = 0;
    if (ReadReg(MPU6050_REG_WHO_AM_I, &who) != ESP_OK) {
        i2c_master_bus_rm_device(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    // MPU6050 returns 0x68, MPU6500 returns 0x70, MPU9250 returns 0x71/0x73
    if (who != 0x68 && who != 0x70 && who != 0x71 && who != 0x72 && who != 0x73) {
        ESP_LOGD(TAG, "Device at 0x%02X is not MPU6050/6500 (WHO_AM_I=0x%02X)", addr, who);
        i2c_master_bus_rm_device(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    // Wake up from sleep mode
    WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(15));

    // Sample rate divider: 1kHz / (1 + 7) = 125Hz
    WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07);
    // DLPF ~44Hz low pass filter (removes servo vibration noise)
    WriteReg(MPU6050_REG_CONFIG, 0x03);
    // Gyroscope +-500 dps
    WriteReg(MPU6050_REG_GYRO_CONFIG, 0x08);
    // Accelerometer +-4g (8192 LSB/g)
    WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x08);

    device_address_ = addr;
    model_ = ImuModel::kMpu6050;
    ESP_LOGI(TAG, "Initialized MPU6050 (0x%02X, WHO_AM_I=0x%02X)", addr, who);
    return true;
}

bool ImuSensor::ProbeAndInitQmi8658(uint8_t addr) {
    if (i2c_master_probe(i2c_bus_, addr, 50) != ESP_OK) {
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400 * 1000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev_handle_) != ESP_OK) {
        return false;
    }

    uint8_t who = 0;
    if (ReadReg(QMI8658_REG_WHO_AM_I, &who) != ESP_OK || who != 0x05) {
        ESP_LOGD(TAG, "Device at 0x%02X is not QMI8658 (WHO_AM_I=0x%02X)", addr, who);
        i2c_master_bus_rm_device(dev_handle_);
        dev_handle_ = nullptr;
        return false;
    }

    // Soft reset
    WriteReg(QMI8658_REG_CTRL9, 0xB0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // CTRL1: Auto-increment address
    WriteReg(QMI8658_REG_CTRL1, 0x60);
    // CTRL2: Accel +-4g, 125Hz
    WriteReg(QMI8658_REG_CTRL2, 0x23);
    // CTRL3: Gyro +-512dps, 125Hz
    WriteReg(QMI8658_REG_CTRL3, 0x53);
    // CTRL5: Enable LPF
    WriteReg(QMI8658_REG_CTRL5, 0x55);
    // CTRL7: Enable Accel & Gyro
    WriteReg(QMI8658_REG_CTRL7, 0x03);

    device_address_ = addr;
    model_ = ImuModel::kQmi8658;
    ESP_LOGI(TAG, "Initialized QMI8658 (0x%02X, WHO_AM_I=0x%02X)", addr, who);
    return true;
}

bool ImuSensor::Initialize(i2c_master_bus_handle_t i2c_bus) {
    if (!i2c_bus) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return false;
    }
    i2c_bus_ = i2c_bus;

    // 1. Try MPU6050 default address 0x68 (AD0=GND), then 0x69 (AD0=VCC)
    if (ProbeAndInitMpu6050(0x68) || ProbeAndInitMpu6050(0x69)) {
        // success
    }
    // 2. Try QMI8658 address 0x6A, then 0x6B
    else if (ProbeAndInitQmi8658(0x6A) || ProbeAndInitQmi8658(0x6B)) {
        // success
    } else {
        ESP_LOGW(TAG, "No supported IMU found (MPU6050 at 0x68/0x69 or QMI8658 at 0x6A/0x6B)");
        return false;
    }

    // Start background shake detection FreeRTOS task
    xTaskCreate(
        ImuTaskTrampoline,
        "imu_shake_task",
        3072,
        this,
        3,
        &task_handle_
    );

    return true;
}

bool ImuSensor::ReadRawData(ImuRawData& data) {
    if (!dev_handle_ || model_ == ImuModel::kNone) {
        return false;
    }

    if (model_ == ImuModel::kMpu6050) {
        uint8_t buf[14];
        if (ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buf, 14) != ESP_OK) {
            return false;
        }
        data.ax = static_cast<int16_t>((buf[0] << 8) | buf[1]);
        data.ay = static_cast<int16_t>((buf[2] << 8) | buf[3]);
        data.az = static_cast<int16_t>((buf[4] << 8) | buf[5]);
        // buf[6], buf[7] is temperature
        data.gx = static_cast<int16_t>((buf[8] << 8) | buf[9]);
        data.gy = static_cast<int16_t>((buf[10] << 8) | buf[11]);
        data.gz = static_cast<int16_t>((buf[12] << 8) | buf[13]);
        return true;
    } else if (model_ == ImuModel::kQmi8658) {
        uint8_t buf[12];
        if (ReadRegs(QMI8658_REG_DATA_AX_L, buf, 12) != ESP_OK) {
            return false;
        }
        data.ax = static_cast<int16_t>((buf[1] << 8) | buf[0]);
        data.ay = static_cast<int16_t>((buf[3] << 8) | buf[2]);
        data.az = static_cast<int16_t>((buf[5] << 8) | buf[4]);
        data.gx = static_cast<int16_t>((buf[7] << 8) | buf[6]);
        data.gy = static_cast<int16_t>((buf[9] << 8) | buf[8]);
        data.gz = static_cast<int16_t>((buf[11] << 8) | buf[10]);
        return true;
    }
    return false;
}

void ImuSensor::ImuTaskTrampoline(void* arg) {
    static_cast<ImuSensor*>(arg)->ImuTaskLoop();
}

void ImuSensor::ImuTaskLoop() {
    ESP_LOGI(TAG, "IMU Shake Detection task running on %s", GetModelName());
    ImuRawData cur = {};

    constexpr int kSampleIntervalMs = 35;
    // Delta thresholds (raw units with +-4g: 1g ~= 8192 LSB)
    constexpr int kJerkThreshold = 9000;       // ~1.1g sudden acceleration delta
    constexpr int kGyroThreshold = 9500;       // ~145 deg/s sudden rotation
    constexpr int kTriggerScore = 60;          // Requires sustained shaking
    constexpr int64_t kCooldownMs = 5000;      // 5-second cooldown to let dizzy sequence finish

    while (true) {
        if (ReadRawData(cur)) {
            if (has_prev_data_) {
                int dx = std::abs(static_cast<int>(cur.ax) - static_cast<int>(prev_data_.ax));
                int dy = std::abs(static_cast<int>(cur.ay) - static_cast<int>(prev_data_.ay));
                int dz = std::abs(static_cast<int>(cur.az) - static_cast<int>(prev_data_.az));
                int jerk = dx + dy + dz;

                int g_mag = std::abs(static_cast<int>(cur.gx)) +
                            std::abs(static_cast<int>(cur.gy)) +
                            std::abs(static_cast<int>(cur.gz));

                int64_t now_ms = esp_timer_get_time() / 1000;

                if (jerk > kJerkThreshold || g_mag > kGyroThreshold) {
                    int add = (jerk / 1500) + (g_mag / 2000);
                    shake_score_ += std::min(add, 25);
                    shake_count_++;
                } else {
                    shake_score_ = (shake_score_ * 75) / 100;
                    if (shake_count_ > 0) shake_count_--;
                }

                if (shake_score_ >= kTriggerScore && (now_ms - last_trigger_ms_) > kCooldownMs) {
                    last_trigger_ms_ = now_ms;
                    shake_score_ = 0;
                    shake_count_ = 0;
                    ESP_LOGI(TAG, "Dog picked up & shaken! Jerk=%d, Gyro=%d -> Triggering Dizzy effect!", jerk, g_mag);
                    if (dizzy_callback_) {
                        dizzy_callback_();
                    }
                }
            }
            prev_data_ = cur;
            has_prev_data_ = true;
        }
        vTaskDelay(pdMS_TO_TICKS(kSampleIntervalMs));
    }
}
