#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"

class MPU6500 {
public:
    float ax, ay, az;
    float temp;
    float gx, gy, gz;

    MPU6500(i2c_inst_t* i2c, uint8_t address = 0x68);

    bool begin();
    void read_data();
    float setSampleRate(uint16_t hz);

private:
    i2c_inst_t* _i2c;
    uint8_t _address;

    static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;
    static constexpr uint8_t REG_INT_ENABLE = 0x38;

    static constexpr float ACCEL_SCALE = 9.80665f / 4096.0f; // ±8g
    static constexpr float GYRO_SCALE  = 0.0174532925f / 65.5f;   // ±500 DPS

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
};