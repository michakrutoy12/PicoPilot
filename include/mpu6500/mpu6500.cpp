#include "mpu6500.hpp"

MPU6500::MPU6500(i2c_inst_t* i2c, uint8_t address) 
    : _i2c(i2c), _address(address) {}

void MPU6500::writeReg(uint8_t reg, uint8_t value) {
    uint8_t buf[] = {reg, value};
    i2c_write_blocking(_i2c, _address, buf, 2, false);
}

uint8_t MPU6500::readReg(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(_i2c, _address, &reg, 1, true);
    i2c_read_blocking(_i2c, _address, &val, 1, false);
    return val;
}

bool MPU6500::begin() {
    uint8_t id = readReg(REG_WHO_AM_I);
    if (id != 0x70) {
        return false;
    }

    writeReg(REG_PWR_MGMT_1, 0x01);
    sleep_ms(10);

    writeReg(REG_CONFIG, 0x03);
    writeReg(REG_SMPLRT_DIV, 0x09);

    writeReg(REG_GYRO_CONFIG, 0x08);
    writeReg(REG_ACCEL_CONFIG, 0x10);

    writeReg(REG_INT_ENABLE, 0x01);

    return true;
}

void MPU6500::read_data() {
    uint8_t reg = REG_ACCEL_XOUT_H;
    uint8_t buffer[14];

    if (i2c_write_timeout_us(_i2c, _address, &reg, 1, true, 1000) == PICO_ERROR_TIMEOUT) return;
    if (i2c_read_timeout_us(_i2c, _address, buffer, 14, false, 1000) == PICO_ERROR_TIMEOUT) return;

    int16_t raw_ax = (buffer[0] << 8) | buffer[1];
    int16_t raw_ay = (buffer[2] << 8) | buffer[3];
    int16_t raw_az = (buffer[4] << 8) | buffer[5];
    
    int16_t raw_temp = (buffer[6] << 8) | buffer[7];
    
    int16_t raw_gx = (buffer[8] << 8) | buffer[9];
    int16_t raw_gy = (buffer[10] << 8) | buffer[11];
    int16_t raw_gz = (buffer[12] << 8) | buffer[13];

    ax = static_cast<float>(raw_ay) * ACCEL_SCALE;
    ay = static_cast<float>(raw_ax) * ACCEL_SCALE;
    az = static_cast<float>(raw_az) * ACCEL_SCALE;
    
    temp = (static_cast<float>(raw_temp) / 333.87f) + 21.0f;
    
    gx = static_cast<float>(raw_gy) * GYRO_SCALE;
    gy = static_cast<float>(raw_gx) * GYRO_SCALE;
    gz = static_cast<float>(raw_gz) * GYRO_SCALE;
}

float MPU6500::setSampleRate(uint16_t hz) {
    const float internal_gyro_rate = 1000.0f;

    if (hz < 4) hz = 4;
    if (hz > 1000) hz = 1000;

    int32_t div = static_cast<int32_t>((internal_gyro_rate / static_cast<float>(hz)) - 0.5f);
    
    if (div < 0) div = 0;
    if (div > 255) div = 255;

    writeReg(REG_SMPLRT_DIV, static_cast<uint8_t>(div));

    return internal_gyro_rate / (1.0f + static_cast<float>(div));
}