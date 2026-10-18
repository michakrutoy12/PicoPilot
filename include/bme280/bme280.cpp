#include "bme280.hpp"
#include <cmath>

namespace Reg {
    constexpr uint8_t ID        = 0xD0;
    constexpr uint8_t RESET     = 0xE0;
    constexpr uint8_t CTRL_HUM  = 0xF2;
    constexpr uint8_t STATUS    = 0xF3;
    constexpr uint8_t CTRL_MEAS = 0xF4;
    constexpr uint8_t CONFIG    = 0xF5;
    constexpr uint8_t PRESS_MSB = 0xF7;
    constexpr uint8_t TEMP_MSB  = 0xFA;
    constexpr uint8_t HUM_MSB   = 0xFD;
    constexpr uint8_t CALIB_00  = 0x88;
    constexpr uint8_t CALIB_26  = 0xE1;
}

BME280::BME280(i2c_inst_t* i2c_instance, uint8_t addr)
    : i2c_(i2c_instance), address_(addr) {}

bool BME280::write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_write_timeout_per_char_us(i2c_, address_, buf, 2, false, 1000) == 2;
}

bool BME280::read_registers(uint8_t reg, uint8_t* buffer, size_t len) {
    if (i2c_write_timeout_per_char_us(i2c_, address_, &reg, 1, true, 1000) != 1) {
        return false;
    }
    
    int bytes_read = i2c_read_timeout_per_char_us(i2c_, address_, buffer, len, false, 1000);
    
    return bytes_read == (int)len;
}


bool BME280::init() {
    uint8_t chip_id = 0;
    if (!read_registers(Reg::ID, &chip_id, 1) || chip_id != 0x60) {
        return false;
    }

    if (!write_register(Reg::RESET, 0xB6)) return false;
    sleep_ms(10);

    if (!read_calibration_data()) {
        return false;
    }

    if (!write_register(Reg::CTRL_HUM, 0x01)) return false;

    if (!write_register(Reg::CONFIG, (0x00 << 5) | (0x02 << 2))) return false;

    if (!write_register(Reg::CTRL_MEAS, (0x01 << 5) | (0x01 << 2) | 0x03)) return false;

    sleep_ms(10);
    return true;
}

bool BME280::read_calibration_data() {
    uint8_t b1[26];
    uint8_t b2[7];

    if (!read_registers(Reg::CALIB_00, b1, 26) || !read_registers(Reg::CALIB_26, b2, 7)) {
        return false;
    }

    calib_.dig_T1 = (b1[1] << 8) | b1[0];
    calib_.dig_T2 = (b1[3] << 8) | b1[2];
    calib_.dig_T3 = (b1[5] << 8) | b1[4];

    calib_.dig_P1 = (b1[7] << 8) | b1[6];
    calib_.dig_P2 = (b1[9] << 8) | b1[8];
    calib_.dig_P3 = (b1[11] << 8) | b1[10];
    calib_.dig_P4 = (b1[13] << 8) | b1[12];
    calib_.dig_P5 = (b1[15] << 8) | b1[14];
    calib_.dig_P6 = (b1[17] << 8) | b1[16];
    calib_.dig_P7 = (b1[19] << 8) | b1[18];
    calib_.dig_P8 = (b1[21] << 8) | b1[20];
    calib_.dig_P9 = (b1[23] << 8) | b1[22];

    calib_.dig_H1 = b1[25];
    calib_.dig_H2 = (b2[1] << 8) | b2[0];
    calib_.dig_H3 = b2[2];
    calib_.dig_H4 = ((int16_t)b2[3] << 4) | (b2[4] & 0x0F);
    calib_.dig_H5 = ((int16_t)b2[5] << 4) | (b2[4] >> 4);
    calib_.dig_H6 = (int8_t)b2[6];

    return true;
}

float BME280::compensate_temp(int32_t adc_T) {
    double var1 = (((double)adc_T) / 16384.0 - ((double)calib_.dig_T1) / 1024.0) * ((double)calib_.dig_T2);
    double var2 = ((((double)adc_T) / 131072.0 - ((double)calib_.dig_T1) / 8192.0) *
                   (((double)adc_T) / 131072.0 - ((double)calib_.dig_T1) / 8192.0)) * ((double)calib_.dig_T3);
    t_fine_ = (int32_t)(var1 + var2);
    return (float)((var1 + var2) / 5120.0);
}

float BME280::compensate_press(int32_t adc_P) {
    double var1 = ((double)t_fine_ / 2.0) - 64000.0;
    double var2 = var1 * var1 * ((double)calib_.dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_.dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_.dig_P4) * 65536.0);
    var1 = (((double)calib_.dig_P3) * var1 * var1 / 524288.0 + ((double)calib_.dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_.dig_P1);

    if (var1 == 0.0) return 0.0f;

    double p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)calib_.dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)calib_.dig_P8) / 32768.0;
    return (float)(p + (var1 + var2 + ((double)calib_.dig_P7)) / 16.0);
}

float BME280::compensate_hum(int32_t adc_H) {
    double h_p = ((double)t_fine_) - 76800.0;
    h_p = (adc_H - (((double)calib_.dig_H4) * 64.0 + ((double)calib_.dig_H5) / 16384.0 * h_p)) *
          (((double)calib_.dig_H2) / 65536.0 * (1.0 + ((double)calib_.dig_H6) / 67108864.0 * h_p *
          (1.0 + ((double)calib_.dig_H3) / 67108864.0 * h_p)));
    h_p = h_p * (1.0 - ((double)calib_.dig_H1) * h_p / 524288.0);

    if (h_p > 100.0) h_p = 100.0;
    else if (h_p < 0.0) h_p = 0.0;

    return (float)h_p;
}

bool BME280::update_all() {
    uint8_t buf[8];
    if (!read_registers(Reg::PRESS_MSB, buf, 8)) return false;

    int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    int32_t adc_H = (buf[6] << 8)  | buf[7];

    float temperature = compensate_temp(adc_T);
    float pressure = compensate_press(adc_P);
    if (!std::isfinite(temperature) || !std::isfinite(pressure) || pressure < 100.0f) {
        return false;
    }
    temperature_c_ = temperature;
    pressure_pa_ = pressure;
    humidity_pct_  = compensate_hum(adc_H);

    return true;
}

bool BME280::update_temperature() {
    uint8_t buf[3];
    if (!read_registers(Reg::TEMP_MSB, buf, 3)) return false;

    int32_t adc_T = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    temperature_c_ = compensate_temp(adc_T);

    return true;
}

bool BME280::update_pressure() {
    uint8_t buf[6];
    if (!read_registers(Reg::PRESS_MSB, buf, 6)) return false;

    int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

    float temperature = compensate_temp(adc_T);
    float pressure = compensate_press(adc_P);
    if (!std::isfinite(temperature) || !std::isfinite(pressure) || pressure < 100.0f) {
        return false;
    }
    temperature_c_ = temperature;
    pressure_pa_ = pressure;

    return true;
}

bool BME280::update_humidity() {
    if (!update_temperature()) return false;

    uint8_t buf[2];
    if (!read_registers(Reg::HUM_MSB, buf, 2)) return false;

    int32_t adc_H = (buf[0] << 8) | buf[1];
    humidity_pct_ = compensate_hum(adc_H);

    return true;
}