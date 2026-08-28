#pragma once

#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class BME280 {
public:
    BME280(i2c_inst_t* i2c_instance, uint8_t addr = 0x76);
    bool init();
    bool update_all();

    bool update_temperature();
    bool update_pressure();
    bool update_humidity();

    float get_temperature() const { return temperature_c_; }
    float get_pressure()    const { return pressure_pa_; }
    float get_humidity()    const { return humidity_pct_; }

private:
    i2c_inst_t* i2c_;
    uint8_t address_;

    float temperature_c_ = 0.0f;
    float pressure_pa_   = 0.0f;
    float humidity_pct_  = 0.0f;

    int32_t t_fine_ = 0;

    struct CalibrationData {
        uint16_t dig_T1;
        int16_t  dig_T2, dig_T3;
        uint16_t dig_P1;
        int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
        uint8_t  dig_H1;
        int16_t  dig_H2;
        uint8_t  dig_H3;
        int16_t  dig_H4, dig_H5;
        int8_t   dig_H6;
    } calib_;

    bool write_register(uint8_t reg, uint8_t value);
    bool read_registers(uint8_t reg, uint8_t* buffer, size_t len);
    bool read_calibration_data();

    float compensate_temp(int32_t adc_T);
    float compensate_press(int32_t adc_P);
    float compensate_hum(int32_t adc_H);
};