#pragma once

#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class BME280 {
public:
    // Creates a sensor driver for the selected I2C bus and device address.
    BME280(i2c_inst_t* i2c_instance, uint8_t addr = 0x76);
    // Checks the device and configures the sensor and calibration data.
    bool init();
    // Reads and updates temperature, pressure, and humidity together.
    bool update_all();

    // Reads and updates temperature.
    bool update_temperature();
    // Reads and updates pressure and its temperature compensation.
    bool update_pressure();
    // Reads and updates humidity using a fresh temperature measurement.
    bool update_humidity();

    // Returns the last compensated temperature in degrees Celsius.
    float get_temperature() const { return temperature_c_; }
    // Returns the last compensated pressure in pascals.
    float get_pressure()    const { return pressure_pa_; }
    // Returns the last compensated relative humidity in percent.
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

    // Writes one value to a sensor register.
    bool write_register(uint8_t reg, uint8_t value);
    // Reads a sequence of bytes starting at a sensor register.
    bool read_registers(uint8_t reg, uint8_t* buffer, size_t len);
    // Reads the sensor's factory calibration coefficients.
    bool read_calibration_data();

    // Converts a raw temperature reading using the factory coefficients.
    float compensate_temp(int32_t adc_T);
    // Converts a raw pressure reading using the factory coefficients.
    float compensate_press(int32_t adc_P);
    // Converts a raw humidity reading using the factory coefficients.
    float compensate_hum(int32_t adc_H);
};