#pragma once

#include <cmath>
#include <cstdint>

class Variometer {
public:
    explicit Variometer(float accel_noise = 0.8f, float baro_drift_noise = 0.005f) {
        set_noise(accel_noise, baro_drift_noise);
        reset();
    }

    void set_noise(float accel_noise, float baro_drift_noise) {
        q_accel_sq_ = accel_noise * accel_noise;
        q_baro_drift_sq_ = baro_drift_noise * baro_drift_noise;
    }

    void reset() {
        x_[0] = 0.0f; 
        x_[1] = 0.0f;
        x_[2] = 0.0f; 

        P00_ = 100.0f;
        P01_ = 0.0f;
        P02_ = 0.0f;
        P11_ = 10.0f;
        P12_ = 0.0f;
        P22_ = 100.0f;

        initialized_ = false;
    }

    static float pressure_to_altitude(float pressure_pa) {
        if (pressure_pa < 100.0f || !std::isfinite(pressure_pa)) {
            pressure_pa = 100.0f; 
        }
        constexpr float inv_p0 = 1.0f / 101325.0f;
        return 44330.0f * (1.0f - std::pow(pressure_pa * inv_p0, 0.190295f));
    }

    void update_baro(float baro_alt, float dt_sec, float r_baro = 0.25f) {
        if (!initialized_) {
            x_[0] = baro_alt;
            initialized_ = true;
            return;
        }

        predict(dt_sec);

        float z_hat = x_[0] + x_[2];
        float y = baro_alt - z_hat;

        float M0 = P00_ + P02_;
        float M1 = P01_ + P12_;
        float M2 = P02_ + P22_;

        float S = M0 + M2 + r_baro;
        if (S <= 0.0f) return;

        float inv_S = 1.0f / S;
        float K0 = M0 * inv_S;
        float K1 = M1 * inv_S;
        float K2 = M2 * inv_S;

        x_[0] += K0 * y;
        x_[1] += K1 * y;
        x_[2] += K2 * y;

        P00_ -= K0 * M0;
        P01_ -= K0 * M1;
        P02_ -= K0 * M2;
        P11_ -= K1 * M1;
        P12_ -= K1 * M2;
        P22_ -= K2 * M2;
    }

    void update_gps(float gps_alt, float dt_sec, float vdop = 1.0f) {
        if (!initialized_) {
            x_[0] = gps_alt;
            initialized_ = true;
            return;
        }

        predict(dt_sec);

        float z_hat = x_[0];
        float y = gps_alt - z_hat;

        float v25 = 2.5f * vdop;
        float r_gps = v25 * v25;

        float M0 = P00_;
        float M1 = P01_;
        float M2 = P02_;

        float S = M0 + r_gps;
        if (S <= 0.0f) return;

        float inv_S = 1.0f / S;
        float K0 = M0 * inv_S;
        float K1 = M1 * inv_S;
        float K2 = M2 * inv_S;

        x_[0] += K0 * y;
        x_[1] += K1 * y;
        x_[2] += K2 * y;

        P00_ -= K0 * M0;
        P01_ -= K0 * M1;
        P02_ -= K0 * M2;
        P11_ -= K1 * M1;
        P12_ -= K1 * M2;
        P22_ -= K2 * M2;
    }

    float get_altitude()       const { return x_[0]; }
    float get_vertical_speed() const { return x_[1]; }
    float get_baro_bias()      const { return x_[2]; }

private:
    float x_[3];
    
    float P00_, P01_, P02_;
    float P11_, P12_;
    float P22_;

    float q_accel_sq_;
    float q_baro_drift_sq_;
    bool initialized_ = false;

    void predict(float dt) {
        if (dt <= 0.0f) return;

        x_[0] += x_[1] * dt;

        float dt2 = dt * dt;
        float dt3 = dt2 * dt;
        float dt4 = dt3 * dt;

        float Q00 = q_accel_sq_ * dt4 * 0.25f;
        float Q01 = q_accel_sq_ * dt3 * 0.5f;
        float Q11 = q_accel_sq_ * dt2;
        float Q22 = q_baro_drift_sq_ * dt;

        P00_ += dt * (2.0f * P01_ + dt * P11_) + Q00;
        P01_ += dt * P11_ + Q01;
        P02_ += dt * P12_;
        P11_ += Q11;
        P22_ += Q22;
    }
};