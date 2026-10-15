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
        if (!std::isfinite(accel_noise) || accel_noise < 0.0f ||
            !std::isfinite(baro_drift_noise) || baro_drift_noise < 0.0f) {
            return;
        }
        const float accel_noise_sq = accel_noise * accel_noise;
        const float baro_drift_noise_sq = baro_drift_noise * baro_drift_noise;
        if (!std::isfinite(accel_noise_sq) || !std::isfinite(baro_drift_noise_sq)) return;
        q_accel_sq_ = accel_noise_sq;
        q_baro_drift_sq_ = baro_drift_noise_sq;
    }

    void predict(float dt) {
        if (!initialized_) return;
        if (!state_is_finite()) {
            reset();
            return;
        }
        if (!std::isfinite(dt) || dt <= 0.0f || dt > 0.02f) return;

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
        if (!state_is_finite()) reset();
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

    static bool pressure_to_altitude(float pressure_pa, float& altitude_m) {
        if (!std::isfinite(pressure_pa) || pressure_pa < 10000.0f || pressure_pa > 120000.0f) {
            return false;
        }
        constexpr float inv_p0 = 1.0f / 101325.0f;
        altitude_m = 44330.0f * (1.0f - std::pow(pressure_pa * inv_p0, 0.190295f));
        return std::isfinite(altitude_m);
    }

    bool update_baro(float baro_alt, float r_baro = 0.25f) {
        if (!std::isfinite(baro_alt) ||
            !std::isfinite(r_baro) || r_baro <= 0.0f ||
            std::abs(baro_alt) > 20000.0f) return false;
        if (!state_is_finite()) {
            reset();
            return false;
        }

        if (!initialized_) {
            x_[0] = baro_alt;
            initialized_ = true;
            return true;
        }

        float z_hat = x_[0] + x_[2];
        float y = baro_alt - z_hat;

        float M0 = P00_ + P02_;
        float M1 = P01_ + P12_;
        float M2 = P02_ + P22_;

        float S = M0 + M2 + r_baro;
        if (!std::isfinite(S) || S <= 1e-9f || !std::isfinite(y)) return false;

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
        if (!state_is_finite()) {
            reset();
            return false;
        }
        return true;
    }

    bool update_gps(float gps_alt, float altitude_variance = 25.0f) {
        if (!std::isfinite(gps_alt) ||
            !std::isfinite(altitude_variance) || altitude_variance <= 0.0f ||
            std::abs(gps_alt) > 20000.0f) return false;
        if (!state_is_finite()) {
            reset();
            return false;
        }

        if (!initialized_) {
            x_[0] = gps_alt;
            initialized_ = true;
            return true;
        }

        float z_hat = x_[0];
        float y = gps_alt - z_hat;

        float r_gps = altitude_variance;

        float M0 = P00_;
        float M1 = P01_;
        float M2 = P02_;

        float S = M0 + r_gps;
        if (!std::isfinite(S) || S <= 1e-9f || !std::isfinite(y) || !std::isfinite(r_gps)) return false;

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
        if (!state_is_finite()) {
            reset();
            return false;
        }
        return true;
    }

    float get_altitude()       const { return x_[0]; }
    float get_vertical_speed() const { return x_[1]; }
    float get_baro_bias()      const { return x_[2]; }
    bool is_initialized()      const { return initialized_ && state_is_finite(); }

private:
    float x_[3];
    
    float P00_, P01_, P02_;
    float P11_, P12_;
    float P22_;

    float q_accel_sq_ = 0.64f;
    float q_baro_drift_sq_ = 0.000025f;
    bool initialized_ = false;

    bool state_is_finite() const {
        for (float value : x_) {
            if (!std::isfinite(value)) return false;
        }
        return std::isfinite(P00_) && std::isfinite(P01_) && std::isfinite(P02_) &&
               std::isfinite(P11_) && std::isfinite(P12_) && std::isfinite(P22_);
    }

};