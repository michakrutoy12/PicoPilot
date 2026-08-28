#pragma once

#include <cmath>
#include <algorithm>

constexpr float GRAVITY_MSS = 9.80665f;

struct Vector3f {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    float norm() const {
        return std::sqrtf(x * x + y * y + z * z);
    }

    Vector3f cross(const Vector3f& v) const {
        return {
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        };
    }
};

struct CentripetalConfig {
    Vector3f lever_arm_m = {0.25f, 0.0f, 0.0f}; 

    float base_r_accel = 0.015f;
    float g_gate_threshold = 1.5f;
    float g_gate_scale = 4.0f;
    float max_r_scale = 100.0f;
    float min_gps_speed_ms = 2.0f; 
    float alpha_gyro_dot = 0.15f;
};

class CentripetalCompensator {
public:
    explicit CentripetalCompensator(const CentripetalConfig& config = CentripetalConfig())
        : config_(config) {}

    void process(const Vector3f& accel_raw,
                 const Vector3f& gyro_rads,
                 float dt,
                 float gps_speed_ms,
                 Vector3f& accel_corr,
                 float& adapted_r) 
    {
        accel_corr = accel_raw;

        if (dt > 0.0001f) {
            Vector3f raw_gyro_dot = {
                (gyro_rads.x - gyro_prev_.x) / dt,
                (gyro_rads.y - gyro_prev_.y) / dt,
                (gyro_rads.z - gyro_prev_.z) / dt
            };

            gyro_dot_lpf_.x += config_.alpha_gyro_dot * (raw_gyro_dot.x - gyro_dot_lpf_.x);
            gyro_dot_lpf_.y += config_.alpha_gyro_dot * (raw_gyro_dot.y - gyro_dot_lpf_.y);
            gyro_dot_lpf_.z += config_.alpha_gyro_dot * (raw_gyro_dot.z - gyro_dot_lpf_.z);

            Vector3f a_tangential = gyro_dot_lpf_.cross(config_.lever_arm_m);

            Vector3f w_cross_r = gyro_rads.cross(config_.lever_arm_m);
            Vector3f a_rot_centrip = gyro_rads.cross(w_cross_r);

            accel_corr.x -= (a_tangential.x + a_rot_centrip.x);
            accel_corr.y -= (a_tangential.y + a_rot_centrip.y);
            accel_corr.z -= (a_tangential.z + a_rot_centrip.z);
        }

        gyro_prev_ = gyro_rads;

        if (gps_speed_ms >= config_.min_gps_speed_ms) {
            accel_corr.y -= gyro_rads.z * gps_speed_ms;
            accel_corr.z += gyro_rads.y * gps_speed_ms;
        }

        float current_norm = accel_corr.norm();
        float delta_g = std::abs(current_norm - GRAVITY_MSS);

        if (delta_g > config_.g_gate_threshold) {
            float excess = delta_g - config_.g_gate_threshold;
            float scale = 1.0f + config_.g_gate_scale * excess;
            float r_multiplier = std::min(scale * scale, config_.max_r_scale);
            
            adapted_r = config_.base_r_accel * r_multiplier;
        } else {
            adapted_r = config_.base_r_accel;
        }
    }

private:
    CentripetalConfig config_;
    Vector3f gyro_prev_{0.0f, 0.0f, 0.0f};
    Vector3f gyro_dot_lpf_{0.0f, 0.0f, 0.0f};
};