#include "control.hpp"

// Replaces non-finite control values with a safe fallback.
inline float sanitize(float val, float default_val = 0.0f) {
    return std::isfinite(val) ? val : default_val;
}

namespace Avionics {

constexpr float G_ACCEL = 9.80665f;
constexpr float PI_F = 3.14159265358979323846f;

Quaternion Quaternion::fromEuler(float roll, float pitch, float yaw) {
    float cr = std::cosf(roll * 0.5f);
    float sr = std::sinf(roll * 0.5f);
    float cp = std::cosf(pitch * 0.5f);
    float sp = std::sinf(pitch * 0.5f);
    float cy = std::cosf(yaw * 0.5f);
    float sy = std::sinf(yaw * 0.5f);

    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    };
}

PIDController::PIDController(float kp, float ki, float kd, float out_min, float out_max, float i_max, float d_filter_alpha)
    : kp_(kp), ki_(ki), kd_(kd), 
      out_min_(out_min), out_max_(out_max), i_max_(i_max),
      d_filter_alpha_(std::clamp(d_filter_alpha, 0.01f, 1.0f)) {}

void PIDController::reset() {
    integrator_ = 0.0f;
    prev_error_ = 0.0f;
    prev_derivative_ = 0.0f;
    last_output_ = 0.0f;
    derivative_initialized_ = false;
}

float PIDController::update(float error, float dt) {
    if (!std::isfinite(error) || !std::isfinite(dt) || dt <= 1e-5f) {
        return last_output_;
    }

    const float p_out = kp_ * error;

    float derivative = 0.0f;
    if (derivative_initialized_) {
        const float raw_derivative = (error - prev_error_) / dt;
        derivative = prev_derivative_ + d_filter_alpha_ * (raw_derivative - prev_derivative_);
    } else {
        derivative_initialized_ = true;
    }

    prev_error_ = error;
    prev_derivative_ = derivative;
    const float d_out = kd_ * derivative;

    const float candidate_integrator = std::clamp(integrator_ + error * dt, -i_max_, i_max_);
    const float candidate_output = p_out + ki_ * candidate_integrator + d_out;
    const float integral_drive = ki_ * error;

    const bool windup_high = candidate_output > out_max_ && integral_drive > 0.0f;
    const bool windup_low = candidate_output < out_min_ && integral_drive < 0.0f;

    if (!windup_high && !windup_low) {
        integrator_ = candidate_integrator;
    }

    last_output_ = std::clamp(
        p_out + ki_ * integrator_ + d_out,
        out_min_, out_max_
    );
    return last_output_;
}

TECSController::TECSController()
    : throttle_pid_(0.06f, 0.010f, 0.000f, -0.50f, 0.50f, 0.20f),
      pitch_pid_(0.12f, 0.005f, 0.000f, -0.35f, 0.35f, 0.15f) {
    last_output_ = {throttle_cruise_, pitch_trim_};
}

void TECSController::reset() {
    throttle_pid_.reset();
    pitch_pid_.reset();
    last_output_ = {
        std::clamp(throttle_cruise_, 0.0f, 1.0f),
        std::clamp(pitch_trim_, -0.35f, 0.35f)
    };
}

TECSOutputs TECSController::update(const TECSInputs& in, float dt) {
    if (!std::isfinite(dt) || dt <= 1e-5f || dt > 0.5f ||
        !std::isfinite(in.h_set) || !std::isfinite(in.h_curr) ||
        !std::isfinite(in.h_dot_curr) || !std::isfinite(in.v_set) ||
        !std::isfinite(in.v_curr) || !std::isfinite(in.v_dot_curr)) {
        return last_output_;
    }

    const float e_h = in.h_set - in.h_curr;
    const float h_dot_set = std::clamp(e_h * k_h_, -max_climb_rate_, max_climb_rate_);

    const float e_v = in.v_set - in.v_curr;
    const float v_dot_set = std::clamp(e_v * k_v_, -max_accel_, max_accel_);

    const float v = std::max(in.v_curr, 0.0f);

    const float H_dot_T_set = h_dot_set + (v * v_dot_set) / G_ACCEL;
    const float H_dot_B_set = h_dot_set - (v * v_dot_set) / G_ACCEL;

    const float H_dot_T_curr = in.h_dot_curr + (v * in.v_dot_curr) / G_ACCEL;
    const float H_dot_B_curr = in.h_dot_curr - (v * in.v_dot_curr) / G_ACCEL;

    const float e_T = H_dot_T_set - H_dot_T_curr;
    const float e_B = H_dot_B_set - H_dot_B_curr;

    TECSOutputs out;
    out.throttle = std::clamp(throttle_cruise_ + throttle_pid_.update(e_T, dt), 0.0f, 1.0f);

    out.pitch_des_rad = std::clamp(
        pitch_trim_ + pitch_pid_.update(e_B, dt),
        pitch_trim_ - 0.35f,
        pitch_trim_ + 0.35f)
    ;

    if (!std::isfinite(out.throttle) || !std::isfinite(out.pitch_des_rad)) {
        return last_output_;
    }

    last_output_ = out;
    return out;
}

FlightController::FlightController()
    : rate_roll_pid_ (0.35f, 0.005f, 0.000f, -1.0f, 1.0f, 0.2f, 0.2f),
      rate_pitch_pid_(0.40f, 0.005f, 0.000f, -1.0f, 1.0f, 0.2f, 0.2f),
      rate_yaw_pid_  (0.10f, 0.010f, 0.000f, -1.0f, 1.0f, 0.15f, 1.0f),
      sideslip_pid_  (0.01f, 0.000f, 0.000f, -0.5f, 0.5f, 0.10f, 1.0f) 
{}

float FlightController::wrapPi(float angle) const {
    float wrapped = std::fmod(angle + PI_F, 2.0f * PI_F);
    if (wrapped < 0.0f) wrapped += 2.0f * PI_F;
    return wrapped - PI_F;
}

void FlightController::updateGPSCourse(float target_course_rad, float gps_course_rad, float ground_speed_m_s) {
    float course_error = wrapPi(target_course_rad - gps_course_rad);

    float turn_rate_des = k_nav_ * course_error;
    float v_safe = std::max(ground_speed_m_s, 4.0f); 
    
    target_roll_rad_ = std::atanf((v_safe * turn_rate_des) / G_ACCEL);
    
    target_roll_rad_ = std::clamp(target_roll_rad_, -0.5236f, 0.5236f);
}

ServoOutputs FlightController::updateAttitude(float dt, 
                                              const Quaternion& q_curr, 
                                              const Vector3& gyro_rad_s, 
                                              float accel_y_m_s2,
                                              float current_yaw_rad,
                                              const TECSOutputs& tecs) {

    Quaternion q_des = Quaternion::fromEuler(target_roll_rad_, tecs.pitch_des_rad, current_yaw_rad);

    Quaternion q_err = q_curr.conjugate() * q_des;

    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    Vector3 rate_des = {
        std::clamp(k_p_att_ * (2.0f * q_err.x), -max_rate_roll_, max_rate_roll_),
        std::clamp(k_p_att_ * (2.0f * q_err.y), -max_rate_pitch_, max_rate_pitch_),
        std::clamp(k_p_att_ * (2.0f * q_err.z), -max_rate_yaw_, max_rate_yaw_)
    };

    ServoOutputs out;
    out.aileron  = sanitize(rate_roll_pid_.update(rate_des.x - gyro_rad_s.x, dt), 0.0f);
    out.elevator = sanitize(rate_pitch_pid_.update(rate_des.y - gyro_rad_s.y, dt), 0.0f);
    
    float yaw_rate_out = rate_yaw_pid_.update(rate_des.z - gyro_rad_s.z, dt);
    float sideslip_out = sideslip_pid_.update(0.0f - accel_y_m_s2, dt);
    
    out.rudder   = sanitize(std::clamp(yaw_rate_out + sideslip_out, -1.0f, 1.0f), 0.0f);
    out.throttle = sanitize(tecs.throttle, 0.0f);

    return out;
}

}