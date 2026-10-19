#pragma once

#include <algorithm>
#include <cmath>

namespace Avionics {
struct Vector3 {
    float x, y, z;
};

struct Quaternion {
    float w, x, y, z;

    [[nodiscard]] Quaternion conjugate() const {
        return {w, -x, -y, -z};
    }

    [[nodiscard]] Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    [[nodiscard]] static Quaternion fromEuler(float roll, float pitch, float yaw);
};

struct TECSInputs {
    float h_set; 
    float h_curr;
    float h_dot_curr; 
    float v_set;
    float v_curr;
    float v_dot_curr;
};

struct TECSOutputs {
    float throttle;
    float pitch_des_rad;
};

struct ServoOutputs {
    float aileron;
    float elevator;
    float rudder;
    float throttle;
};

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float out_min, float out_max, float i_max, float d_filter_alpha = 1.0f);

    void reset();
    float update(float error, float dt);

private:
    float kp_, ki_, kd_;
    float out_min_, out_max_, i_max_;
    float d_filter_alpha_;

    float integrator_ = 0.0f;
    float prev_error_ = 0.0f;
    float prev_derivative_ = 0.0f;
    float last_output_ = 0.0f;
    bool derivative_initialized_ = false;
};

class TECSController {
public:
    TECSController();
    TECSOutputs update(const TECSInputs& in, float dt);
    void reset();

private:
    float k_h_ = 0.15f;
    float k_v_ = 0.2f;
    float max_climb_rate_ = 2.5f;
    float max_accel_ = 2.0f;
    float throttle_cruise_ = 0.5f;
    float pitch_trim_ = 0.00f;

    PIDController throttle_pid_;
    PIDController pitch_pid_;
    TECSOutputs last_output_{0.5f, 0.0f};
};

class FlightController {
public:
    FlightController();

    void updateGPSCourse(float target_course_rad, float gps_course_rad, float ground_speed_m_s);

    ServoOutputs updateAttitude(float dt, 
                                const Quaternion& q_curr, 
                                const Vector3& gyro_rad_s, 
                                float accel_y_m_s2,
                                float current_yaw_rad,
                                const TECSOutputs& tecs);

private:
    [[nodiscard]] float wrapPi(float angle) const;

    float target_roll_rad_ = 0.0f;
    
    float k_nav_ = 0.5f;   
    float k_p_att_ = 3.5f;

    float max_rate_roll_  = 2.0f; 
    float max_rate_pitch_ = 0.45f;
    float max_rate_yaw_   = 0.5f;

    PIDController rate_roll_pid_;
    PIDController rate_pitch_pid_;
    PIDController rate_yaw_pid_;
    PIDController sideslip_pid_;
};

}