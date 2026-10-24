#pragma once

#include <algorithm>
#include <cmath>

namespace Avionics {
struct Vector3 {
    float x, y, z;
};

struct Quaternion {
    float w, x, y, z;

    // Returns the conjugate, used as the inverse for unit quaternions.
    [[nodiscard]] Quaternion conjugate() const {
        return {w, -x, -y, -z};
    }

    // Multiplies this quaternion by another quaternion.
    [[nodiscard]] Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    // Converts roll, pitch, and yaw angles to a quaternion.
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
    // Configures PID gains, output limits, integrator limit, and derivative filtering.
    PIDController(float kp, float ki, float kd, float out_min, float out_max, float i_max, float d_filter_alpha = 1.0f);

    // Clears the controller's accumulated and previous state.
    void reset();
    // Computes a bounded PID output for the supplied error and timestep.
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
    // Creates the throttle and pitch controllers with their default gains.
    TECSController();
    // Computes throttle and pitch targets from altitude and speed errors.
    TECSOutputs update(const TECSInputs& in, float dt);
    // Resets both controllers and restores their initial output.
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
    // Creates the roll, pitch, yaw, and sideslip controllers.
    FlightController();

    // Converts course error and ground speed into a desired roll angle.
    void updateGPSCourse(float target_course_rad, float gps_course_rad, float ground_speed_m_s);

    // Converts attitude and rate errors into normalized servo and throttle outputs.
    ServoOutputs updateAttitude(float dt, 
                                const Quaternion& q_curr, 
                                const Vector3& gyro_rad_s, 
                                float accel_y_m_s2,
                                float current_yaw_rad,
                                const TECSOutputs& tecs);

private:
    // Wraps an angle into the range [-pi, pi).
    [[nodiscard]] float wrapPi(float angle) const;

    float target_roll_rad_ = 0.0f;
    
    float k_nav_ = 0.5f;   
    float k_p_att_ = 4.5f;

    float max_rate_roll_  = 2.0f; 
    float max_rate_pitch_ = 0.875f;
    float max_rate_yaw_   = 0.5f;

    PIDController rate_roll_pid_;
    PIDController rate_pitch_pid_;
    PIDController rate_yaw_pid_;
    PIDController sideslip_pid_;
};

}