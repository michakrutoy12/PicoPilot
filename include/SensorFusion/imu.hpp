#pragma once

inline float fastInvSqrt(float x);

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter() = default;

    void init(float gyroNoise, float biasNoise, float accelNoise);
    void predict(float gx, float gy, float gz, float dt);
    void updateAccel(float ax, float ay, float az);

    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const;
    void getGyroBias(float& bx, float& by, float& bz) const;
    void getEulerAngles(float& roll, float& pitch, float& yaw) const;
    void correctYawDrift(float gpsCourseRad, float gpsSpeed, float dt, float K_drift = 0.05f);
    void set_r_accel(float r_accel);

    void reset();

private:
    float x[7];
    
    float P[7][7];

    float Q_gyro;
    float Q_bias;
    float R_accel;
    float K_drift = 0.05f;
    
    void normalizeQuaternion();
    void matrixMultiply7x7(const float A[7][7], const float B[7][7], float C[7][7]);
};