#pragma once

inline float fastInvSqrt(float x);

class ExtendedKalmanFilter {
public:
    // Initializes the attitude filter with default noise parameters.
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter() = default;

    // Sets process and accelerometer measurement noise levels.
    void init(float gyroNoise, float biasNoise, float accelNoise);
    // Predicts attitude and gyro bias using angular-rate measurements.
    void predict(float gx, float gy, float gz, float dt);
    // Corrects attitude using a normalized accelerometer measurement.
    void updateAccel(float ax, float ay, float az);

    // Copies the current attitude quaternion to the output references.
    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const;
    // Copies the estimated gyro bias to the output references.
    void getGyroBias(float& bx, float& by, float& bz) const;
    // Converts the current quaternion to roll, pitch, and yaw.
    void getEulerAngles(float& roll, float& pitch, float& yaw) const;
    // Adjusts gyro-bias estimates toward the GPS course while moving.
    void correctYawDrift(float gpsCourseRad, float gpsSpeed, float dt, float K_drift = 0.05f);
    // Sets the accelerometer measurement-noise value used for correction.
    void set_r_accel(float r_accel);

    // Restores the identity attitude and initial covariance.
    void reset();

private:
    float x[7];
    
    float P[7][7];

    float Q_gyro;
    float Q_bias;
    float R_accel;
    float K_drift = 0.05f;
    
    // Keeps the estimated attitude quaternion at unit length.
    void normalizeQuaternion();
    // Multiplies two 7-by-7 matrices.
    void matrixMultiply7x7(const float A[7][7], const float B[7][7], float C[7][7]);
};