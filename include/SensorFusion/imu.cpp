#include "imu.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace {
float fastInvSqrt(float x) {
    if (!std::isfinite(x) || x <= 1e-12f) return 0.0f;
    return 1.0f / std::sqrt(x);
}
}

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    if (!init(0.005f, 0.00001f, 0.015f)) reset();
}

void ExtendedKalmanFilter::reset() {
    x[0] = 1.0f; x[1] = 0.0f; x[2] = 0.0f; x[3] = 0.0f;
    x[4] = 0.0f; x[5] = 0.0f; x[6] = 0.0f;

    std::memset(P, 0, sizeof(P));
    for (int i = 0; i < 4; i++) P[i][i] = 0.01f;
    for (int i = 4; i < 7; i++) P[i][i] = 0.001f;
}

bool ExtendedKalmanFilter::init(float gyroNoise, float biasNoise, float accelNoise) {
    if (!std::isfinite(gyroNoise) || gyroNoise < 0.0f ||
        !std::isfinite(biasNoise) || biasNoise < 0.0f ||
        !std::isfinite(accelNoise) || accelNoise <= 0.0f ||
        gyroNoise > 1e6f || biasNoise > 1e6f || accelNoise > 1e6f) {
        return false;
    }

    Q_gyro = gyroNoise;
    Q_bias = biasNoise;
    R_accel = accelNoise;
    reset();
    return true;
}

bool ExtendedKalmanFilter::set_r_accel(float r_accel){
    if (!std::isfinite(r_accel) || r_accel <= 0.0f || r_accel > 1e6f) return false;
    R_accel = r_accel;
    return true;
}

bool ExtendedKalmanFilter::stateIsFinite() const {
    for (float value : x) {
        if (!std::isfinite(value)) return false;
    }
    for (const auto& row : P) {
        for (float value : row) {
            if (!std::isfinite(value)) return false;
        }
    }
    return true;
}

bool ExtendedKalmanFilter::normalizeQuaternion() {
    float normSq = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
    float invNorm = fastInvSqrt(normSq);
    if (invNorm == 0.0f) {
        reset();
        return false;
    }

    x[0] *= invNorm; x[1] *= invNorm; x[2] *= invNorm; x[3] *= invNorm;
    return true;
}

bool ExtendedKalmanFilter::predict(float gx, float gy, float gz, float dt) {
    if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz) ||
        !std::isfinite(dt) || dt <= 0.0f || dt > 0.02f) {
        return false;
    }
    if (!stateIsFinite()) {
        reset();
        return false;
    }

    float wx = gx - x[4];
    float wy = gy - x[5];
    float wz = gz - x[6];

    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

    x[0] += 0.5f * (-q1*wx - q2*wy - q3*wz) * dt;
    x[1] += 0.5f * ( q0*wx - q3*wy + q2*wz) * dt;
    x[2] += 0.5f * ( q3*wx + q0*wy - q1*wz) * dt;
    x[3] += 0.5f * (-q2*wx + q1*wy + q0*wz) * dt;
    const float predicted_norm_sq = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
    const float predicted_norm = std::sqrt(predicted_norm_sq);
    if (!normalizeQuaternion()) return false;

    float F[7][7];
    std::memset(F, 0, sizeof(F));

    F[0][0] = 1.0f;        F[0][1] = -0.5f*wx*dt; F[0][2] = -0.5f*wy*dt; F[0][3] = -0.5f*wz*dt;
    F[1][0] = 0.5f*wx*dt;  F[1][1] = 1.0f;        F[1][2] =  0.5f*wz*dt; F[1][3] = -0.5f*wy*dt;
    F[2][0] = 0.5f*wy*dt;  F[2][1] = -0.5f*wz*dt; F[2][2] = 1.0f;        F[2][3] =  0.5f*wx*dt;
    F[3][0] = 0.5f*wz*dt;  F[3][1] =  0.5f*wy*dt; F[3][2] = -0.5f*wx*dt; F[3][3] = 1.0f;

    F[0][4] =  0.5f*q1*dt; F[0][5] =  0.5f*q2*dt; F[0][6] =  0.5f*q3*dt;
    F[1][4] = -0.5f*q0*dt; F[1][5] =  0.5f*q3*dt; F[1][6] = -0.5f*q2*dt;
    F[2][4] = -0.5f*q3*dt; F[2][5] = -0.5f*q0*dt; F[2][6] =  0.5f*q1*dt;
    F[3][4] =  0.5f*q2*dt; F[3][5] = -0.5f*q1*dt; F[3][6] = -0.5f*q0*dt;

    float F_raw[4][7];
    std::memcpy(F_raw, F, sizeof(F_raw));
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 7; j++) {
            F[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                const float normalization_jacobian =
                    ((i == k ? 1.0f : 0.0f) - x[i] * x[k]) / predicted_norm;
                F[i][j] += normalization_jacobian * F_raw[k][j];
            }
        }
    }

    float FP[7][7];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 7; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }
    for (int i = 4; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            FP[i][j] = P[i][j];
        }
    }

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = 0.0f;
            for (int k = 0; k < 7; k++) {
                P[i][j] += FP[i][k] * F[j][k];
            }
        }
        for (int j = 4; j < 7; j++) {
            P[i][j] = FP[i][j];
        }
    }

    const float G[4][3] = {
        {-0.5f*q1, -0.5f*q2, -0.5f*q3},
        { 0.5f*q0, -0.5f*q3,  0.5f*q2},
        { 0.5f*q3,  0.5f*q0, -0.5f*q1},
        {-0.5f*q2,  0.5f*q1,  0.5f*q0}
    };
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float gyro_noise_covariance = 0.0f;
            for (int k = 0; k < 3; k++) {
                gyro_noise_covariance += G[i][k] * G[j][k];
            }
            P[i][j] += Q_gyro * dt * gyro_noise_covariance;
        }
    }
    for (int i = 4; i < 7; i++) P[i][i] += Q_bias * dt;
    if (!stateIsFinite()) {
        reset();
        return false;
    }
    return true;
}

bool ExtendedKalmanFilter::updateAccel(float ax, float ay, float az) {
    float normSq = ax*ax + ay*ay + az*az;
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
        !std::isfinite(normSq)) return false;
    if (normSq < 25.0f || normSq > 225.0f) return false;
    if (!stateIsFinite()) {
        reset();
        return false;
    }
    
    float invNorm = fastInvSqrt(normSq);
    if (invNorm == 0.0f) return false;
    ax *= invNorm; ay *= invNorm; az *= invNorm;

    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

    float hx = 2.0f * (q1*q3 - q0*q2);
    float hy = 2.0f * (q0*q1 + q2*q3);
    float hz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float z_y[3] = { ax - hx, ay - hy, az - hz };

    float H[3][4]; 
    H[0][0] = -2.0f*q2; H[0][1] =  2.0f*q3; H[0][2] = -2.0f*q0; H[0][3] =  2.0f*q1;
    H[1][0] =  2.0f*q1; H[1][1] =  2.0f*q0; H[1][2] =  2.0f*q3; H[1][3] =  2.0f*q2;
    H[2][0] =  2.0f*q0; H[2][1] = -2.0f*q1; H[2][2] = -2.0f*q2; H[2][3] =  2.0f*q3;

    float PH_T[7][3];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            PH_T[i][j] = P[i][0]*H[j][0] + P[i][1]*H[j][1] + P[i][2]*H[j][2] + P[i][3]*H[j][3];
        }
    }

    float S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = H[i][0]*PH_T[0][j] + H[i][1]*PH_T[1][j] + H[i][2]*PH_T[2][j] + H[i][3]*PH_T[3][j];
            if (i == j) S[i][j] += R_accel;
        }
    }

    float det = S[0][0] * (S[1][1] * S[2][2] - S[2][1] * S[1][2]) -
                S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
                S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);

    if (!std::isfinite(det) || det <= 1e-12f) {
        return false;
    }
    float invDet = 1.0f / det;

    float S_inv[3][3];
    S_inv[0][0] = (S[1][1] * S[2][2] - S[2][1] * S[1][2]) * invDet;
    S_inv[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * invDet;
    S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * invDet;
    S_inv[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) * invDet;
    S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * invDet;
    S_inv[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) * invDet;
    S_inv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * invDet;
    S_inv[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) * invDet;
    S_inv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * invDet;

    float K[7][3];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = PH_T[i][0]*S_inv[0][j] + PH_T[i][1]*S_inv[1][j] + PH_T[i][2]*S_inv[2][j];
        }
    }

    for (int i = 0; i < 7; i++) {
        x[i] += K[i][0]*z_y[0] + K[i][1]*z_y[1] + K[i][2]*z_y[2];
    }
    const float corrected_norm_sq = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
    const float corrected_norm = std::sqrt(corrected_norm_sq);
    if (!normalizeQuaternion()) return false;

    float P_prior[7][7];
    std::memcpy(P_prior, P, sizeof(P_prior));

    float A[7][7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            float kh = 0.0f;
            if (j < 4) {
                kh = K[i][0]*H[0][j] + K[i][1]*H[1][j] + K[i][2]*H[2][j];
            }
            A[i][j] = (i == j ? 1.0f : 0.0f) - kh;
        }
    }

    float AP[7][7] = {};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                AP[i][j] += A[i][k] * P_prior[k][j];
            }
        }
    }

    float updatedP[7][7] = {};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                updatedP[i][j] += AP[i][k] * A[j][k];
            }
            for (int k = 0; k < 3; k++) {
                updatedP[i][j] += K[i][k] * R_accel * K[j][k];
            }
        }
    }
    float normalization_jacobian[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            normalization_jacobian[i][j] =
                ((i == j ? 1.0f : 0.0f) - x[i] * x[j]) / corrected_norm;
        }
    }

    float normalization_transform[7][7] = {};
    for (int i = 0; i < 7; i++) normalization_transform[i][i] = 1.0f;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            normalization_transform[i][j] = normalization_jacobian[i][j];
        }
    }

    float transformedP[7][7] = {};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                transformedP[i][j] += normalization_transform[i][k] * updatedP[k][j];
            }
        }
    }

    float normalizedP[7][7] = {};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                normalizedP[i][j] += transformedP[i][k] * normalization_transform[j][k];
            }
        }
    }
    std::memcpy(P, normalizedP, sizeof(P));
    if (!stateIsFinite()) {
        reset();
        return false;
    }
    return true;
}

void ExtendedKalmanFilter::getQuaternion(float& q0, float& q1, float& q2, float& q3) const {
    q0 = x[0]; q1 = x[1]; q2 = x[2]; q3 = x[3];
}

void ExtendedKalmanFilter::getGyroBias(float& bx, float& by, float& bz) const {
    bx = x[4]; by = x[5]; bz = x[6];
}

void ExtendedKalmanFilter::getEulerAngles(float& roll, float& pitch, float& yaw) const {
    roll = std::atan2f(2.0f*(x[0]*x[1] + x[2]*x[3]), 1.0f - 2.0f*(x[1]*x[1] + x[2]*x[2]));
    
    float sin_pitch = std::clamp(2.0f * (x[0]*x[2] - x[3]*x[1]), -1.0f, 1.0f);
    pitch = std::asinf(sin_pitch);
    
    yaw = std::atan2f(2.0f*(x[0]*x[3] + x[1]*x[2]), 1.0f - 2.0f*(x[2]*x[2] + x[3]*x[3]));
}

void ExtendedKalmanFilter::correctYawDrift(float gpsCourseRad, float gpsSpeed, float dt, float K_drift) {
    if (!std::isfinite(gpsCourseRad) || !std::isfinite(gpsSpeed) ||
        !std::isfinite(dt) || !std::isfinite(K_drift) ||
        gpsSpeed < 1.0f || dt <= 0.0f || dt > 0.02f ||
        gpsCourseRad < 0.0f || gpsCourseRad > 6.2831855f ||
        K_drift < 0.0f || K_drift > 1.0f || !stateIsFinite()) return;

    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

    float current_yaw = std::atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    
    float error_yaw = std::fmod(gpsCourseRad - current_yaw + 3.14159265f, 2.0f * 3.14159265f);
    if (error_yaw < 0.0f) error_yaw += 2.0f * 3.14159265f;
    error_yaw -= 3.14159265f;

    float z_x = 2.0f * (q1 * q3 - q0 * q2);
    float z_y = 2.0f * (q2 * q3 + q0 * q1);
    float z_z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    x[4] -= K_drift * error_yaw * z_x * dt;
    x[5] -= K_drift * error_yaw * z_y * dt;
    x[6] -= K_drift * error_yaw * z_z * dt;
    if (!stateIsFinite()) reset();
}