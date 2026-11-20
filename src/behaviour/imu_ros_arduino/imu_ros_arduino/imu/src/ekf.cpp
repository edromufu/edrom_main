#include "ekf.h"

#include <math.h>

namespace {

const float PI_F = 3.14159265358979f;
// Evita a divisão por zero em tan(pitch) perto de ±90° (gimbal lock
// da representação de Euler, ex.: robô caído de frente)
const float COS_PITCH_MIN = 0.05f;

}  // namespace

void AttitudeEKF::begin(float ax, float ay, float az) {
    x_[0] = atan2f(ay, az);
    x_[1] = atan2f(-ax, sqrtf(ay * ay + az * az));
    x_[2] = 0.0f;
    x_[3] = 0.0f;

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            P_[i][j] = 0.0f;
    P_[0][0] = P_[1][1] = 0.1f * 0.1f;     // ~6° de incerteza inicial
    P_[2][2] = P_[3][3] = 0.01f * 0.01f;   // bias já calibrado no boot
}

void AttitudeEKF::predict(float gx, float gy, float gz, float dt) {
    const float phi = x_[0];
    const float th = x_[1];
    const float pc = gx - x_[2];
    const float qc = gy - x_[3];
    const float r = gz;

    const float sphi = sinf(phi), cphi = cosf(phi);
    float cth = cosf(th);
    if (fabsf(cth) < COS_PITCH_MIN) cth = copysignf(COS_PITCH_MIN, cth);
    const float tth = sinf(th) / cth;

    // Propagação do estado
    x_[0] += (pc + sphi * tth * qc + cphi * tth * r) * dt;
    x_[1] += (cphi * qc - sphi * r) * dt;
    normalizeAngles();

    // Jacobiano contínuo A = df/dx (linhas dos biases são zero)
    float A[2][4];
    A[0][0] = (cphi * qc - sphi * r) * tth;
    A[0][1] = (sphi * qc + cphi * r) / (cth * cth);
    A[0][2] = -1.0f;
    A[0][3] = -sphi * tth;
    A[1][0] = -sphi * qc - cphi * r;
    A[1][1] = 0.0f;
    A[1][2] = 0.0f;
    A[1][3] = -cphi;

    // F = I + A dt  ->  P = F P F^T + Q
    float F[4][4] = {};
    for (int i = 0; i < 4; i++) F[i][i] = 1.0f;
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 4; j++)
            F[i][j] += A[i][j] * dt;

    float FP[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += F[i][k] * P_[k][j];
            FP[i][j] = s;
        }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += FP[i][k] * F[j][k];
            P_[i][j] = s;
        }

    const float qAngle = EKF_GYRO_NOISE * EKF_GYRO_NOISE * dt;
    const float qBias = EKF_BIAS_NOISE * EKF_BIAS_NOISE * dt;
    P_[0][0] += qAngle;
    P_[1][1] += qAngle;
    P_[2][2] += qBias;
    P_[3][3] += qBias;
    symmetrize();
}

bool AttitudeEKF::update(float ax, float ay, float az) {
    const float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return false;

    const float dev = fabsf(norm - 1.0f);
    if (dev > EKF_ACCEL_REJECT) return false;   // impacto: confia só no giro

    const float z[3] = {ax / norm, ay / norm, az / norm};

    const float phi = x_[0], th = x_[1];
    const float sphi = sinf(phi), cphi = cosf(phi);
    const float sth = sinf(th), cth = cosf(th);

    const float h[3] = {-sth, sphi * cth, cphi * cth};

    // H (3x4): só as colunas de roll e pitch são não nulas
    float H[3][2];
    H[0][0] = 0.0f;         H[0][1] = -cth;
    H[1][0] = cphi * cth;   H[1][1] = -sphi * sth;
    H[2][0] = -sphi * cth;  H[2][1] = -cphi * sth;

    // R adaptativo
    const float k = dev / EKF_ACCEL_DEV_SCALE;
    const float rVar = EKF_ACCEL_NOISE * EKF_ACCEL_NOISE * (1.0f + k * k);

    // PHt = P H^T (4x3)
    float PHt[4][3];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
            PHt[i][j] = P_[i][0] * H[j][0] + P_[i][1] * H[j][1];

    // S = H P H^T + R (3x3)
    float S[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S[i][j] = H[i][0] * PHt[0][j] + H[i][1] * PHt[1][j];
    for (int i = 0; i < 3; i++) S[i][i] += rVar;

    // S^-1 pela adjunta
    const float c00 = S[1][1] * S[2][2] - S[1][2] * S[2][1];
    const float c01 = S[1][2] * S[2][0] - S[1][0] * S[2][2];
    const float c02 = S[1][0] * S[2][1] - S[1][1] * S[2][0];
    const float det = S[0][0] * c00 + S[0][1] * c01 + S[0][2] * c02;
    if (fabsf(det) < 1e-12f) return false;
    const float inv = 1.0f / det;

    float Si[3][3];
    Si[0][0] = c00 * inv;
    Si[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * inv;
    Si[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * inv;
    Si[1][0] = c01 * inv;
    Si[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * inv;
    Si[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) * inv;
    Si[2][0] = c02 * inv;
    Si[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) * inv;
    Si[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * inv;

    // K = P H^T S^-1 (4x3)
    float K[4][3];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
            K[i][j] = PHt[i][0] * Si[0][j] + PHt[i][1] * Si[1][j] + PHt[i][2] * Si[2][j];

    // x = x + K (z - h)
    const float y[3] = {z[0] - h[0], z[1] - h[1], z[2] - h[2]};
    for (int i = 0; i < 4; i++)
        x_[i] += K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
    normalizeAngles();

    // P = (I - K H) P = P - K (H P),  com H P = (P H^T)^T
    float newP[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            newP[i][j] = P_[i][j] - (K[i][0] * PHt[j][0] + K[i][1] * PHt[j][1] + K[i][2] * PHt[j][2]);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            P_[i][j] = newP[i][j];
    symmetrize();
    return true;
}

float AttitudeEKF::rollStd() const { return sqrtf(P_[0][0] > 0.0f ? P_[0][0] : 0.0f); }
float AttitudeEKF::pitchStd() const { return sqrtf(P_[1][1] > 0.0f ? P_[1][1] : 0.0f); }

void AttitudeEKF::normalizeAngles() {
    // Pitch de Euler fica em [-90°, 90°]; ao passar, a mesma atitude é
    // representada com roll + 180° e pitch espelhado
    if (x_[1] > PI_F / 2.0f) {
        x_[1] = PI_F - x_[1];
        x_[0] += PI_F;
    } else if (x_[1] < -PI_F / 2.0f) {
        x_[1] = -PI_F - x_[1];
        x_[0] += PI_F;
    }
    while (x_[0] > PI_F) x_[0] -= 2.0f * PI_F;
    while (x_[0] < -PI_F) x_[0] += 2.0f * PI_F;
}

void AttitudeEKF::symmetrize() {
    for (int i = 0; i < 4; i++)
        for (int j = i + 1; j < 4; j++) {
            const float m = 0.5f * (P_[i][j] + P_[j][i]);
            P_[i][j] = P_[j][i] = m;
        }
}
