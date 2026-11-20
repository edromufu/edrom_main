#include "mahony.h"

#include <math.h>

namespace {

float clampf(float v, float lim) {
    if (v > lim) return lim;
    if (v < -lim) return -lim;
    return v;
}

}  // namespace

void MahonyFilter::begin(float ax, float ay, float az) {
    // Quaternion a partir de roll/pitch do acelerômetro (yaw = 0)
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    q0_ = cr * cp;
    q1_ = sr * cp;
    q2_ = cr * sp;
    q3_ = -sr * sp;
    ix_ = iy_ = iz_ = 0.0f;
}

bool MahonyFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    bool accelUsed = false;
    const float norm = sqrtf(ax * ax + ay * ay + az * az);

    if (norm > 1e-6f) {
        const float dev = fabsf(norm - 1.0f);
        if (dev <= MAHONY_ACCEL_REJECT) {
            // Mesma regra do EKF: R do EKF cresce com (1 + k^2), aqui os
            // ganhos caem com 1 / (1 + k^2)
            const float k = dev / MAHONY_ACCEL_DEV_SCALE;
            const float trust = 1.0f / (1.0f + k * k);

            ax /= norm;
            ay /= norm;
            az /= norm;

            // Gravidade estimada no referencial do corpo
            const float vx = 2.0f * (q1_ * q3_ - q0_ * q2_);
            const float vy = 2.0f * (q0_ * q1_ + q2_ * q3_);
            const float vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;

            // Erro = medida x estimada
            const float ex = ay * vz - az * vy;
            const float ey = az * vx - ax * vz;
            const float ez = ax * vy - ay * vx;

            const float ki = MAHONY_KI * trust;
            ix_ = clampf(ix_ + ki * ex * dt, MAHONY_INTEGRAL_LIMIT);
            iy_ = clampf(iy_ + ki * ey * dt, MAHONY_INTEGRAL_LIMIT);
            iz_ = clampf(iz_ + ki * ez * dt, MAHONY_INTEGRAL_LIMIT);

            const float kp = MAHONY_KP * trust;
            gx += kp * ex;
            gy += kp * ey;
            gz += kp * ez;
            accelUsed = true;
        }
    }

    // O integral (estimativa do bias) continua sendo aplicado mesmo
    // quando o acelerômetro é descartado
    gx += ix_;
    gy += iy_;
    gz += iz_;

    // q' = 0.5 * q (x) [0, g]
    const float h = 0.5f * dt;
    const float a = q0_, b = q1_, c = q2_, d = q3_;
    q0_ += (-b * gx - c * gy - d * gz) * h;
    q1_ += ( a * gx + c * gz - d * gy) * h;
    q2_ += ( a * gy - b * gz + d * gx) * h;
    q3_ += ( a * gz + b * gy - c * gx) * h;

    const float qn = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (qn > 1e-9f) {
        q0_ /= qn;
        q1_ /= qn;
        q2_ /= qn;
        q3_ /= qn;
    }
    return accelUsed;
}

float MahonyFilter::roll() const {
    return atan2f(2.0f * (q0_ * q1_ + q2_ * q3_), 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_));
}

float MahonyFilter::pitch() const {
    float s = 2.0f * (q0_ * q2_ - q3_ * q1_);
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    return asinf(s);
}
