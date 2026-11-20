#pragma once

// Filtro de Mahony (complementar não linear em quaternion) para
// giroscópio + acelerômetro (MPU-6050).
//
// - Termo proporcional (Kp): puxa a gravidade estimada para a medida.
// - Termo integral (Ki): estima o bias do giroscópio (bias ~= -integral).
//
// Para uma comparação justa com o EKF (ekf.h), o acelerômetro é tratado
// com a MESMA regra: a confiança nele cai quando |a| se afasta de 1 g e,
// acima de MAHONY_ACCEL_REJECT, a correção é descartada (só giroscópio).
//
// Mesma convenção do EKF: roll/pitch de Euler ZYX, com a gravidade no
// corpo dada por [-sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch)].
// Yaw não é observável sem magnetômetro (fica só integrando o giro).
//
// Os parâmetros podem ser sobrescritos por build_flags no platformio.ini.

#ifndef MAHONY_KP
#define MAHONY_KP 1.0f               // ganho proporcional (1/s): constante de tempo ~1/Kp
#endif
#ifndef MAHONY_KI
#define MAHONY_KI 0.02f              // ganho integral (estimativa do bias)
#endif
#ifndef MAHONY_INTEGRAL_LIMIT
#define MAHONY_INTEGRAL_LIMIT 0.1f   // anti-windup do integral (rad/s)
#endif
#ifndef MAHONY_ACCEL_DEV_SCALE
#define MAHONY_ACCEL_DEV_SCALE 0.05f // | |a| - 1 g | onde os ganhos começam a cair (g)
#endif
#ifndef MAHONY_ACCEL_REJECT
#define MAHONY_ACCEL_REJECT 0.3f     // | |a| - 1 g | acima disso ignora o acelerômetro (g)
#endif

class MahonyFilter {
public:
    // Inicializa a partir de uma leitura do acelerômetro (qualquer unidade)
    void begin(float ax, float ay, float az);

    // Giroscópio em rad/s (bias de calibração já removido), aceleração em g,
    // dt em segundos. Retorna false se o acelerômetro foi descartado.
    bool update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    float roll() const;                      // rad
    float pitch() const;                     // rad
    float biasX() const { return -ix_; }     // rad/s
    float biasY() const { return -iy_; }     // rad/s

private:
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    float ix_ = 0.0f, iy_ = 0.0f, iz_ = 0.0f;  // termo integral (rad/s)
};
