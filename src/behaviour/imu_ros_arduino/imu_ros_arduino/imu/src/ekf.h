#pragma once

// EKF de atitude (roll/pitch) para giroscópio + acelerômetro (MPU-6050).
//
// Estado: x = [roll, pitch, bias_gx, bias_gy]   (rad, rad, rad/s, rad/s)
//
// Predição: cinemática de Euler ZYX com o giroscópio (p, q, r) sem bias
//   roll'  = p + sin(roll) tan(pitch) q + cos(roll) tan(pitch) r
//   pitch' =     cos(roll) q            - sin(roll) r
// Medição: direção da gravidade vista pelo acelerômetro (normalizada)
//   h(x) = [-sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch)]
//
// O bias em z não é estimado: sem magnetômetro ele não é observável
// (vem só da calibração no boot). Yaw também não é estimado.
//
// R adaptativo: durante impactos/acelerações (|a| longe de 1 g) o
// acelerômetro não mede só a gravidade, então a confiança nele cai e,
// acima de um limite, a correção é descartada.
//
// Os parâmetros podem ser sobrescritos por build_flags no platformio.ini.

#ifndef EKF_GYRO_NOISE
#define EKF_GYRO_NOISE 0.002f        // densidade de ruído do giro (rad/s/sqrt(Hz))
#endif
#ifndef EKF_BIAS_NOISE
#define EKF_BIAS_NOISE 0.0001f       // passeio aleatório do bias (rad/s/sqrt(s))
#endif
#ifndef EKF_ACCEL_NOISE
#define EKF_ACCEL_NOISE 0.05f        // desvio do acelerômetro normalizado (~rad)
#endif
#ifndef EKF_ACCEL_DEV_SCALE
#define EKF_ACCEL_DEV_SCALE 0.05f    // | |a| - 1 g | onde R começa a crescer (g)
#endif
#ifndef EKF_ACCEL_REJECT
#define EKF_ACCEL_REJECT 0.3f        // | |a| - 1 g | acima disso ignora o acelerômetro (g)
#endif

class AttitudeEKF {
public:
    // Inicializa a partir de uma leitura do acelerômetro (qualquer unidade)
    void begin(float ax, float ay, float az);

    // gx, gy, gz em rad/s (bias de calibração já removido), dt em segundos
    void predict(float gx, float gy, float gz, float dt);

    // Aceleração em g. Retorna false se a medida foi rejeitada (impacto).
    bool update(float ax, float ay, float az);

    float roll() const { return x_[0]; }    // rad
    float pitch() const { return x_[1]; }   // rad
    float biasX() const { return x_[2]; }   // rad/s
    float biasY() const { return x_[3]; }   // rad/s
    float rollStd() const;                  // rad (sqrt de P[0][0])
    float pitchStd() const;                 // rad (sqrt de P[1][1])

private:
    void normalizeAngles();
    void symmetrize();

    float x_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float P_[4][4] = {};
};
