#ifndef AUREA_WALK_TORSO_ORIENTATION_CONTROLLER_HPP_
#define AUREA_WALK_TORSO_ORIENTATION_CONTROLLER_HPP_

// Controlador de inclinação do torso (Maximo, 2017, seção 4.4.2).
//
// Controladores P+V desacoplados para roll e pitch. O termo derivativo usa a
// velocidade angular medida pelo giroscópio (não a derivada do ângulo), para
// não amplificar ruído. Cada junta recebe um deslocamento de comando:
//
//   Δθ_j = Kp_j · e + Kv_j · ω        e = medido − desejado
//
// roll  -> hip_roll, ank_roll        (ω = p, giro em x)
// pitch -> hip_pitch, knee, ank_pitch (ω = q, giro em y)
//
// Os ganhos têm SINAL: o sinal define o sentido da correção no espaço de
// juntas do robô e precisa ser verificado experimentalmente.
//
// Classe pura (sem ROS) para poder ser testada e reutilizada (ex.: kick).

#include <array>
#include <map>
#include <string>

namespace aurea_walk
{

struct TorsoControllerParams
{
  bool enabled = false;

  double roll_ref = 0.0;   // rad, inclinação desejada do torso
  double pitch_ref = 0.0;  // rad
  double deadband = 0.0;   // rad, erro abaixo disso é ignorado

  // Ganhos com sinal [rad/rad] e [rad/(rad/s)]
  double kp_hip_roll = 0.0, kv_hip_roll = 0.0;
  double kp_ank_roll = 0.0, kv_ank_roll = 0.0;
  double kp_hip_pitch = 0.0, kv_hip_pitch = 0.0;
  double kp_knee = 0.0, kv_knee = 0.0;
  double kp_ank_pitch = 0.0, kv_ank_pitch = 0.0;

  double max_offset = 0.15;        // rad, saturação por junta
  double max_offset_rate = 1.0;    // rad/s, limite de variação (entra/sai suave)
  double fall_angle = 0.6;         // rad; acima disso o robô está caindo: não corrige
};

struct ImuSample
{
  double roll = 0.0;        // rad
  double pitch = 0.0;       // rad
  double roll_rate = 0.0;   // rad/s (giro x, p)
  double pitch_rate = 0.0;  // rad/s (giro y, q)
};

class TorsoOrientationController
{
public:
  // Motivo pelo qual a correção está (ou não) ativa, para diagnóstico
  enum class Status { ACTIVE, DISABLED, IMU_INVALID, FALLING, INACTIVE_STATE };

  void set_params(const TorsoControllerParams & params) { params_ = params; }
  const TorsoControllerParams & params() const { return params_; }

  // w_left / w_right ∈ [0, 1]: participação de cada perna (apoio = 1,
  // balanço = 0, interpolado no duplo apoio). active_state = estado do walking
  // em que a correção faz sentido. Nunca dá saltos: quando desativado, os
  // deslocamentos voltam a zero respeitando max_offset_rate.
  std::map<std::string, double> compute(
    const ImuSample & imu, bool imu_valid, bool active_state,
    double w_left, double w_right, double dt);

  void reset() { offsets_.fill(0.0); status_ = Status::DISABLED; }
  Status status() const { return status_; }
  static const char * status_name(Status s);

private:
  // Ordem: hip_roll, ank_roll, hip_pitch, knee, ank_pitch  × {l, r}
  static constexpr int kJoints = 10;
  static const std::array<const char *, kJoints> kNames;

  TorsoControllerParams params_;
  std::array<double, kJoints> offsets_{};  // deslocamentos atuais (já suavizados)
  Status status_{Status::DISABLED};
};

}  // namespace aurea_walk

#endif  // AUREA_WALK_TORSO_ORIENTATION_CONTROLLER_HPP_
