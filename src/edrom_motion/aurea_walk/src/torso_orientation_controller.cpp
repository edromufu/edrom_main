#include "aurea_walk/torso_orientation_controller.hpp"

#include <algorithm>
#include <cmath>

namespace aurea_walk
{

const std::array<const char *, TorsoOrientationController::kJoints>
TorsoOrientationController::kNames = {
  "l_hip_roll", "l_ank_roll", "l_hip_pitch", "l_knee", "l_ank_pitch",
  "r_hip_roll", "r_ank_roll", "r_hip_pitch", "r_knee", "r_ank_pitch"};

namespace
{
double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

double apply_deadband(double e, double db)
{
  if (std::abs(e) <= db) {return 0.0;}
  return e > 0.0 ? e - db : e + db;  // contínuo na borda da zona morta
}
}  // namespace

const char * TorsoOrientationController::status_name(Status s)
{
  switch (s) {
    case Status::ACTIVE: return "ACTIVE";
    case Status::DISABLED: return "DISABLED";
    case Status::IMU_INVALID: return "IMU_INVALID";
    case Status::FALLING: return "FALLING";
    case Status::INACTIVE_STATE: return "INACTIVE_STATE";
  }
  return "?";
}

std::map<std::string, double> TorsoOrientationController::compute(
  const ImuSample & imu, bool imu_valid, bool active_state,
  double w_left, double w_right, double dt)
{
  const auto & p = params_;
  std::array<double, kJoints> target{};  // zero = sem correção

  const bool finite = std::isfinite(imu.roll) && std::isfinite(imu.pitch) &&
    std::isfinite(imu.roll_rate) && std::isfinite(imu.pitch_rate);

  if (!p.enabled) {
    status_ = Status::DISABLED;
  } else if (!imu_valid || !finite) {
    status_ = Status::IMU_INVALID;
  } else if (std::abs(imu.roll) > p.fall_angle || std::abs(imu.pitch) > p.fall_angle) {
    status_ = Status::FALLING;
  } else if (!active_state) {
    status_ = Status::INACTIVE_STATE;
  } else {
    status_ = Status::ACTIVE;
    const double e_roll = apply_deadband(imu.roll - p.roll_ref, p.deadband);
    const double e_pitch = apply_deadband(imu.pitch - p.pitch_ref, p.deadband);

    const double hip_roll = p.kp_hip_roll * e_roll + p.kv_hip_roll * imu.roll_rate;
    const double ank_roll = p.kp_ank_roll * e_roll + p.kv_ank_roll * imu.roll_rate;
    const double hip_pitch = p.kp_hip_pitch * e_pitch + p.kv_hip_pitch * imu.pitch_rate;
    const double knee = p.kp_knee * e_pitch + p.kv_knee * imu.pitch_rate;
    const double ank_pitch = p.kp_ank_pitch * e_pitch + p.kv_ank_pitch * imu.pitch_rate;

    const std::array<double, 5> per_leg = {hip_roll, ank_roll, hip_pitch, knee, ank_pitch};
    const double wl = clamp(w_left, 0.0, 1.0);
    const double wr = clamp(w_right, 0.0, 1.0);
    for (int i = 0; i < 5; ++i) {
      target[i] = clamp(wl * per_leg[i], -p.max_offset, p.max_offset);
      target[i + 5] = clamp(wr * per_leg[i], -p.max_offset, p.max_offset);
    }
  }

  // Limita a taxa de variação: liga, desliga e troca de perna sem trancos
  const double max_step = std::max(0.0, p.max_offset_rate) * std::max(0.0, dt);
  std::map<std::string, double> out;
  for (int i = 0; i < kJoints; ++i) {
    offsets_[i] += clamp(target[i] - offsets_[i], -max_step, max_step);
    out[kNames[i]] = offsets_[i];
  }
  return out;
}

}  // namespace aurea_walk
