#include "aurea_kick/kick_engine.hpp"
#include <cmath>

namespace aurea_kick
{

KickEngine::KickEngine(const KickParameters & params) : params_(params) {}

void KickEngine::start(bool is_left_kick, const Eigen::Vector2d & initial_torso_pos, double initial_torso_yaw)
{
  is_left_kick_ = is_left_kick;
  current_phase_ = Phase::SHIFT_TO_SUPPORT; // CORRIGIDO
  phase_time_ = 0.0;
  
  torso_start_pos_ = initial_torso_pos;
  torso_start_yaw_ = initial_torso_yaw;
  kick_foot_start_pos_ = torso_start_pos_ + Eigen::Vector2d(0, is_left_kick_ ? params_.y_sep : -params_.y_sep);
  support_foot_start_pos_ = torso_start_pos_ + Eigen::Vector2d(0, is_left_kick_ ? -params_.y_sep : params_.y_sep);
}

bool KickEngine::update(double dt, Eigen::Vector3d & out_torso_pos, Eigen::Vector3d & out_support_foot_pos, Eigen::Vector3d & out_kick_foot_pos)
{
  phase_time_ += dt;
  
  out_support_foot_pos = {support_foot_start_pos_.x(), support_foot_start_pos_.y(), 0.0};
  out_kick_foot_pos = {kick_foot_start_pos_.x(), kick_foot_start_pos_.y(), 0.0};
  out_torso_pos = {torso_start_pos_.x(), torso_start_pos_.y(), 0.0};

  switch(current_phase_)
  {
    case Phase::SHIFT_TO_SUPPORT: // CORRIGIDO
      {
        Eigen::Vector2d com_target = support_foot_start_pos_;
        com_target.x() = torso_start_pos_.x();
        double phi = std::min(phase_time_ / params_.tA, 1.0);
        double h_phi = 0.5 * (1.0 - std::cos(M_PI * phi));
        Eigen::Vector2d current_torso_pos_2d = (1.0 - h_phi) * torso_start_pos_ + h_phi * com_target;
        out_torso_pos.head<2>() = current_torso_pos_2d;
        
        if (phase_time_ >= params_.tA) {
          phase_time_ = 0;
          current_phase_ = Phase::EXECUTE_KICK; // CORRIGIDO
        }
      }
      break;
    
    case Phase::EXECUTE_KICK: // CORRIGIDO
      {
        Eigen::Vector2d com_target = support_foot_start_pos_;
        com_target.x() = torso_start_pos_.x();
        out_torso_pos.head<2>() = com_target;
        Eigen::Vector3d kick_foot_offset = Eigen::Vector3d::Zero();
        double kick_phase_time = phase_time_;
        if (kick_phase_time < params_.tB) {
            double phi = kick_phase_time / params_.tB;
            double h_phi = 0.5 * (1.0 - std::cos(M_PI * phi));
            kick_foot_offset.x() = h_phi * (-params_.x_kick);
            kick_foot_offset.z() = h_phi * params_.z_kick;
        } else if (kick_phase_time < params_.tB + params_.tC) {
            double phase_t = kick_phase_time - params_.tB;
            double phi = phase_t / params_.tC;
            double kick_phi = (phi < 0.5) ? (2.0*phi*phi) : (1.0 - 2.0*(1.0-phi)*(1.0-phi));
            kick_foot_offset.x() = -params_.x_kick + (params_.x_kick * 1.5) * kick_phi;
            kick_foot_offset.z() = params_.z_kick;
        } else {
            double phase_t = kick_phase_time - (params_.tB + params_.tC);
            double phi = phase_t / params_.tD;
            double h_phi = 0.5 * (1.0 - std::cos(M_PI * phi));
            kick_foot_offset.x() = (1.0 - h_phi) * (params_.x_kick * 0.5);
            kick_foot_offset.z() = (1.0 - h_phi) * params_.z_kick;
        }
        
        Eigen::Rotation2Dd rot(torso_start_yaw_);
        Eigen::Vector2d world_offset = rot * kick_foot_offset.head<2>();
        out_kick_foot_pos.head<2>() += world_offset;
        out_kick_foot_pos.z() += kick_foot_offset.z();

        if (phase_time_ >= params_.tB + params_.tC + params_.tD) {
          phase_time_ = 0;
          current_phase_ = Phase::RETURN_TO_CENTER; // CORRIGIDO
        }
      }
      break;

    case Phase::RETURN_TO_CENTER: // CORRIGIDO
      {
        Eigen::Vector2d com_target = support_foot_start_pos_;
        com_target.x() = torso_start_pos_.x();
        double phi = std::min(phase_time_ / params_.tE, 1.0);
        double h_phi = 0.5 * (1.0 - std::cos(M_PI * phi));
        Eigen::Vector2d current_torso_pos_2d = (1.0 - h_phi) * com_target + h_phi * torso_start_pos_;
        out_torso_pos.head<2>() = current_torso_pos_2d;

        if (phase_time_ >= params_.tE) {
          phase_time_ = 0;
          current_phase_ = Phase::DONE; // CORRIGIDO
        }
      }
      break;

    case Phase::DONE: // CORRIGIDO
    case Phase::IDLE: // CORRIGIDO
      return false;
  }
  return true;
}

std::string KickEngine::get_phase_name() const
{
  switch(current_phase_) {
    case Phase::SHIFT_TO_SUPPORT: return "SHIFTING_WEIGHT"; // CORRIGIDO
    case Phase::EXECUTE_KICK: return "KICKING"; // CORRIGIDO
    case Phase::RETURN_TO_CENTER: return "RETURNING_TO_CENTER"; // CORRIGIDO
    case Phase::DONE: return "DONE"; // CORRIGIDO
    case Phase::IDLE: return "IDLE"; // CORRIGIDO
    default: return "UNKNOWN";
  }
}

} // namespace aurea_kick