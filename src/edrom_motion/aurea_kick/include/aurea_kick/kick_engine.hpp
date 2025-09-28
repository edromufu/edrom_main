#ifndef AUREA_KICK_KICK_ENGINE_HPP_
#define AUREA_KICK_KICK_ENGINE_HPP_

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace aurea_kick
{

struct KickParameters {
  double tA, tB, tC, tD, tE;
  double x_kick, z_kick, y_sep;
  double com_height;
};

struct LinkData {
  std::string name;
  std::string parent_name;
  double mass;
  Eigen::Vector3d com_position_local;
  Eigen::Vector3d joint_axis_parent;
  Eigen::Vector3d translation_from_parent;
};


class KickEngine
{
public:
  enum class Phase {  
    SHIFT_TO_SUPPORT,
    EXECUTE_KICK,
    RETURN_TO_CENTER,
    DONE,
    IDLE
  };

  KickEngine(const KickParameters & params);
  void start(bool is_left_kick, const Eigen::Vector2d & initial_torso_pos, double initial_torso_yaw);
  bool update(double dt, Eigen::Vector3d & out_torso_pos, Eigen::Vector3d & out_support_foot_pos, Eigen::Vector3d & out_kick_foot_pos);
  Phase get_current_phase() const { return current_phase_; }
  std::string get_phase_name() const;

private:  
  KickParameters params_;
  Phase current_phase_{Phase::IDLE};
  double phase_time_{0.0};
  bool is_left_kick_{false};
  
  // Parâmetros de trajetória
  Eigen::Vector2d torso_start_pos_;
  double torso_start_yaw_;
  Eigen::Vector2d support_foot_start_pos_;
  Eigen::Vector2d kick_foot_start_pos_;
};

} // namespace aurea_kick
#endif // AUREA_KICK_KICK_ENGINE_HPP_