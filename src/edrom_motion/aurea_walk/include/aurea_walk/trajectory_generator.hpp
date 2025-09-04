#ifndef AUREA_WALK_TRAJECTORY_GENERATOR_HPP_
#define AUREA_WALK_TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>

namespace aurea_walk
{

// Estrutura para armazenar dados de pose de forma simplificada no frame do mundo
struct PoseData
{
  Eigen::Vector2d position{0.0, 0.0};
  double yaw{0.0};
  bool is_left{false};
};

// Gera a pose do CoM (torso) para um tempo 't' dentro do passo
void get_com_pose_at_time(
  Eigen::Vector2d & out_com_pos, double & out_com_yaw,
  double t, double T, double ds_ratio, double z_com, double g,
  const PoseData & p_start, const PoseData & p_end, 
  const PoseData & p_support);
// Gera a pose do pé de balanço no frame do mundo para um tempo 't'
void get_swing_foot_pose_at_time(
  Eigen::Vector3d & out_swing_pos_world, double & out_swing_yaw_world,
  double t, double T, double z_step, double ds_ratio,
  const PoseData & p_start, const PoseData & p_end);

// Calcula as poses alvo para o fim do passo (Baseado na Seção 3.2 da tese - Simplificado)
void select_next_poses(
  PoseData & out_next_torso, PoseData & out_next_swing,
  const PoseData & current_torso, const PoseData & swing_foot, const geometry_msgs::msg::Twist & v_cmd,
  double T, double y_sep);

}  // namespace aurea_walk

#endif  // aurea_walk_TRAJECTORY_GENERATOR_HPP_