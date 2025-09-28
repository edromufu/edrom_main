#ifndef AUREA_KICK_KICK_NODE_HPP_
#define AUREA_KICK_KICK_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "aurea_walk/srv/solve_ik.hpp" // Seu serviço de IK
#include "aurea_kick/action/kick.hpp"
#include "aurea_kick/kick_engine.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <map>
#include <string>

using namespace std::placeholders;

// NOVO: Estrutura para descrever as propriedades de um link (peça) do robô
struct LinkData {
  std::string name;
  std::string parent_name;
  double mass;
  Eigen::Vector3d com_position_local;
  Eigen::Vector3d joint_axis_parent;
  Eigen::Vector3d translation_from_parent;
};


class KickNode : public rclcpp::Node
{
public:
  using Kick = aurea_kick::action::Kick;
  using GoalHandleKick = rclcpp_action::ServerGoalHandle<Kick>;
  using SolveIK = aurea_walk::srv::SolveIK;

  KickNode();

private:
  // Suas funções de action handle existentes
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Kick::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleKick> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleKick> goal_handle);
  void execute(const std::shared_ptr<GoalHandleKick> goal_handle);
  
  // --- NOVAS FUNÇÕES PARA O COMPENSADOR DE GRAVIDADE ---
  void initialize_robot_model();
  void run_forward_kinematics(const std::map<std::string, double>& joint_angles, const std::string& base_link_name, const Eigen::Affine3d& base_link_pose);
  void calculate_downstream_properties(const std::string& current_link_name, double& total_mass, Eigen::Vector3d& combined_com_moment_world);
  std::map<std::string, double> calculate_gravity_compensation(const std::map<std::string, double>& base_joint_angles, bool is_left_support);

  // --- ESTRUTURAS DE DADOS DO MODELO DO ROBÔ ---
  std::map<std::string, LinkData> robot_model_;
  std::map<std::string, std::string> joint_to_link_map_;
  std::map<std::string, Eigen::Affine3d> link_poses_world_;

  // Parâmetros de compensação
  double kp_gain_hip_roll_;
  double kp_gain_hip_pitch_;
  double kp_gain_knee_;

  // Seus membros de classe existentes
  rclcpp_action::Server<Kick>::SharedPtr action_server_;
  rclcpp::Client<SolveIK>::SharedPtr ik_client_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<aurea_kick::KickEngine> kick_engine_;
};

#endif // AUREA_KICK_KICK_NODE_HPP_