#ifndef AUREA_WALK_WALKING_ENGINE_NODE_HPP_
#define AUREA_WALK_WALKING_ENGINE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <string>
#include "std_msgs/msg/empty.hpp" // Adicione este include
#include "aurea_walk/srv/solve_ik.hpp" 
#include "aurea_walk/trajectory_generator.hpp"// ADICIONADO

using SolveIK = aurea_walk::srv::SolveIK; 
struct LinkData {
  std::string name;
  std::string parent_name;
  double mass;
  Eigen::Vector3d com_position_local; // Posição do CoM relativa à origem DO PRÓPRIO LINK
  Eigen::Vector3d joint_axis_parent;   // Eixo de rotação da junta no frame do PAI
  Eigen::Vector3d translation_from_parent; // Translação da origem do PAI para a origem DESTE link
};

struct Foot {
  bool is_left;
  Eigen::Vector2d position;
  double yaw;
};

class WalkingEngineNode : public rclcpp::Node
{
public:
  enum WalkingState {
    IDLE,
    WALKING,
    IDLE_MARCH,
    STOPPING,
    HOMING
  };

  WalkingEngineNode();

private:

  void initialize_robot_model();
  void run_forward_kinematics(
  const std::map<std::string, double>& joint_angles,
  const std::string& base_link_name,
  const Eigen::Affine3d& base_link_pose);
  void calculate_downstream_properties(
  const std::string& current_link_name,
  double& total_mass,
  Eigen::Vector3d& combined_com_world);
  std::map<std::string, double> calculate_gravity_compensation_for_support_leg(
  const std::map<std::string, double>& base_joint_angles, bool is_left_support);
  void homing_loop();
  // --- ESTRUTURAS DE DADOS DO MODELO DO ROBÔ ---
  std::map<std::string, LinkData> robot_model_;
  std::map<std::string, std::string> joint_to_link_map_;
  std::map<std::string, Eigen::Affine3d> link_poses_world_;
  // Parâmetros de controle e compensação
  double backlash_offset_hp_;
  //double servo_kp_gain_; // Ganho para converter torque (Nm) em offset de posição (rad)
  double kp_gain_hip_roll_;
  double kp_gain_hip_pitch_;
  double kp_gain_knee_;
   //std::map<std::string, double> last_filtered_gravity_offsets_;
  //double filter_alpha_; // Parâmetro do filtro de suavização


  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void main_loop();
  void ik_response_callback(rclcpp::Client<SolveIK>::SharedFuture future);
  void go_homing();
  void start_new_step();
  void start_homing_motion();
  void stop_command_callback(const std_msgs::msg::Empty::SharedPtr msg);
  // Parâmetros
  double T_, z_com_, z_step_, ds_ratio_, y_sep_;
  double arm_swing_amplitude_;
  double idle_shoulder_pitch_;
  double idle_shoulder_roll_;
  double idle_elbow_;
  double update_period_;

  double homing_duration_;
  double t_homing_{0.0};
  aurea_walk::PoseData torso_homing_start_;
  aurea_walk::PoseData left_foot_homing_start_;
  aurea_walk::PoseData right_foot_homing_start_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  bool stop_requested_ = false;
  std::mutex stop_mutex_;

  // Estado da caminhada
  geometry_msgs::msg::Twist v_cmd_;
  std::mutex cmd_mutex_;
  
  WalkingState current_state_{IDLE}; 
  double t_step_{0.0};

  aurea_walk::PoseData torso_, torso_start_, torso_target_;
  aurea_walk::PoseData left_foot_, right_foot_;
  aurea_walk::PoseData swing_start_, swing_target_;
  aurea_walk::PoseData left_foot_start_homing_, right_foot_start_homing_; 
  aurea_walk::PoseData * support_foot_;
  aurea_walk::PoseData * swing_foot_;

  // Estado para combinar resultados de IK
  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState combined_joint_state_;
  int ik_responses_received_{0};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Client<SolveIK>::SharedPtr ik_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // aurea_walk_WALKING_ENGINE_NODE_HPP_