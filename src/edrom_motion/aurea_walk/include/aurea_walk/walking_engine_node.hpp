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

class WalkingEngineNode : public rclcpp::Node
{
public:
  enum WalkingState {
    IDLE,
    WALKING,
    IDLE_MARCH,
    STOPPING
  };

  WalkingEngineNode();

private:
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