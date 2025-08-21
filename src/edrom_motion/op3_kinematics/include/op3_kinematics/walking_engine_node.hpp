#ifndef OP3_KINEMATICS_WALKING_ENGINE_NODE_HPP_
#define OP3_KINEMATICS_WALKING_ENGINE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <string>

#include "op3_kinematics/srv/solve_ik.hpp" 
#include "op3_kinematics/trajectory_generator.hpp"

using SolveIK = op3_kinematics::srv::SolveIK;

class WalkingEngineNode : public rclcpp::Node
{
public:
  enum WalkingState {
    IDLE,
    WALKING,
    IDLE_MARCH
  };

  WalkingEngineNode();

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void main_loop();
  void ik_response_callback(rclcpp::Client<SolveIK>::SharedFuture future);
  
  void start_new_step();
  void start_homing_motion();

  // Parâmetros
  double T_, z_com_, z_step_, ds_ratio_, y_sep_;
  double update_period_;

  // Estado da caminhada
  geometry_msgs::msg::Twist v_cmd_;
  std::mutex cmd_mutex_;
  
  WalkingState current_state_{IDLE}; 
  double t_step_{0.0};

  op3_kinematics::PoseData torso_, torso_start_, torso_target_;
  op3_kinematics::PoseData left_foot_, right_foot_;
  op3_kinematics::PoseData swing_start_, swing_target_;
  op3_kinematics::PoseData left_foot_start_homing_, right_foot_start_homing_; 
  op3_kinematics::PoseData * support_foot_;
  op3_kinematics::PoseData * swing_foot_;

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

#endif  // OP3_KINEMATICS_WALKING_ENGINE_NODE_HPP_