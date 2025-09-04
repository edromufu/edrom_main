#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "aurea_walk/srv/solve_ik.hpp" // Seu serviço de IK
#include "aurea_kick/action/kick.hpp"
#include "aurea_kick/kick_engine.hpp"
#include <Eigen/Dense>
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals; // <-- ADICIONE ESTA LINHA

// Função auxiliar para converter Yaw para Quaternion
geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0); q.x = 0.0; q.y = 0.0; q.z = std::sin(yaw / 2.0);
  return q;
}

class KickNode : public rclcpp::Node
{
public:
  using Kick = aurea_kick::action::Kick;
  using GoalHandleKick = rclcpp_action::ServerGoalHandle<Kick>;
  using SolveIK = aurea_walk::srv::SolveIK;

  KickNode() : Node("kick_node")
  {
    auto params = aurea_kick::KickParameters();
    this->declare_parameter("phase_a_time", 1.2);
    this->declare_parameter("phase_b_time", 0.5);
    this->declare_parameter("phase_c_time", 0.25);
    this->declare_parameter("phase_d_time", 0.5);
    this->declare_parameter("phase_e_time", 1.2);
    this->declare_parameter("x_amplitude", 0.1);
    this->declare_parameter("z_height", 0.04);
    this->declare_parameter("com_height", 0.18);
    this->declare_parameter("feet_separation", 0.05);

    params.tA = this->get_parameter("phase_a_time").as_double();
    params.tB = this->get_parameter("phase_b_time").as_double();
    params.tC = this->get_parameter("phase_c_time").as_double();
    params.tD = this->get_parameter("phase_d_time").as_double();
    params.tE = this->get_parameter("phase_e_time").as_double();
    params.x_kick = this->get_parameter("x_amplitude").as_double();
    params.z_kick = this->get_parameter("z_height").as_double();
    params.com_height = this->get_parameter("com_height").as_double();
    params.y_sep = this->get_parameter("feet_separation").as_double();
    
    kick_engine_ = std::make_unique<aurea_kick::KickEngine>(params);

    ik_client_ = this->create_client<SolveIK>("/solve_ik");
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/goal_joint_states", 10);

    action_server_ = rclcpp_action::create_server<Kick>(
      this, "kick",
      std::bind(&KickNode::handle_goal, this, _1, _2),
      std::bind(&KickNode::handle_cancel, this, _1),
      std::bind(&KickNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Servidor de Ação de Chute pronto.");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Kick::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Recebido pedido de chute para a perna '%s'", goal->leg_id.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleKick> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Recebido pedido para cancelar o chute");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleKick> goal_handle)
  {
    std::thread{std::bind(&KickNode::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleKick> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executando o chute!");
    rclcpp::Rate loop_rate(100);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Kick::Feedback>();
    auto result = std::make_shared<Kick::Result>();
    
    kick_engine_->start(goal->leg_id == "esquerda", Eigen::Vector2d(0,0), 0);
    
    Eigen::Vector3d torso_pose_world, support_foot_pose_world, kick_foot_pose_world;
    
    while (rclcpp::ok() && kick_engine_->update(0.01, torso_pose_world, support_foot_pose_world, kick_foot_pose_world))
    {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
      
      // Converte poses do mundo para poses relativas ao torso para a IK
      double torso_yaw = 0.0; // O chute acontece sem girar o corpo
      Eigen::Rotation2Dd world_to_torso_rot(-torso_yaw);
      
      geometry_msgs::msg::Pose support_pose_req;
      Eigen::Vector2d support_in_torso = world_to_torso_rot * (support_foot_pose_world.head<2>() - torso_pose_world.head<2>());
      support_pose_req.position.x = support_in_torso.x();
      support_pose_req.position.y = support_in_torso.y();
      support_pose_req.position.z = 0.0;
      support_pose_req.orientation = yaw_to_quaternion(0);
      
      geometry_msgs::msg::Pose kick_pose_req;
      Eigen::Vector2d kick_in_torso = world_to_torso_rot * (kick_foot_pose_world.head<2>() - torso_pose_world.head<2>());
      kick_pose_req.position.x = kick_in_torso.x();
      kick_pose_req.position.y = kick_in_torso.y();
      kick_pose_req.position.z = kick_foot_pose_world.z();
      kick_pose_req.orientation = yaw_to_quaternion(0);

      // Chama o serviço de IK para cada perna
      auto support_req = std::make_shared<SolveIK::Request>();
      support_req->target_pose = support_pose_req;
      support_req->leg_id = (goal->leg_id == "esquerda") ? "direita" : "esquerda";
      
      auto kick_req = std::make_shared<SolveIK::Request>();
      kick_req->target_pose = kick_pose_req;
      kick_req->leg_id = goal->leg_id;
      
      auto support_future = ik_client_->async_send_request(support_req);
      auto kick_future = ik_client_->async_send_request(kick_req);
      
      if (support_future.wait_for(1s) == std::future_status::ready &&
          kick_future.wait_for(1s) == std::future_status::ready)
      {
        auto support_res = support_future.get();
        auto kick_res = kick_future.get();
        if (support_res->success && kick_res->success) {
          sensor_msgs::msg::JointState final_joints;
          final_joints.header.stamp = this->now();
          final_joints.name.insert(final_joints.name.end(), support_res->result_joint_state.name.begin(), support_res->result_joint_state.name.end());
          final_joints.name.insert(final_joints.name.end(), kick_res->result_joint_state.name.begin(), kick_res->result_joint_state.name.end());
          final_joints.position.insert(final_joints.position.end(), support_res->result_joint_state.position.begin(), support_res->result_joint_state.position.end());
          final_joints.position.insert(final_joints.position.end(), kick_res->result_joint_state.position.begin(), kick_res->result_joint_state.position.end());
          joint_pub_->publish(final_joints);
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Serviço de IK não respondeu a tempo.");
      }
      
      feedback->current_phase = kick_engine_->get_phase_name();
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
    
    if (rclcpp::ok()) {
      result->success = (kick_engine_->get_current_phase() == aurea_kick::KickEngine::Phase::DONE);
      if (result->success) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    }
  }

  rclcpp_action::Server<Kick>::SharedPtr action_server_;
  rclcpp::Client<SolveIK>::SharedPtr ik_client_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<aurea_kick::KickEngine> kick_engine_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KickNode>());
  rclcpp::shutdown();
  return 0;
}