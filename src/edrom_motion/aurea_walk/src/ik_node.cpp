#include "rclcpp/rclcpp.hpp"
#include "aurea_walk/ik_solver.hpp"
#include "aurea_walk/srv/solve_ik.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>

class IKServiceNode : public rclcpp::Node
{
public:
  IKServiceNode() : Node("ik_service_node")
  {
    ik_solver_ = std::make_shared<aurea_walk::IKSolver>();

    service_ = this->create_service<aurea_walk::srv::SolveIK>(
      "solve_ik",
      std::bind(&IKServiceNode::handle_solve_ik, this, std::placeholders::_1, std::placeholders::_2));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    RCLCPP_INFO(this->get_logger(), "Serviço de IK pronto para receber requisições em /solve_ik");
  }

private:
  void handle_solve_ik(
    const std::shared_ptr<aurea_walk::srv::SolveIK::Request> request,
    std::shared_ptr<aurea_walk::srv::SolveIK::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Requisição de IK recebida para a perna %s", request->leg_id.c_str());

    Eigen::Vector3d p_body = {0.0, 0.0, 0.21};
    Eigen::Quaterniond R_body = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p_foot_target = {
      request->target_pose.position.x, request->target_pose.position.y, request->target_pose.position.z
    };
    Eigen::Quaterniond R_foot_target(
      request->target_pose.orientation.w, request->target_pose.orientation.x,
      request->target_pose.orientation.y, request->target_pose.orientation.z
    );
    
    std::vector<double> calculated_angles;
    response->success = ik_solver_->solve(
      p_body, R_body, p_foot_target, R_foot_target, request->leg_id, calculated_angles);

    if (response->success) {
      const std::vector<std::string> all_joint_names = {
      "head_pan", "head_tilt", "l_sho_pitch", "l_sho_roll", "l_el",
      "r_sho_pitch", "r_sho_roll", "r_el", "r_hip_yaw", "r_hip_roll",
      "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll", "l_hip_yaw",
      "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll"
      };

     std::string prefix = (request->leg_id == "direita") ? "r_" : "l_";
  std::vector<std::string> leg_joint_names = {
    prefix + "hip_yaw", prefix + "hip_roll", prefix + "hip_pitch",
    prefix + "knee", prefix + "ank_pitch", prefix + "ank_roll"
  };
      std::map<std::string, double> calculated_leg_angles;
  for (size_t i = 0; i < leg_joint_names.size(); ++i) {
    calculated_leg_angles[leg_joint_names[i]] = calculated_angles[i];
  }

  // Cria a mensagem JointState que será enviada.
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  joint_state_msg->header.stamp = this->now();

  // Preenche a mensagem completa, na ordem correta.
  for (const auto& name : all_joint_names) {
    joint_state_msg->name.push_back(name);
    // Verifica se a junta atual é uma das que calculamos.
    if (calculated_leg_angles.count(name)) {
      // Se for, usa o valor calculado.
      joint_state_msg->position.push_back(calculated_leg_angles.at(name));
    } else {
      // Se não for, usa 0.0 como padrão.
      joint_state_msg->position.push_back(0.0);
    }
  }

  // Preenche a resposta do serviço (apenas com os 6 ângulos da perna, para clareza).
  response->result_joint_state.name = leg_joint_names;
  response->result_joint_state.position = calculated_angles;
  
  // Publica a mensagem COMPLETA para o RViz.
  joint_state_pub_->publish(std::move(joint_state_msg));

  RCLCPP_INFO(this->get_logger(), "Solução encontrada. Estado completo do robô foi publicado.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Não foi possível encontrar uma solução de IK.");
    }
  }

  std::shared_ptr<aurea_walk::IKSolver> ik_solver_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Service<aurea_walk::srv::SolveIK>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKServiceNode>());
  rclcpp::shutdown();
  return 0;
}