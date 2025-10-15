#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

// Inclua o tipo do seu serviço de IK
#include "aurea_walk/srv/solve_ik.hpp" 

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// Função auxiliar para criar um quaternion de orientação neutra
geometry_msgs::msg::Quaternion create_neutral_orientation() {
    geometry_msgs::msg::Quaternion q;
    q.w = 1.0; q.x = 0.0; q.y = 0.0; q.z = 0.0;
    return q;
}

class RobotInitializer : public rclcpp::Node
{
public:
    using SolveIK = aurea_walk::srv::SolveIK; 

    RobotInitializer() : Node("robot_initializer")
    {
        this->declare_parameter("initial_com_height", 0.22);
        this->declare_parameter("initial_feet_separation", 0.045);

        ik_client_ptr_ = this->create_client<SolveIK>("/solve_ik");
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/goal_joint_states", 10);

        init_timer_ = this->create_wall_timer(
            1s, std::bind(&RobotInitializer::initialize_pose, this));

        RCLCPP_INFO(this->get_logger(), "Robot Initializer pronto. Executando em 1 segundo...");
    }

private: // <--- SEÇÃO QUE FALTAVA
    // Declaração das variáveis de membro
    rclcpp::Client<SolveIK>::SharedPtr ik_client_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    void initialize_pose()
    {
        init_timer_->cancel();

        if (!ik_client_ptr_->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "Serviço de IK '/solve_ik' não disponível. Desligando.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Requisitando IK para a pose inicial...");

        double com_height = this->get_parameter("initial_com_height").as_double();
        double feet_sep = this->get_parameter("initial_feet_separation").as_double();

        auto right_leg_req = std::make_shared<SolveIK::Request>();
        right_leg_req->leg_id = "direita";
        right_leg_req->target_pose.orientation = create_neutral_orientation();
        right_leg_req->target_pose.position.x = 0.0;
        right_leg_req->target_pose.position.y = -feet_sep;
        right_leg_req->target_pose.position.z = 0.0;

        auto left_leg_req = std::make_shared<SolveIK::Request>();
        left_leg_req->leg_id = "esquerda";
        left_leg_req->target_pose.orientation = create_neutral_orientation();
        left_leg_req->target_pose.position.x = 0.0;
        left_leg_req->target_pose.position.y = feet_sep;
        left_leg_req->target_pose.position.z = 0.0;

        auto right_future = ik_client_ptr_->async_send_request(right_leg_req);
        auto left_future = ik_client_ptr_->async_send_request(left_leg_req);
        
        std::thread([this, r_future = std::move(right_future), l_future = std::move(left_future)]() mutable {
            
            auto right_result = r_future.get();
            auto left_result = l_future.get();

            if (right_result && left_result && right_result->success && left_result->success) {
                RCLCPP_INFO(this->get_logger(), "IK resolvida com sucesso. Publicando pose e desligando.");
                
                auto final_joints = sensor_msgs::msg::JointState();
                final_joints.header.stamp = this->now();

                final_joints.name.insert(final_joints.name.end(), right_result->result_joint_state.name.begin(), right_result->result_joint_state.name.end());
                final_joints.position.insert(final_joints.position.end(), right_result->result_joint_state.position.begin(), right_result->result_joint_state.position.end());
                final_joints.name.insert(final_joints.name.end(), left_result->result_joint_state.name.begin(), left_result->result_joint_state.name.end());
                final_joints.position.insert(final_joints.position.end(), left_result->result_joint_state.position.begin(), left_result->result_joint_state.position.end());
                
                for (size_t i = 0; i < final_joints.name.size(); ++i) {
                if (final_joints.name[i] == "l_hip_pitch" || final_joints.name[i] == "r_hip_pitch") {
                    final_joints.position[i] -= 0.2;
                }
            }

                joint_pub_->publish(final_joints);
                std::this_thread::sleep_for(500ms);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Falha ao resolver IK para a pose inicial.");
            }
            rclcpp::shutdown();
        }).detach();
    }
}; // <--- Fechamento da classe

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotInitializer>());
  return 0;
}