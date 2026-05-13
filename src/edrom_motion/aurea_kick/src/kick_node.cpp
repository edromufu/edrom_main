#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "aurea_walk/srv/solve_ik.hpp" 
#include "std_msgs/msg/bool.hpp" 
#include "aurea_kick/action/kick.hpp"
#include "aurea_kick/kick_engine.hpp"
#include <Eigen/Dense>
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;
const double g = 9.81;
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

  using LinkData = aurea_kick::LinkData;

  KickNode() : Node("kick_node")
  {
    auto params = aurea_kick::KickParameters();
    this->declare_parameter("phase_a_time", 1.0);
    this->declare_parameter("phase_b_time", 0.7);
    this->declare_parameter("phase_c_time", 0.1);
    this->declare_parameter("phase_d_time", 0.7);
    this->declare_parameter("phase_e_time", 1.0);
    this->declare_parameter("x_amplitude", 0.1);
    this->declare_parameter("z_height", 0.05);
    this->declare_parameter("com_height", 0.22);
    this->declare_parameter("feet_separation", 0.044);
    this->declare_parameter("torso_kick_offset_x", 0.0); 
    this->declare_parameter<double>("kp_gain_hip_roll", 1.0);
    this->declare_parameter<double>("kp_gain_hip_pitch", -10.0);
    this->declare_parameter<double>("kp_gain_knee", 10.0);
    this->declare_parameter<double>("backlash_hip_offset", -0.2);

    kp_gain_hip_roll_ = this->get_parameter("kp_gain_hip_roll").as_double();
    kp_gain_hip_pitch_ = this->get_parameter("kp_gain_hip_pitch").as_double();
    kp_gain_knee_ = this->get_parameter("kp_gain_knee").as_double();

    params.tA = this->get_parameter("phase_a_time").as_double();
    params.tB = this->get_parameter("phase_b_time").as_double();
    params.tC = this->get_parameter("phase_c_time").as_double();
    params.tD = this->get_parameter("phase_d_time").as_double();
    params.tE = this->get_parameter("phase_e_time").as_double();
    params.x_kick = this->get_parameter("x_amplitude").as_double();
    params.z_kick = this->get_parameter("z_height").as_double();
    params.com_height = this->get_parameter("com_height").as_double();
    params.y_sep = this->get_parameter("feet_separation").as_double();
    params.torso_kick_offset_x = this->get_parameter("torso_kick_offset_x").as_double();
    kick_engine_ = std::make_unique<aurea_kick::KickEngine>(params);
    backlash_hip_offset_ = this->get_parameter("backlash_hip_offset").as_double();
    initialize_robot_model(); 
    ik_client_ = this->create_client<SolveIK>("/solve_ik");
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/goal_joint_states", 10);
    kick_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/kick_done", 10);
    action_server_ = rclcpp_action::create_server<Kick>(
      this, "kick",
      std::bind(&KickNode::handle_goal, this, _1, _2),
      std::bind(&KickNode::handle_cancel, this, _1),
      std::bind(&KickNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Servidor de Ação de Chute pronto.");
  }

private:
  void initialize_robot_model();
  void run_forward_kinematics(
    const std::map<std::string, double>& joint_angles,
    const std::string& base_link_name,
    const Eigen::Affine3d& base_link_pose);
  void calculate_downstream_properties(
    const std::string& current_link_name,
    double& total_mass,
    Eigen::Vector3d& combined_com_moment_world);
  std::map<std::string, double> calculate_gravity_compensation_for_support_leg(
    const std::map<std::string, double>& base_joint_angles, bool is_left_support);

  std::map<std::string, LinkData> robot_model_;
  std::map<std::string, std::string> joint_to_link_map_;
  std::map<std::string, Eigen::Affine3d> link_poses_world_;
  
  double kp_gain_hip_roll_;
  double kp_gain_hip_pitch_;
  double kp_gain_knee_;

  double backlash_hip_offset_;
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

                // 1. Converte a resposta da IK para um mapa de ângulos (para a FK)
                std::map<std::string, double> ik_angles;
                for (size_t i = 0; i < support_res->result_joint_state.name.size(); ++i) {
                    ik_angles[support_res->result_joint_state.name[i]] = support_res->result_joint_state.position[i];
                }
                for (size_t i = 0; i < kick_res->result_joint_state.name.size(); ++i) {
                    ik_angles[kick_res->result_joint_state.name[i]] = kick_res->result_joint_state.position[i];
                }

                // 2. Calcula os offsets de gravidade para a perna de apoio
                bool is_left_support = (goal->leg_id == "direita");
                // Chamada de função CORRIGIDA
                auto gravity_offsets = calculate_gravity_compensation_for_support_leg(ik_angles, is_left_support);

                // 3. Monta a mensagem final de juntas a partir dos resultados da IK
                sensor_msgs::msg::JointState final_joints;
                final_joints.header.stamp = this->now();
                final_joints.name = support_res->result_joint_state.name;
                final_joints.name.insert(final_joints.name.end(), kick_res->result_joint_state.name.begin(), kick_res->result_joint_state.name.end());
                final_joints.position = support_res->result_joint_state.position;
                final_joints.position.insert(final_joints.position.end(), kick_res->result_joint_state.position.begin(), kick_res->result_joint_state.position.end());
                
                
                
                // --- FIM DA LÓGICA DO GRAVITY COMPENSATOR ---
          final_joints.header.stamp = this->now();
          final_joints.name.insert(final_joints.name.end(), support_res->result_joint_state.name.begin(), support_res->result_joint_state.name.end());
          final_joints.name.insert(final_joints.name.end(), kick_res->result_joint_state.name.begin(), kick_res->result_joint_state.name.end());
          final_joints.position.insert(final_joints.position.end(), support_res->result_joint_state.position.begin(), support_res->result_joint_state.position.end());
          final_joints.position.insert(final_joints.position.end(), kick_res->result_joint_state.position.begin(), kick_res->result_joint_state.position.end());
          for (size_t i = 0; i < final_joints.name.size(); ++i) {
                    const std::string& joint_name = final_joints.name[i];

                    // 1. Aplica compensação de gravidade
                    if (gravity_offsets.count(joint_name)) {
                        final_joints.position[i] += gravity_offsets.at(joint_name);
                    }
                    
                    // ====================== LÓGICA DE ESTABILIDADE ADICIONADA ======================
                    // Aplica o offset de estabilidade estática nos motores do quadril (pitch)
                    if (joint_name == "l_hip_pitch" || joint_name == "r_hip_pitch") {
                        final_joints.position[i] += backlash_hip_offset_;
                    }
                    // =============================================================================
                }
          
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
      auto done_msg = std_msgs::msg::Bool();
      done_msg.data = result->success;
      if (result->success) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
      kick_done_pub_->publish(done_msg);
    }
  }

  rclcpp_action::Server<Kick>::SharedPtr action_server_;
  rclcpp::Client<SolveIK>::SharedPtr ik_client_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kick_done_pub_;
  std::unique_ptr<aurea_kick::KickEngine> kick_engine_;
};

void KickNode::initialize_robot_model() 
{ 
  RCLCPP_INFO(this->get_logger(), "Inicializando modelo do robô para compensação de gravidade a partir do URDF...");
    // NOTA: As massas e posições do CoM foram extraídas do seu URDF.

    // Link base (torso)
    robot_model_["base_link"] = {"base_link", "", 1.65, {-0.0054, -0.0013, 0.0714}, {0,0,0}, {0,0,0}};

    // --- Cabeça ---
    robot_model_["head_pan_link"]  = {"head_pan_link",  "base_link",        0.04, {0, 0.008, 0.042}, {0, 0, 1}, {0.012, 0.0007, 0.1444}};
    robot_model_["head_tilt_link"] = {"head_tilt_link", "head_pan_link",    0.066, {0.018, 0, 0.041}, {0, 1, 0}, {0, 0.0002, 0.061}};
    joint_to_link_map_["head_pan"] = "head_pan_link";
    joint_to_link_map_["head_tilt"] = "head_tilt_link";
    
    // --- Braço Esquerdo ---
    robot_model_["l_sho_pitch_link"] = {"l_sho_pitch_link", "base_link",         0.03, {0.003, 0.04, -0.009}, {0, 1, 0}, {0.0126, 0.0682, 0.1366}};
    robot_model_["l_sho_roll_link"]  = {"l_sho_roll_link",  "l_sho_pitch_link",  0.4, {0, 0.05, 0}, {1, 0, 0}, {0.0008, 0.05, -0.032}};
    robot_model_["l_el_link"]        = {"l_el_link",        "l_sho_roll_link",   0.075, {0, 0.08, 0.016}, {0, 0, 1}, {0.0007, 0.1012, 0}};
    joint_to_link_map_["l_sho_pitch"] = "l_sho_pitch_link";
    joint_to_link_map_["l_sho_roll"] = "l_sho_roll_link";
    joint_to_link_map_["l_el"] = "l_el_link";

    // --- Braço Direito ---
    robot_model_["r_sho_pitch_link"] = {"r_sho_pitch_link", "base_link",         0.03, {-0.003, -0.04, -0.009}, {0, 1, 0}, {0.0117, -0.0677, 0.1366}};
    robot_model_["r_sho_roll_link"]  = {"r_sho_roll_link",  "r_sho_pitch_link",  0.4, {0, -0.05, 0}, {1, 0, 0}, {0, -0.05, -0.032}};
    robot_model_["r_el_link"]        = {"r_el_link",        "r_sho_roll_link",   0.075, {0, -0.08, 0.016}, {0, 0, 1}, {-0.0007, -0.1012, 0}};
    joint_to_link_map_["r_sho_pitch"] = "r_sho_pitch_link";
    joint_to_link_map_["r_sho_roll"] = "r_sho_roll_link";
    joint_to_link_map_["r_el"] = "r_el_link";

    // --- Perna Direita ---
    robot_model_["r_hip_yaw_link"]   = {"r_hip_yaw_link",   "base_link",          0.0069, {0, 0, -0.036}, {0, 0, 1}, {0, -0.0425, 0}};
    robot_model_["r_hip_roll_link"]  = {"r_hip_roll_link",  "r_hip_yaw_link",     0.32, {0.0298, 0, -0.0152}, {-1, 0, 0}, {-0.054, -0.0005, -0.062}};
    robot_model_["r_hip_pitch_link"] = {"r_hip_pitch_link", "r_hip_roll_link",    0.193, {0, 0, -0.0855}, {0, 1, 0}, {0.054, 0.0005, 0}};
    robot_model_["r_knee_link"]      = {"r_knee_link",      "r_hip_pitch_link",   0.0371, {0, 0, -0.0419}, {0, 1, 0}, {0, -0.00043, -0.12}};
    robot_model_["r_ank_pitch_link"] = {"r_ank_pitch_link", "r_knee_link",        0.32, {-0.0241, 0, 0.0152}, {0, 1, 0}, {0, -0.0005, -0.085}};
    robot_model_["r_ank_roll_link"]  = {"r_ank_roll_link",  "r_ank_pitch_link",   0.0877, {0.054, -0.0118, -0.0351}, {1, 0, 0}, {-0.054, 0, 0}};
    joint_to_link_map_["r_hip_yaw"]   = "r_hip_yaw_link";
    joint_to_link_map_["r_hip_roll"]  = "r_hip_roll_link";
    joint_to_link_map_["r_hip_pitch"] = "r_hip_pitch_link";
    joint_to_link_map_["r_knee"]      = "r_knee_link";
    joint_to_link_map_["r_ank_pitch"] = "r_ank_pitch_link";
    joint_to_link_map_["r_ank_roll"]  = "r_ank_roll_link";

    // --- Perna Esquerda ---
    robot_model_["l_hip_yaw_link"]   = {"l_hip_yaw_link",   "base_link",          0.007, {0, 0, -0.036}, {0, 0, 1}, {0, 0.0425, 0}};
    robot_model_["l_hip_roll_link"]  = {"l_hip_roll_link",  "l_hip_yaw_link",     0.32, {0.0298, 0, -0.0152}, {-1, 0, 0}, {-0.054, -0.0005, -0.062}};
    robot_model_["l_hip_pitch_link"] = {"l_hip_pitch_link", "l_hip_roll_link",    0.193, {0, 0, -0.0855}, {0, 1, 0}, {0.054, 0.0005, 0}};
    robot_model_["l_knee_link"]      = {"l_knee_link",      "l_hip_pitch_link",   0.0371, {0, 0, -0.0419}, {0, 1, 0}, {0, -0.00043, -0.12}};

    robot_model_["l_ank_pitch_link"] = {"l_ank_pitch_link", "l_knee_link",        0.32, {-0.0241, 0, 0.0152}, {0, 1, 0}, {0, -0.0005, -0.085}};
    robot_model_["l_ank_roll_link"]  = {"l_ank_roll_link",  "l_ank_pitch_link",   0.0877, {0.054, 0.0113, -0.0351}, {1, 0, 0}, {-0.054, 0.0005, 0}};
    joint_to_link_map_["l_hip_yaw"]   = "l_hip_yaw_link";
    joint_to_link_map_["l_hip_roll"]  = "l_hip_roll_link";
    joint_to_link_map_["l_hip_pitch"] = "l_hip_pitch_link";
    joint_to_link_map_["l_knee"]      = "l_knee_link";
    joint_to_link_map_["l_ank_pitch"] = "l_ank_pitch_link";
    joint_to_link_map_["l_ank_roll"]  = "l_ank_roll_link";

}
void KickNode::run_forward_kinematics(const std::map<std::string, double>& joint_angles,
  const std::string& base_link_name,
  const Eigen::Affine3d& base_link_pose) 
{ 
  link_poses_world_.clear();
    link_poses_world_[base_link_name] = base_link_pose;

    // Constrói um mapa de filhos para cada pai para facilitar a travessia
    std::map<std::string, std::vector<std::string>> parent_to_children_map;
    for (const auto& pair : robot_model_) {
        if (!pair.second.parent_name.empty()) {
            parent_to_children_map[pair.second.parent_name].push_back(pair.first);
        }
    }

    std::vector<std::string> unprocessed_links;
    for(const auto& pair : robot_model_){
        if (pair.first != base_link_name) {
            unprocessed_links.push_back(pair.first);
        }
    }

    // Itera até que todos os links sejam processados ou não haja mais progresso
    int passes = 0;
    while (!unprocessed_links.empty() && passes < robot_model_.size()) {
        for (auto it = unprocessed_links.begin(); it != unprocessed_links.end(); ) {
            std::string current_link_name = *it;
            const auto& link_info = robot_model_.at(current_link_name);
            bool processed = false;

            // Tenta processar "para baixo" (do pai para o filho)
            if (link_poses_world_.count(link_info.parent_name)) {
                auto joint_it = std::find_if(joint_to_link_map_.begin(), joint_to_link_map_.end(), 
                                            [&](const auto& p){ return p.second == link_info.name; });
                
                double angle = 0.0;
                if (joint_it != joint_to_link_map_.end() && joint_angles.count(joint_it->first)) {
                    angle = joint_angles.at(joint_it->first);
                }

                Eigen::Affine3d parent_pose = link_poses_world_.at(link_info.parent_name);
                Eigen::Affine3d transform = Eigen::Affine3d::Identity();
                transform.translate(link_info.translation_from_parent);
                transform.rotate(Eigen::AngleAxisd(angle, link_info.joint_axis_parent));
                
                link_poses_world_[current_link_name] = parent_pose * transform;
                processed = true;
            }
            // Tenta processar "para cima" (do filho para o pai)
            else if (parent_to_children_map.count(current_link_name) > 0) {
                 for (const std::string& child_name : parent_to_children_map.at(current_link_name)) {
                    if (link_poses_world_.count(child_name)) {
                        const auto& child_info = robot_model_.at(child_name);
                        auto joint_it = std::find_if(joint_to_link_map_.begin(), joint_to_link_map_.end(), 
                                                    [&](const auto& p){ return p.second == child_name; });

                        double angle = 0.0;
                        if (joint_it != joint_to_link_map_.end() && joint_angles.count(joint_it->first)) {
                            angle = joint_angles.at(joint_it->first);
                        }

                        Eigen::Affine3d transform_child_to_parent = Eigen::Affine3d::Identity();
                        transform_child_to_parent.translate(child_info.translation_from_parent);
                        transform_child_to_parent.rotate(Eigen::AngleAxisd(angle, child_info.joint_axis_parent));

                        link_poses_world_[current_link_name] = link_poses_world_.at(child_name) * transform_child_to_parent.inverse();
                        processed = true;
                        break; // Encontrou um filho processado, pode calcular o pai
                    }
                }
            }

            if (processed) {
                it = unprocessed_links.erase(it);
            } else {
                ++it;
            }
        }
        passes++;
    }

    if(!unprocessed_links.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Erro de FK: Loop ou parente faltando no modelo do robô. Links não processados:");
        for(const auto& name : unprocessed_links){
            RCLCPP_ERROR(this->get_logger(), "- %s (pai: %s)", name.c_str(), robot_model_.at(name).parent_name.c_str());
        }
    }
}
void KickNode::calculate_downstream_properties(const std::string& current_link_name,
  double& total_mass,
  Eigen::Vector3d& combined_com_moment_world)
{   
  if (robot_model_.count(current_link_name) == 0 || link_poses_world_.count(current_link_name) == 0) {
        total_mass = 0.0;
        combined_com_moment_world = Eigen::Vector3d::Zero();
        return;
    }

    const auto& link_info = robot_model_.at(current_link_name);
    total_mass = link_info.mass;
    combined_com_moment_world = (link_poses_world_.at(current_link_name) * link_info.com_position_local) * link_info.mass;

    for (const auto& pair : robot_model_) {
        if (pair.second.parent_name == current_link_name) {
            double child_mass;
            Eigen::Vector3d child_com_moment;
            calculate_downstream_properties(pair.first, child_mass, child_com_moment);
            total_mass += child_mass;
            combined_com_moment_world += child_com_moment;
        }
    }
}
std::map<std::string, double> KickNode::calculate_gravity_compensation_for_support_leg(const std::map<std::string, double>& base_joint_angles, bool is_left_support) 
{
  std::map<std::string, double> gravity_offsets;
    
    std::string support_foot_link_name = is_left_support ? "l_ank_roll_link" : "r_ank_roll_link";
    run_forward_kinematics(base_joint_angles, support_foot_link_name, Eigen::Affine3d::Identity());

    std::vector<std::string> joints_to_compensate;
    if (is_left_support) {
        joints_to_compensate = {"l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll"};
    } else {
        joints_to_compensate = {"r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll"};
    }
    
    Eigen::Vector3d gravity_vector(0, 0, -g);

    for (const auto& joint_name : joints_to_compensate) {
        if (joint_to_link_map_.count(joint_name) == 0) continue;

        std::string link_name = joint_to_link_map_.at(joint_name);
        const auto& link_info = robot_model_.at(link_name);

        double downstream_mass = 0.0;
        Eigen::Vector3d downstream_com_moment_world(0,0,0);
        
        // A função auxiliar agora retorna o momento de massa total
        calculate_downstream_properties(link_name, downstream_mass, downstream_com_moment_world);

        if(downstream_mass < 1e-6) continue;

        // A DIVISÃO ACONTECE AQUI, UMA ÚNICA VEZ
        Eigen::Vector3d downstream_com_world = downstream_com_moment_world / downstream_mass;

        Eigen::Vector3d joint_position_world = link_poses_world_.at(link_info.parent_name) * link_info.translation_from_parent;
        Eigen::Vector3d joint_axis_world = link_poses_world_.at(link_info.parent_name).rotation() * link_info.joint_axis_parent;
        
        Eigen::Vector3d vector_to_com = downstream_com_world - joint_position_world;
        Eigen::Vector3d gravity_force = downstream_mass * gravity_vector;
        
        Eigen::Vector3d torque_vector = vector_to_com.cross(gravity_force);
        double compensating_torque = -torque_vector.dot(joint_axis_world);
        double current_kp_gain = 0.0; // Valor padrão
        if (joint_name.find("hip_roll") != std::string::npos) {
          current_kp_gain = kp_gain_hip_roll_;
        } else if (joint_name.find("hip_pitch") != std::string::npos) {
          current_kp_gain = kp_gain_hip_pitch_;
        } else if (joint_name.find("knee") != std::string::npos) {
          current_kp_gain = kp_gain_knee_;
        } else if(joint_name.find("ank_roll") != std::string::npos){
          current_kp_gain = 1500000.0;
        }else if(joint_name.find("ank_pitch") != std::string::npos){
          current_kp_gain = 1500000.0;
        }
        gravity_offsets[joint_name] = 0.0;
        //gravity_offsets[joint_name] = compensating_torque / current_kp_gain;
        //gravity_offsets[joint_name] = compensating_torque / servo_kp_gain_;
    }
    
    return gravity_offsets;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KickNode>());
  rclcpp::shutdown();
  return 0;
}