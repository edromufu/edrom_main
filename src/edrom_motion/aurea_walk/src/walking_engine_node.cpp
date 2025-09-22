#include "aurea_walk/walking_engine_node.hpp"
#include <cmath> 

using namespace std::chrono_literals;
const double g = 9.81;
// Função auxiliar para converter Yaw (ângulo) para Quaternion
geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  return q;
}

WalkingEngineNode::WalkingEngineNode()
: Node("walking_engine_node")
{
  // Declara e carrega os parâmetros
  this->declare_parameter<double>("step_period", 1.2);
  this->declare_parameter<double>("com_height", 0.22);
  this->declare_parameter<double>("step_height", 0.033);
  this->declare_parameter<double>("double_support_ratio", 0.75);
  this->declare_parameter<double>("feet_separation", 0.045);
  this->declare_parameter<std::string>("ik_service_name", "/solve_ik");
  this->declare_parameter<std::string>("joint_command_topic", "/goal_joint_states");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<double>("update_frequency", 100.0);
  this->declare_parameter<double>("arm_swing_amplitude", 0.4);
  this->declare_parameter<double>("idle_arm_pose.shoulder_pitch", 0.7);
  this->declare_parameter<double>("idle_arm_pose.shoulder_roll", -1.4);
  this->declare_parameter<double>("idle_arm_pose.elbow", -1.6);
this->declare_parameter<double>("backlash_offset_hp", -0.18);
  //this->declare_parameter<double>("servo_kp_gain", 5.0); // Ganho para converter Nm em rad. Sintonize este valor!
  this->declare_parameter<double>("kp_gain_hip_roll", 6.0);
  this->declare_parameter<double>("kp_gain_hip_pitch", -1.5);
  this->declare_parameter<double>("kp_gain_knee", 5.0);
  //this->declare_parameter<double>("filter_alpha", 0.2);

  T_ = this->get_parameter("step_period").as_double();
  z_com_ = this->get_parameter("com_height").as_double();
  z_step_ = this->get_parameter("step_height").as_double();
  ds_ratio_ = this->get_parameter("double_support_ratio").as_double();
  y_sep_ = this->get_parameter("feet_separation").as_double();
  update_period_ = 1.0 / this->get_parameter("update_frequency").as_double();
  y_sep_ = this->get_parameter("feet_separation").as_double();
  update_period_ = 1.0 / this->get_parameter("update_frequency").as_double();
  arm_swing_amplitude_ = this->get_parameter("arm_swing_amplitude").as_double(); // <-- LEIA O NOVO PARÂMETRO
  idle_shoulder_pitch_ = this->get_parameter("idle_arm_pose.shoulder_pitch").as_double();
  idle_shoulder_roll_ = this->get_parameter("idle_arm_pose.shoulder_roll").as_double();
  idle_elbow_ = this->get_parameter("idle_arm_pose.elbow").as_double();
  stop_sub_ = this->create_subscription<std_msgs::msg::Empty>("/stop_walking", 10, std::bind(&WalkingEngineNode::stop_command_callback, this, std::placeholders::_1));
  backlash_offset_hp_ = this->get_parameter("backlash_offset_hp").as_double();
//  servo_kp_gain_ = this->get_parameter("servo_kp_gain").as_double();
  kp_gain_hip_pitch_ = this->get_parameter("kp_gain_hip_pitch").as_double();
  kp_gain_hip_roll_ = this->get_parameter("kp_gain_hip_roll").as_double();
  kp_gain_knee_ = this->get_parameter("kp_gain_knee").as_double();
  //filter_alpha_ = this->get_parameter("filter_alpha").as_double();
  initialize_robot_model();

  // Configuração inicial do estado
  left_foot_.is_left = true;
  right_foot_.is_left = false;
  left_foot_.position = {0.0, y_sep_};
  right_foot_.position = {0.0, -y_sep_};
  support_foot_ = &right_foot_;
  swing_foot_ = &left_foot_;

  // Cliente para o serviço de IK
  auto ik_service_name = this->get_parameter("ik_service_name").as_string();
  ik_client_ = this->create_client<SolveIK>(ik_service_name);
  while (!ik_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrompido enquanto aguardava o serviço de IK. Saindo.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Serviço de IK '%s' não disponível, aguardando...", ik_service_name.c_str());
  }


  // Publisher e Subscriber
  auto joint_cmd_topic = this->get_parameter("joint_command_topic").as_string();
  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_cmd_topic, 10);
  auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10, std::bind(&WalkingEngineNode::cmd_vel_callback, this, std::placeholders::_1));

  // Timer principal
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(update_period_),
    std::bind(&WalkingEngineNode::main_loop, this));
  
  RCLCPP_INFO(this->get_logger(), "Walking Engine (C++) com marcha no lugar inicializado.");
}

void WalkingEngineNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  v_cmd_ = *msg;
}

void WalkingEngineNode::start_new_step()
{
  t_step_ = 0.0;
  
  if (support_foot_->is_left) {
    support_foot_ = &right_foot_;
    swing_foot_ = &left_foot_;
  } else {
    support_foot_ = &left_foot_;
    swing_foot_ = &right_foot_;
  }
  
  geometry_msgs::msg::Twist command_to_use;
  // Se o estado for de caminhada normal, usa o comando recebido.
  // Se for de marcha no lugar, usa um comando zerado para não sair do lugar.
  if (current_state_ == WALKING) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    command_to_use = v_cmd_;
  } else { // IDLE_MARCH ou STOPPING
    command_to_use = geometry_msgs::msg::Twist();
  }

  aurea_walk::select_next_poses(
    torso_target_, swing_target_, torso_, *swing_foot_,
    command_to_use, T_, y_sep_);
  
  torso_start_ = torso_;
  swing_start_ = *swing_foot_;

  RCLCPP_INFO(this->get_logger(), "Iniciando novo passo. Estado: %d, Apoio: %s", current_state_, support_foot_->is_left ? "Esquerdo" : "Direito");
}

void WalkingEngineNode::initialize_robot_model()
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
    // LINHAS FALTANTES ADICIONADAS AQUI:
    robot_model_["l_ank_pitch_link"] = {"l_ank_pitch_link", "l_knee_link",        0.32, {-0.0241, 0, 0.0152}, {0, 1, 0}, {0, -0.0005, -0.085}};
    robot_model_["l_ank_roll_link"]  = {"l_ank_roll_link",  "l_ank_pitch_link",   0.0877, {0.054, 0.0113, -0.0351}, {1, 0, 0}, {-0.054, 0.0005, 0}};
    joint_to_link_map_["l_hip_yaw"]   = "l_hip_yaw_link";
    joint_to_link_map_["l_hip_roll"]  = "l_hip_roll_link";
    joint_to_link_map_["l_hip_pitch"] = "l_hip_pitch_link";
    joint_to_link_map_["l_knee"]      = "l_knee_link";
    joint_to_link_map_["l_ank_pitch"] = "l_ank_pitch_link";
    joint_to_link_map_["l_ank_roll"]  = "l_ank_roll_link";

    // Adicione os links dos braços e cabeça aqui também para um modelo completo
    // ...
}

void WalkingEngineNode::run_forward_kinematics(
  const std::map<std::string, double>& joint_angles,
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


void WalkingEngineNode::calculate_downstream_properties(
  const std::string& current_link_name,
  double& total_mass,
  Eigen::Vector3d& combined_com_moment_world) // Renomeado para maior clareza
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


// VERSÃO CORRIGIDA
// Esta função agora faz a divisão pela massa total
std::map<std::string, double> WalkingEngineNode::calculate_gravity_compensation_for_support_leg(
  const std::map<std::string, double>& base_joint_angles, bool is_left_support)
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
          current_kp_gain = 1500.0;
        }else if(joint_name.find("ank_pitch") != std::string::npos){
          current_kp_gain = 1500.0;
        }

        gravity_offsets[joint_name] = compensating_torque / current_kp_gain;
        //gravity_offsets[joint_name] = compensating_torque / servo_kp_gain_;
    }
    
    return gravity_offsets;
}

void WalkingEngineNode::stop_command_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg; // Evita warning de variável não utilizada
  std::lock_guard<std::mutex> lock(stop_mutex_);
  
  if (current_state_ == WALKING || current_state_ == IDLE_MARCH) {
    RCLCPP_INFO(this->get_logger(), "Comando de parada recebido. Finalizando o passo atual...");
    stop_requested_ = true;
  }
}

void WalkingEngineNode::main_loop()
{

  bool stop_now = false;
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    if (stop_requested_) {
      stop_now = true;
      stop_requested_ = false; // Reseta a flag
    }
  }

  geometry_msgs::msg::Twist current_cmd;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    current_cmd = v_cmd_;
  }
  bool should_walk = std::abs(current_cmd.linear.x) > 0.01 || 
                     std::abs(current_cmd.linear.y) > 0.01 || 
                     std::abs(current_cmd.angular.z) > 0.01;

  // Atualiza o estado da máquina de estados com base no comando
  WalkingState previous_state = current_state_;
  if (stop_now && (current_state_ == WALKING || current_state_ == IDLE_MARCH)) {
      current_state_ = STOPPING;
      RCLCPP_INFO(this->get_logger(), "Transição para o estado STOPPING.");
  }else if (current_state_ == IDLE) {
    if (should_walk) {
      current_state_ = WALKING;
    }
  } else if (current_state_ == WALKING || current_state_ == IDLE_MARCH) {
    if (should_walk) {
      current_state_ = WALKING;
    } else {
      current_state_ = IDLE_MARCH;
    }
  }

  // Se o estado mudou de IDLE para um estado ativo, inicia o primeiro passo
  if (previous_state == IDLE && (current_state_ == WALKING || current_state_ == IDLE_MARCH)) {
      start_new_step();
  }
  // Se não estiver fazendo nada (nem andando, nem marchando), sai do loop
  if (current_state_ == IDLE) {
    sensor_msgs::msg::JointState idle_arm_msg;
    idle_arm_msg.header.stamp = this->get_clock()->now();
    
    // Nomes das juntas dos braços (verifique se correspondem ao seu robô)
    idle_arm_msg.name = {
      "l_sho_pitch", "r_sho_pitch",
      "l_sho_roll", "r_sho_roll",
      "l_el", "r_el"
    };
    // Posições de repouso (simétricas) lidas dos parâmetros
    idle_arm_msg.position = {
      idle_shoulder_pitch_,  idle_shoulder_pitch_,
      idle_shoulder_roll_,  -idle_shoulder_roll_, // Roll é simétrico
      idle_elbow_,           -idle_elbow_
    };
    
    joint_pub_->publish(idle_arm_msg);
    return; // Sai do loop
  }

  // Prepara para receber novas respostas da IK
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    ik_responses_received_ = 0;
    combined_joint_state_.name.clear();
    combined_joint_state_.position.clear();
  }

  t_step_ += update_period_;

  { // Bloco de lógica da caminhada/marcha
    Eigen::Vector2d current_torso_pos_2d;
    double current_torso_yaw;
    const double g = 9.81;
    aurea_walk::get_com_pose_at_time(
        current_torso_pos_2d, current_torso_yaw, t_step_, T_, ds_ratio_, z_com_, g,
        torso_start_, torso_target_, *support_foot_);

    Eigen::Vector3d current_swing_pos_world;
    double current_swing_yaw_world;
    aurea_walk::get_swing_foot_pose_at_time(
        current_swing_pos_world, current_swing_yaw_world, t_step_, T_, z_step_, ds_ratio_,
        swing_start_, swing_target_);

    Eigen::Rotation2Dd world_to_torso_rot(-current_torso_yaw);
    
    Eigen::Vector2d support_pos_in_torso_2d = world_to_torso_rot * (support_foot_->position - current_torso_pos_2d);
    geometry_msgs::msg::Pose support_pose_msg;
    support_pose_msg.position.x = support_pos_in_torso_2d.x();
    support_pose_msg.position.y = support_pos_in_torso_2d.y();
    support_pose_msg.position.z = 0.0;
    support_pose_msg.orientation = yaw_to_quaternion(support_foot_->yaw - current_torso_yaw);

    Eigen::Vector2d swing_pos_in_torso_2d = world_to_torso_rot * (current_swing_pos_world.head<2>() - current_torso_pos_2d);
    geometry_msgs::msg::Pose swing_pose_msg;
    swing_pose_msg.position.x = swing_pos_in_torso_2d.x();
    swing_pose_msg.position.y = swing_pos_in_torso_2d.y();
    swing_pose_msg.position.z = current_swing_pos_world.z();
    swing_pose_msg.orientation = yaw_to_quaternion(current_swing_yaw_world - current_torso_yaw);

    auto req_support = std::make_shared<SolveIK::Request>();
    req_support->target_pose = support_pose_msg;
    req_support->leg_id = support_foot_->is_left ? "esquerda" : "direita";
    ik_client_->async_send_request(req_support, std::bind(&WalkingEngineNode::ik_response_callback, this, std::placeholders::_1));

    auto req_swing = std::make_shared<SolveIK::Request>();
    req_swing->target_pose = swing_pose_msg;
    req_swing->leg_id = swing_foot_->is_left ? "esquerda" : "direita";
    ik_client_->async_send_request(req_swing, std::bind(&WalkingEngineNode::ik_response_callback, this, std::placeholders::_1));
  }

  // Ao final do passo, inicia o próximo (seja andando ou marchando)
  if (t_step_ >= T_) {
    // Atualiza as posições finais do passo
    torso_ = torso_target_;
    swing_foot_->position = swing_target_.position;
    swing_foot_->yaw = swing_target_.yaw;
    
    // DECISÃO PÓS-PASSO
    if (current_state_ == STOPPING) {
      // Se estávamos parando, o último passo foi concluído. Agora vamos para IDLE.
      RCLCPP_INFO(this->get_logger(), "Parada concluída. Entrando em estado IDLE.");
      current_state_ = IDLE;
      // NÃO chamamos start_new_step() aqui, pois queremos parar.
    } else {
      // Se não, continuamos o ciclo normal de caminhada/marcha.
      start_new_step(); 
    }// Prepara o próximo passo para o ciclo contínuo
  }
}

void WalkingEngineNode::ik_response_callback(rclcpp::Client<SolveIK>::SharedFuture future)
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  
  try {
    auto response = future.get();
    if (response->success) {
      combined_joint_state_.name.insert(
        combined_joint_state_.name.end(),
        response->result_joint_state.name.begin(),
        response->result_joint_state.name.end());
      combined_joint_state_.position.insert(
        combined_joint_state_.position.end(),
        response->result_joint_state.position.begin(),
        response->result_joint_state.position.end());
      ik_responses_received_++;
    } else {
      RCLCPP_WARN(this->get_logger(), "IK para uma perna falhou, comando não será enviado.");
      ik_responses_received_ = -1;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exceção no callback do serviço de IK: %s", e.what());
    ik_responses_received_ = -1;
  }

  if (ik_responses_received_ >= 2) {
        std::map<std::string, double> ik_angles;
        for (size_t i = 0; i < combined_joint_state_.name.size(); ++i) {
            ik_angles[combined_joint_state_.name[i]] = combined_joint_state_.position[i];
        }
// --- INÍCIO DA LÓGICA DE INTERPOLAÇÃO LINEAR ---
        
        std::map<std::string, double> gravity_offsets;
        double ds_time = ds_ratio_ * T_ / 2.0;

        // Calcula os offsets para ambas as pernas como se estivessem em apoio
        auto offsets_left_support = calculate_gravity_compensation_for_support_leg(ik_angles, true);
        auto offsets_right_support = calculate_gravity_compensation_for_support_leg(ik_angles, false);

        if (t_step_ < ds_time) { // Primeira fase de apoio duplo
            auto& offsets_old = support_foot_->is_left ? offsets_right_support : offsets_left_support;
            auto& offsets_new = support_foot_->is_left ? offsets_left_support : offsets_right_support;

            // FATOR DE INTERPOLAÇÃO LINEAR (0.0 -> 1.0)
            double ratio = t_step_ / ds_time;
            
            for(auto const& [key, val] : offsets_old) {
                gravity_offsets[key] = (1.0 - ratio) * val;
            }
            for(auto const& [key, val] : offsets_new) {
                gravity_offsets[key] += ratio * val;
            }

        } else if (t_step_ > T_ - ds_time) { // Segunda fase de apoio duplo
            auto& offsets_current = support_foot_->is_left ? offsets_left_support : offsets_right_support;
            auto& offsets_next = support_foot_->is_left ? offsets_right_support : offsets_left_support;
            
            // FATOR DE INTERPOLAÇÃO LINEAR (0.0 -> 1.0)
            double ratio = (t_step_ - (T_ - ds_time)) / ds_time;

            for(auto const& [key, val] : offsets_current) {
                gravity_offsets[key] = (1.0 - ratio) * val;
            }
            for(auto const& [key, val] : offsets_next) {
                gravity_offsets[key] += ratio * val;
            }

        } else { // Apoio único
            // A compensação está em 100% na perna de apoio.
            gravity_offsets = support_foot_->is_left ? offsets_left_support : offsets_right_support;
        }

        // Aplica os offsets já interpolados e os estáticos
        std::map<std::string, double> final_angles = ik_angles;
        for (const auto & pair : gravity_offsets) {
            if (final_angles.count(pair.first)) {
                final_angles[pair.first] += pair.second;
            }
        }
        if (final_angles.count("r_hip_pitch")) final_angles["r_hip_pitch"] += backlash_offset_hp_;
        if (final_angles.count("l_hip_pitch")) final_angles["l_hip_pitch"] += backlash_offset_hp_;
        
        // --- FIM DA LÓGICA DE INTERPOLAÇÃO ---

      RCLCPP_INFO(this->get_logger(), "--- [DEPURAÇÃO] Detalhes da Compensação (t=%.2f) ---", t_step_);
      for (const auto& pair : gravity_offsets) { // Usa a variável correta
          std::string joint_name = pair.first;
          double raw_offset = pair.second;
          if (ik_angles.count(joint_name)) {
              RCLCPP_INFO(this->get_logger(),
                  "  -> Joint[%s]: IK=%.4f, RawOffset=%.4f, FINAL=%.4f",
                  joint_name.c_str(),
                  ik_angles.at(joint_name),
                  raw_offset,
                  final_angles.at(joint_name)
              );
          }
      }
      RCLCPP_INFO(this->get_logger(), "--- [DEPURAÇÃO] Fim do Relatório ---");

        combined_joint_state_.name.clear();
        combined_joint_state_.position.clear();
        for (const auto & pair : final_angles) {
            combined_joint_state_.name.push_back(pair.first);
            combined_joint_state_.position.push_back(pair.second);
        }

        // Adiciona balanço dos braços
        double phi = t_step_ / T_;
        double base_angle = arm_swing_amplitude_ * std::sin(M_PI * phi);
        double l_sho_pitch = (support_foot_->is_left) ? -base_angle : base_angle;
        double r_sho_pitch = (support_foot_->is_left) ? base_angle : -base_angle;
        
        combined_joint_state_.name.push_back("l_sho_pitch");
        combined_joint_state_.position.push_back(l_sho_pitch);
        combined_joint_state_.name.push_back("r_sho_pitch");
        combined_joint_state_.position.push_back(r_sho_pitch);
        
        combined_joint_state_.header.stamp = this->get_clock()->now();    

        std::string support_hip_roll = support_foot_->is_left ? "l_hip_roll" : "r_hip_roll";

        joint_pub_->publish(combined_joint_state_);
    
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkingEngineNode>());
  rclcpp::shutdown();
  return 0;
}