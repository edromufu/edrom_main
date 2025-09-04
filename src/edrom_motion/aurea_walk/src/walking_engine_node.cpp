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
  this->declare_parameter<double>("step_period", 0.8);
  this->declare_parameter<double>("com_height", 0.18);
  this->declare_parameter<double>("step_height", 0.02);
  this->declare_parameter<double>("double_support_ratio", 0.2);
  this->declare_parameter<double>("feet_separation", 0.05);
  this->declare_parameter<std::string>("ik_service_name", "/solve_ik");
  this->declare_parameter<std::string>("joint_command_topic", "/goal_joint_states");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<double>("update_frequency", 100.0);
  this->declare_parameter<double>("arm_swing_amplitude", 0.4);
  this->declare_parameter<double>("idle_arm_pose.shoulder_pitch", 0.7);
  this->declare_parameter<double>("idle_arm_pose.shoulder_roll", -1.4);
  this->declare_parameter<double>("idle_arm_pose.elbow", -1.6);
  
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
      double phi = t_step_ / T_; // Fase do passo atual (de 0.0 a 1.0)
    // Usamos sin(PI * phi) para criar um movimento suave de ida
      double base_angle = arm_swing_amplitude_ * std::sin(M_PI * phi);

      double l_sho_pitch = 0.0;
      double r_sho_pitch = 0.0;
    
    // O braço se move em oposição à perna de APOIO
      if (support_foot_->is_left) {
        l_sho_pitch = -base_angle;
        r_sho_pitch = base_angle;
      } else {
        l_sho_pitch = base_angle;
        r_sho_pitch = -base_angle;
      }
    
      combined_joint_state_.name.push_back("l_sho_pitch");
      combined_joint_state_.position.push_back(l_sho_pitch);
    
      combined_joint_state_.name.push_back("r_sho_pitch");
      combined_joint_state_.position.push_back(r_sho_pitch);
    
    

    //combined_joint_state_.name.push_back("l_el");
    //combined_joint_state_.position.push_back(std::abs(l_sho_pitch) * 0.5); // Dobra metade do ombro
    //combined_joint_state_.name.push_back("r_el");
    //combined_joint_state_.position.push_back(std::abs(r_sho_pitch) * 0.5); // Dobra metade do ombro

    // =================================================================
    
    combined_joint_state_.header.stamp = this->get_clock()->now();
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