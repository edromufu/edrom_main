#include "edrom_lowlevel/direct_controller.hpp"

// Construtor: Carrega parâmetros, inicializa os motores e cria o subscriber
DirectController::DirectController() : Node("direct_controller")
{
  this->declare_parameter<std::string>("usb_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 1000000);
  this->declare_parameter<std::vector<std::string>>("joint_names", {});

  auto joint_names = this->get_parameter("joint_names").as_string_array();

  for (const auto &name : joint_names) {
    this->declare_parameter<int>(name + ".id", 0);
    this->declare_parameter<double>(name + ".protocol", 2.0);
    this->declare_parameter<bool>(name + ".inverted", false);
    this->declare_parameter<double>(name + ".calibration_offset", 0.0);

    motors_[name] = {
      .id = (uint8_t)this->get_parameter(name + ".id").as_int(),
      .protocol = (float)this->get_parameter(name + ".protocol").as_double(),
      .inverted = this->get_parameter(name + ".inverted").as_bool(),
      .calibration_offset = this->get_parameter(name + ".calibration_offset").as_double()
    };
    RCLCPP_INFO(this->get_logger(), "Carregou motor '%s' com ID %d", name.c_str(), motors_[name].id);
  }

  std::string usb_port = this->get_parameter("usb_port").as_string();
  int baud_rate = this->get_parameter("baud_rate").as_int();

  portHandler_ = dynamixel::PortHandler::getPortHandler(usb_port.c_str());
  packetHandlerV1_ = dynamixel::PacketHandler::getPacketHandler(1.0);
  packetHandlerV2_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  groupSyncWriteV1_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandlerV1_, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION);
  groupSyncWriteV2_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandlerV2_, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
  groupSyncReadV2_ = new dynamixel::GroupSyncRead(portHandler_, packetHandlerV2_, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

  if (!portHandler_->openPort()) {
    RCLCPP_FATAL(this->get_logger(), "Falha ao abrir a porta %s", usb_port.c_str());
    rclcpp::shutdown();
    return;
  }
  if (!portHandler_->setBaudRate(baud_rate)) {
    RCLCPP_FATAL(this->get_logger(), "Falha ao definir o baud rate para %d", baud_rate);
    rclcpp::shutdown();
    return;
  }

  for (const auto &pair : motors_) {
    const auto &config = pair.second;
    uint16_t addr_torque = (config.protocol == 1.0) ? ADDR_AX_TORQUE_ENABLE : ADDR_MX_TORQUE_ENABLE;
    dynamixel::PacketHandler *handler = (config.protocol == 1.0) ? packetHandlerV1_ : packetHandlerV2_;
    handler->write1ByteTxRx(portHandler_, config.id, addr_torque, 1, nullptr);
  }
  RCLCPP_INFO(this->get_logger(), "Torque habilitado em todos os motores.");

  RCLCPP_INFO(this->get_logger(), "--- Verificando resposta individual dos motores (ping) ---");
  for (const auto &pair : motors_) {
    const auto &name = pair.first;
    const auto &config = pair.second;
    dynamixel::PacketHandler *handler = (config.protocol == 1.0) ? packetHandlerV1_ : packetHandlerV2_;
    uint8_t dxl_error = 0;
    int comm_result = handler->ping(portHandler_, config.id, &dxl_error);
    if (comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Motor '%s' (ID %d) NAO respondeu ao ping, excluido do feedback: %s",
        name.c_str(), config.id, handler->getTxRxResult(comm_result));
      continue;
    }

    if (dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "Motor '%s' (ID %d) respondeu com erro de hardware: %s",
        name.c_str(), config.id, handler->getRxPacketError(dxl_error));
    } else {
      RCLCPP_INFO(this->get_logger(), "Motor '%s' (ID %d) OK", name.c_str(), config.id);
    }

    // Motor respondeu (com ou sem erro de hardware), entao entra no feedback
    if (config.protocol == 2.0) {
      groupSyncReadV2_->addParam(config.id);
    }
  }

  subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/goal_joint_states", 10, std::bind(&DirectController::jointStateCallback, this, std::placeholders::_1));

  feedback_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/feedback", 10);
  feedback_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&DirectController::feedbackTimerCallback, this));
}

// Destrutor: Desliga o torque e fecha a porta serial por segurança
DirectController::~DirectController()
{
  RCLCPP_INFO(this->get_logger(), "Desligando controlador. Desabilitando torque dos motores.");
  for (const auto &pair : motors_) {
    const auto &config = pair.second;
    uint16_t addr_torque = (config.protocol == 1.0) ? ADDR_AX_TORQUE_ENABLE : ADDR_MX_TORQUE_ENABLE;
    dynamixel::PacketHandler *handler = (config.protocol == 1.0) ? packetHandlerV1_ : packetHandlerV2_;
    handler->write1ByteTxRx(portHandler_, config.id, addr_torque, 0, nullptr);
  }
  portHandler_->closePort();
}

// Callback: Chamado toda vez que uma mensagem chega
void DirectController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  groupSyncWriteV1_->clearParam();
  groupSyncWriteV2_->clearParam();
  
  bool param_added_v1 = false;
  bool param_added_v2 = false;
  RCLCPP_INFO(this->get_logger(), "--- Nova Mensagem Recebida ---");    
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string &joint_name = msg->name[i];
    auto it = motors_.find(joint_name);
    
    if (it == motors_.end()) continue;

    const auto &config = it->second;
    double target_pos = msg->position[i];

    if (joint_name == "r_hip_roll" || joint_name == "l_hip_roll") {
      RCLCPP_INFO(this->get_logger(), "Processando %s: Posição inicial = %.3f", joint_name.c_str(), target_pos);
    }

    if (config.inverted) {
      target_pos *= -1.0;
    }

    target_pos += config.calibration_offset;

    
    if (joint_name == "r_hip_roll" || joint_name == "l_hip_roll") {
      RCLCPP_INFO(this->get_logger(), "-> Posição FINAL para %s = %.3f (após offset de %.3f)", joint_name.c_str(), target_pos, config.calibration_offset);
    }

    int goal_value = rad_to_value(target_pos, config.protocol);

    uint8_t param_goal_position[4];
    if (config.protocol == 1.0) {
      param_goal_position[0] = DXL_LOBYTE(goal_value);
      param_goal_position[1] = DXL_HIBYTE(goal_value);
      if(groupSyncWriteV1_->addParam(config.id, param_goal_position)) param_added_v1 = true;
    } else {
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_value));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_value));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_value));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_value));
      if(groupSyncWriteV2_->addParam(config.id, param_goal_position)) param_added_v2 = true;
    }
  }
  
  if(param_added_v1) groupSyncWriteV1_->txPacket();
  if(param_added_v2) groupSyncWriteV2_->txPacket();
}

// Timer: Lê a posição atual dos motores MX-106 via GroupSyncRead e publica em /feedback
void DirectController::feedbackTimerCallback()
{
  int comm_result = groupSyncReadV2_->txRxPacket();
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "Falha no GroupSyncRead: %s", packetHandlerV2_->getTxRxResult(comm_result));
    return;
  }

  sensor_msgs::msg::JointState feedback_msg;
  feedback_msg.header.stamp = this->get_clock()->now();

  for (const auto &pair : motors_) {
    const auto &joint_name = pair.first;
    const auto &config = pair.second;

    if (config.protocol != 2.0) continue;

    if (!groupSyncReadV2_->isAvailable(config.id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)) {
      continue;
    }

    uint32_t raw_position = groupSyncReadV2_->getData(config.id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    double position = value_to_rad(raw_position, config.protocol);

    position -= config.calibration_offset;
    if (config.inverted) {
      position *= -1.0;
    }

    feedback_msg.name.push_back(joint_name);
    feedback_msg.position.push_back(position);
  }

  feedback_publisher_->publish(feedback_msg);
}

// Função auxiliar para conversão
int DirectController::rad_to_value(double rad, float protocol)
{
  if (protocol == 2.0) { // MX-106 (0 a 4095 para 360 graus)
    return static_cast<int>((rad + M_PI) * (4095.0 / (2.0 * M_PI)));
  } else { // AX-12A (0 a 1023 para 300 graus)
    return static_cast<int>((rad + 150.0 * M_PI / 180.0) * (1023.0 / (300.0 * M_PI / 180.0)));
  }
}

// Função auxiliar para conversão inversa (valor bruto do motor -> radianos)
double DirectController::value_to_rad(int value, float protocol)
{
  if (protocol == 2.0) { // MX-106 (0 a 4095 para 360 graus)
    return static_cast<double>(value) * (2.0 * M_PI / 4095.0) - M_PI;
  } else { // AX-12A (0 a 1023 para 300 graus)
    return static_cast<double>(value) * (300.0 * M_PI / 180.0 / 1023.0) - 150.0 * M_PI / 180.0;
  }
}

// Main: Inicia o nó
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectController>());
  rclcpp::shutdown();
  return 0;
}