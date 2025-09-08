#include "edrom_lowlevel/aurea_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace edrom_lowlevel
{

hardware_interface::CallbackReturn AureaHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  usb_port_ = info_.hardware_parameters["usb_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  motor_ids_.resize(info_.joints.size(), 0);
  motor_protocols_.resize(info_.joints.size(), 0.0);
  motor_inversions_.resize(info_.joints.size(), false);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    motor_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    motor_protocols_[i] = std::stof(info_.joints[i].parameters.at("protocol"));
    // O parâmetro 'inverted' é opcional, padrão para 'false'
    if (info_.joints[i].parameters.count("inverted")) {
        motor_inversions_[i] = (info_.joints[i].parameters.at("inverted") == "true");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("AureaHardwareInterface"), "Interface de Hardware inicializada.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AureaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AureaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn AureaHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
  packetHandlerV1_ = dynamixel::PacketHandler::getPacketHandler(1.0);
  packetHandlerV2_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (!portHandler_->openPort()) {
    RCLCPP_FATAL(rclcpp::get_logger("AureaHardwareInterface"), "Falha ao abrir a porta %s", usb_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!portHandler_->setBaudRate(baud_rate_)) {
    RCLCPP_FATAL(rclcpp::get_logger("AureaHardwareInterface"), "Falha ao definir o baud rate para %d", baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    dynamixel::PacketHandler * ph = (motor_protocols_[i] == 1.0) ? packetHandlerV1_ : packetHandlerV2_;
    uint16_t addr_torque = (motor_protocols_[i] == 1.0) ? ADDR_AX_TORQUE_ENABLE : ADDR_MX_TORQUE_ENABLE;
    ph->write1ByteTxRx(portHandler_, motor_ids_[i], addr_torque, 1, nullptr);
  }
  
  RCLCPP_INFO(rclcpp::get_logger("AureaHardwareInterface"), "Hardware ativado e torque habilitado.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AureaHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    dynamixel::PacketHandler * ph = (motor_protocols_[i] == 1.0) ? packetHandlerV1_ : packetHandlerV2_;
    uint16_t addr_torque = (motor_protocols_[i] == 1.0) ? ADDR_AX_TORQUE_ENABLE : ADDR_MX_TORQUE_ENABLE;
    ph->write1ByteTxRx(portHandler_, motor_ids_[i], addr_torque, 0, nullptr);
  }
  portHandler_->closePort();
  RCLCPP_INFO(rclcpp::get_logger("AureaHardwareInterface"), "Hardware desativado e porta fechada.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AureaHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    uint32_t current_pos_raw = 0;
    dynamixel::PacketHandler * ph = (motor_protocols_[i] == 1.0) ? packetHandlerV1_ : packetHandlerV2_;
    uint16_t addr_pos = (motor_protocols_[i] == 1.0) ? ADDR_AX_PRESENT_POSITION : ADDR_MX_PRESENT_POSITION;
    
    if (motor_protocols_[i] == 1.0) {
      uint16_t pos_16 = 0;
      ph->read2ByteTxRx(portHandler_, motor_ids_[i], addr_pos, &pos_16, nullptr);
      current_pos_raw = pos_16;
    } else {
      ph->read4ByteTxRx(portHandler_, motor_ids_[i], addr_pos, &current_pos_raw, nullptr);
    }
    
    hw_positions_[i] = value_to_rad(current_pos_raw, motor_protocols_[i]);
    if (motor_inversions_[i]) {
        hw_positions_[i] *= -1.0;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AureaHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // A escrita otimizada com SyncWrite seria ideal aqui, mas a escrita individual é mais simples de ilustrar.
  for (size_t i = 0; i < motor_ids_.size(); ++i) {
    dynamixel::PacketHandler * ph = (motor_protocols_[i] == 1.0) ? packetHandlerV1_ : packetHandlerV2_;
    uint16_t addr_goal = (motor_protocols_[i] == 1.0) ? ADDR_AX_GOAL_POSITION : ADDR_MX_GOAL_POSITION;
    
    double pos_cmd = hw_commands_[i];
    if (motor_inversions_[i]) {
        pos_cmd *= -1.0;
    }
    
    uint32_t goal_value = rad_to_value(pos_cmd, motor_protocols_[i]);
    
    if (motor_protocols_[i] == 1.0) {
      ph->write2ByteTxRx(portHandler_, motor_ids_[i], addr_goal, goal_value, nullptr);
    } else {
      ph->write4ByteTxRx(portHandler_, motor_ids_[i], addr_goal, goal_value, nullptr);
    }
  }
  return hardware_interface::return_type::OK;
}

int AureaHardwareInterface::rad_to_value(double rad, float protocol) {
    if (protocol == 2.0) { // MX-106
        return static_cast<int>((rad + M_PI) * (4095.0 / (2.0 * M_PI)));
    } else { // AX-12A
        return static_cast<int>((rad + 150.0 * M_PI / 180.0) * (1023.0 / (300.0 * M_PI / 180.0)));
    }
}

double AureaHardwareInterface::value_to_rad(int value, float protocol) {
    if (protocol == 2.0) { // MX-106
        return (double)value * (2.0 * M_PI / 4095.0) - M_PI;
    } else { // AX-12A
        return (double)value * (300.0 * M_PI / 180.0 / 1023.0) - 150.0 * M_PI / 180.0;
    }
}

}

PLUGINLIB_EXPORT_CLASS(edrom_lowlevel::AureaHardwareInterface, hardware_interface::SystemInterface)