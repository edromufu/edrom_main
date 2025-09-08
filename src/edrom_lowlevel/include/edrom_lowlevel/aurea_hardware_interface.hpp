#ifndef AUREA_HARDWARE_INTERFACE_HPP_
#define AUREA_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Endereços da Tabela de Controle (Control Table)
#define ADDR_MX_TORQUE_ENABLE           64
#define ADDR_MX_GOAL_POSITION           116
#define ADDR_MX_PRESENT_POSITION        132

#define ADDR_AX_TORQUE_ENABLE           24
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36

namespace edrom_lowlevel
{
class AureaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AureaHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Funções auxiliares para conversão
  int rad_to_value(double rad, float protocol);
  double value_to_rad(int value, float protocol);

  // Instâncias do Dynamixel SDK
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandlerV1_;
  dynamixel::PacketHandler *packetHandlerV2_;

  // Configurações do hardware
  std::string usb_port_;
  int baud_rate_;

  // Vetores para estado e comandos
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  
  // Mapeamento para informações de cada motor
  std::vector<uint8_t> motor_ids_;
  std::vector<float> motor_protocols_;
  std::vector<bool> motor_inversions_;
};
}

#endif // AUREA_HARDWARE_INTERFACE_HPP_