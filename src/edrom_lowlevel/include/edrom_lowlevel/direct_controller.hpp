#ifndef DIRECT_CONTROLLER_HPP_
#define DIRECT_CONTROLLER_HPP_

#include <map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Endereços da Tabela de Controle (Control Table)
#define ADDR_MX_TORQUE_ENABLE           64
#define ADDR_MX_GOAL_POSITION           116
#define LEN_MX_GOAL_POSITION            4

#define ADDR_AX_TORQUE_ENABLE           24
#define ADDR_AX_GOAL_POSITION           30
#define LEN_AX_GOAL_POSITION            2

struct MotorConfig {
  uint8_t id;
  float protocol;
  bool inverted;
};

class DirectController : public rclcpp::Node
{
public:
  DirectController();
  ~DirectController();

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  int rad_to_value(double rad, float protocol);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::map<std::string, MotorConfig> motors_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandlerV1_;
  dynamixel::PacketHandler *packetHandlerV2_;
  
  dynamixel::GroupSyncWrite *groupSyncWriteV1_;
  dynamixel::GroupSyncWrite *groupSyncWriteV2_;
};

#endif // DIRECT_CONTROLLER_HPP_