#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <map>
#include <string>
#include <cmath> // Para M_PI

// ==============================================================================
// === ÁREA DE CONFIGURAÇÃO - AJUSTE ESTES VALORES ===
// ==============================================================================

// -- Configurações da Porta Serial --
const char* DEVICE_NAME = "/dev/ttyUSB0";
const int BAUDRATE = 1000000;

// -- Mapeamento de Nomes de Juntas para IDs dos Motores --
// !!! MUITO IMPORTANTE: Verifique se 19 e 20 são os IDs reais dos seus motores !!!
const std::map<std::string, int> MOTOR_MAP = {
    {"head_pan", 1},
    {"head_tilt", 2}
};

// -- Especificações dos Motores AX-12 --
const float PROTOCOL_VERSION = 1.0;
const uint16_t ADDR_PRESENT_POSITION = 36;

// ==============================================================================
// === NÓ ROS 2 ===
// ==============================================================================

class HeadReaderNode : public rclcpp::Node
{
public:
    HeadReaderNode() : Node("head_reader_node")
    {
        // Inicializa os handlers da SDK do Dynamixel
        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Abre a porta e define o baudrate
        if (!portHandler_->openPort()) {
            RCLCPP_FATAL(this->get_logger(), "Falha ao abrir a porta %s", DEVICE_NAME);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Porta serial aberta com sucesso.");

        if (!portHandler_->setBaudRate(BAUDRATE)) {
            RCLCPP_FATAL(this->get_logger(), "Falha ao definir o baudrate para %d", BAUDRATE);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Baudrate definido para %d.", BAUDRATE);

        // Cria o publisher para o tópico /head_feedback
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/head_feedback", 10);

        // Cria um timer que chama a função de leitura a cada 50ms (20Hz)
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(50ms, std::bind(&HeadReaderNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Nó de leitura da cabeça iniciado. Publicando em /head_feedback a 20Hz.");
    }

    ~HeadReaderNode()
    {
        // Garante que a porta serial seja fechada ao destruir o nó
        portHandler_->closePort();
        RCLCPP_INFO(this->get_logger(), "Porta serial fechada.");
    }

private:
    void timer_callback()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();

        for (const auto& pair : MOTOR_MAP) {
            std::string joint_name = pair.first;
            int motor_id = pair.second;
            
            uint16_t dxl_present_position = 0; // Para AX-12, a posição é de 2 bytes (uint16_t)
            uint8_t dxl_error = 0;
            int dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, motor_id, ADDR_PRESENT_POSITION, &dxl_present_position, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_WARN(this->get_logger(), "Falha de comunicação ao ler motor ID %d: %s", motor_id, packetHandler_->getTxRxResult(dxl_comm_result));
                continue;
            } else if (dxl_error != 0) {
                RCLCPP_WARN(this->get_logger(), "Erro de hardware no motor ID %d: %s", motor_id, packetHandler_->getRxPacketError(dxl_error));
                continue;
            }

            // Conversão de "ticks" (0-1023) para radianos para o AX-12
            // Ângulo (rad) = (Posição em Ticks - Posição Central) * (Range Angular em Rad / Range de Ticks)
            double angle_rad = (static_cast<double>(dxl_present_position) - 512.0) * (300.0 * M_PI / 180.0) / 1024.0;
            
            msg.name.push_back(joint_name);
            msg.position.push_back(angle_rad);
        }

        if (!msg.name.empty()) {
            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    dynamixel::PortHandler* portHandler_;
    dynamixel::PacketHandler* packetHandler_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadReaderNode>());
    rclcpp::shutdown();
    return 0;
}