#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"      // Para escutar o tópico de teleop
#include "aurea_kick/action/kick.hpp" // Para chamar a ação de chute

/*
 * Este nó atua como uma "ponte" entre um tópico simples de teleop e o
 * complexo servidor de Ação de chute. Ele escuta por uma String no tópico
 * /teleop/kick e chama a Ação /kick com o conteúdo da string como 'leg_id'.
 */
class TeleopKickBridgeNode : public rclcpp::Node
{
public:
    using KickAction = aurea_kick::action::Kick;
    using GoalHandleKick = rclcpp_action::ClientGoalHandle<KickAction>;

    TeleopKickBridgeNode() : Node("teleop_kick_bridge")
    {
        // 1. Cria o Cliente de Ação para o /kick
        this->client_ptr_ = rclcpp_action::create_client<KickAction>(this, "/kick");

        // 2. Cria o Subscriber para o tópico de teleop
        this->subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/teleop/kick", 10, std::bind(&TeleopKickBridgeNode::kick_command_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Ponte Teleop-Chute (teleop_kick_bridge) iniciada.");
        RCLCPP_INFO(this->get_logger(), "Escutando em /teleop/kick...");
    }

private:
    void kick_command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Bridge: Recebido comando de chute: '%s'", msg->data.c_str());

        // Verifica se o servidor de ação /kick está disponível
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Bridge: Servidor de Ação /kick não está disponível!");
            return;
        }

        auto goal_msg = KickAction::Goal();
        goal_msg.leg_id = msg->data;

        // Envia o goal de forma assíncrona ("fire and forget")
        // Não precisamos esperar pelo resultado aqui, apenas retransmitir o comando.
        client_ptr_->async_send_goal(goal_msg);
        
        RCLCPP_INFO(this->get_logger(), "Bridge: Goal de chute enviado para o /kick_node.");
    }

    rclcpp_action::Client<KickAction>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopKickBridgeNode>());
    rclcpp::shutdown();
    return 0;
}