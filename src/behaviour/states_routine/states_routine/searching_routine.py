#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from modularized_bhv_msgs.msg import CurrentStateMsg
from std_msgs.msg import String # Para comandar o HeadControllerNode
from geometry_msgs.msg import Twist

class SearchingRoutine(Node):
    """
    Nó Gerente da Rotina de Busca.
    Quando ativado pelo estado 'searching', comanda o corpo para girar e 
    o nó especialista da cabeça para iniciar seu padrão de busca.
    """
    def __init__(self):
        super().__init__('searching_routine_node')

        self.is_active = False

        # Parâmetro para a velocidade de giro do corpo
        self.declare_parameter('body_spin_speed_rad_s', 0.4)
        self.body_spin_speed = self.get_parameter('body_spin_speed_rad_s').get_parameter_value().double_value
        
        self.timer_period = 0.05  # 20 Hz

        # --- Subscriber ---
        # Ouve as ordens da StateMachine principal
        self.state_sub = self.create_subscription(
            CurrentStateMsg, 
            '/transitions_and_states/state_machine', 
            self.state_callback, 
            10)

        # --- Publishers ---
        # Publisher para comandar o especialista da cabeça
        self.head_control_pub = self.create_publisher(String, '/head_control/state', 10)
        # Publisher para comandar o corpo
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Timer ---
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info("Rotina de Busca ('searching_routine') pronta e ouvindo ordens.")

    def state_callback(self, msg):
        """Ativa ou desativa a rotina."""
        if msg.current_state == 'searching':
            if not self.is_active:
                self.get_logger().info("Ordem 'searching' recebida! Ativando rotina de busca.")
            self.is_active = True
        else:
            if self.is_active:
                self.get_logger().info(f"Ordem '{msg.current_state}' recebida. Desativando rotina de busca.")
            self.is_active = False

    def control_loop(self):
        """Se a rotina estiver ativa, envia os comandos para os especialistas."""
        if not self.is_active:
            # Se a rotina não está ativa, não fazemos nada. Outra rotina (ex: walking) está no controle.
            return

        # --- Comandos para os especialistas ---

        # 1. Ordem para o especialista da cabeça: "Execute a busca"
        head_command = String()
        head_command.data = "SEARCHING"
        self.head_control_pub.publish(head_command)

        # 2. Ordem para o motor de caminhada: "Gire no lugar"
        twist_command = Twist()
        twist_command.angular.z = self.body_spin_speed
        self.cmd_vel_pub.publish(twist_command)

def main(args=None):
    rclpy.init(args=args)
    searching_routine = SearchingRoutine()
    try:
        rclpy.spin(searching_routine)
    except KeyboardInterrupt:
        pass
    finally:
        searching_routine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()