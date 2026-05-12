#!/usr/bin/env python3
#coding=utf-8
'''
Nó "Cérebro" do Robô.
- Ouve os dados interpretados dos sensores.
- Usa a StateMachine para decidir o estado global do robô.
- Publica o estado para que as rotinas possam agir.
'''

import rclpy
from rclpy.node import Node
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg
from std_msgs.msg import String as StringMsg # Renomeado para evitar conflito com 'str'
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# Importa a classe de lógica
from .state_machine import StateMachine

class StateMachineReceiver(Node):
    def __init__(self):
        super().__init__('state_machine_receiver')
        
        # Instancia a classe de lógica
        self.state_machine = StateMachine()

        # --- Parâmetros ---
        self.declare_parameter('spin_search_speed', 0.5)
        self.declare_parameter('walk_forward_speed', 0.1)
        self.declare_parameter('kp_body_align', 0.8)

        self.spin_speed = self.get_parameter('spin_search_speed').get_parameter_value().double_value
        self.walk_speed = self.get_parameter('walk_forward_speed').get_parameter_value().double_value
        self.kp_align = self.get_parameter('kp_body_align').get_parameter_value().double_value

        # --- Variáveis de Sensores (para guardar os últimos dados recebidos) ---
        self.ball_found = False
        self.fall_state = 'Up'
        self.head_pan_angle = 0.0

        # --- Subscribers ---
        # Ouve os dados consolidados do ROSPacker
        self.create_subscription(StateMachineMsg, 'sensor_observer/state_machine_vars', self.sensor_data_callback, 10)
        # Ouve a posição atual da cabeça para o alinhamento do corpo
        self.create_subscription(JointState, '/goal_joint_states', self.head_feedback_callback, 10)

        # --- Publishers ---
        # Publica o estado global para as rotinas
        self.state_publisher = self.create_publisher(CurrentStateMsg, '/transitions_and_states/state_machine', 10)
        # Comanda o especialista da cabeça
        self.head_control_pub = self.create_publisher(StringMsg, '/head_control/state', 10)
        # Comanda o motor de caminhada
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Timer Principal (O "coração" do cérebro) ---
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz
        self.get_logger().info("StateMachineReceiver (Cérebro) iniciado e operacional.")

    def sensor_data_callback(self, msg: StateMachineMsg):
        """Atualiza o estado dos sensores com base nos dados do ROSPacker."""
        self.ball_found = msg.ball_found
        self.fall_state = msg.fall_state

    def head_feedback_callback(self, msg: JointState):
        """Armazena o ângulo de pan da cabeça para o alinhamento."""
        try:
            index = msg.name.index('head_pan')
            self.head_pan_angle = msg.position[index]
        except (ValueError, IndexError):
            pass

    def control_loop(self):
        """O ciclo principal de Percepção -> Decisão -> Ação."""
        # 1. Decisão: Roda a StateMachine para obter a string do estado atual
        current_state_str = self.state_machine.request_state_machine_update(
            ball_found=self.ball_found,
            fall_state=self.fall_state
        )

        # Publica o estado atual para depuração e para as rotinas antigas (se houver)
        state_msg = CurrentStateMsg()
        state_msg.current_state = current_state_str.lower()
        self.state_publisher.publish(state_msg)
        
        # 2. Ação: Envia os comandos corretos com base no estado
        head_command = StringMsg()
        twist_command = Twist()

        if current_state_str == 'SEARCHING':
            # Comanda a cabeça para procurar e o corpo para girar
            head_command.data = 'SEARCHING'
            twist_command.angular.z = self.spin_speed
            self.get_logger().info("Ação: Busca Ativa (girando corpo e cabeça)", throttle_duration_sec=1)

        elif current_state_str == 'WALKING':
            # Comanda a cabeça para rastrear e o corpo para andar e alinhar
            head_command.data = 'TRACKING'
            twist_command.linear.x = self.walk_speed
            twist_command.angular.z = self.kp_align * self.head_pan_angle # <-- Alinhamento do corpo!
            self.get_logger().info("Ação: Andando em direção à bola e alinhando", throttle_duration_sec=1)

        elif current_state_str == 'LOST_BALL_WALK':
            # Comanda a cabeça para rastrear (onde a bola estava) e o corpo para continuar reto
            head_command.data = 'TRACKING'
            twist_command.linear.x = self.walk_speed # Continua andando reto
            self.get_logger().warn("Ação: Bola perdida, andando reto por 2s...", throttle_duration_sec=1)
        
        elif current_state_str == 'GETTING_UP':
            # Para todos os movimentos enquanto o robô se levanta
            head_command.data = 'IDLE'
            # A rotina de "levantar" deve ser acionada em outro lugar

        # Envia os comandos para os especialistas
        self.head_control_pub.publish(head_command)
        self.cmd_vel_pub.publish(twist_command)

def main(args=None):
    rclpy.init(args=args)
    receiver = StateMachineReceiver()
    try:
        rclpy.spin(receiver)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()