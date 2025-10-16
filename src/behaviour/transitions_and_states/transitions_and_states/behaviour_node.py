#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
import time

# Mensagens para comunicação
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class StateMachine:
    """
    Máquina de estados simplificada e robusta para o comportamento de futebol.
    """
    def __init__(self):
        self.state = 'SEARCHING'
        self.LOST_BALL_TIMEOUT = 2.0  # Segundos de "paciência"
        self.time_ball_was_lost = None

    def update(self, ball_found, head_pan_angle):
        # --- Lógica de Transição de Estados ---

        # 1. Se a bola foi encontrada
        if ball_found:
            self.time_ball_was_lost = None # Reseta o timer de bola perdida
            
            # Se estava procurando, agora começa a andar
            if self.state == 'SEARCHING' or self.state == 'LOST_BALL_WALK':
                self.state = 'WALKING'
        
        # 2. Se a bola NÃO foi encontrada
        else:
            # Se estava andando e acabou de perder a bola
            if self.state == 'WALKING':
                self.state = 'LOST_BALL_WALK'
                self.time_ball_was_lost = time.time() # Inicia o cronômetro

            # Se está no estado de paciência (LOST_BALL_WALK)
            elif self.state == 'LOST_BALL_WALK':
                # Verifica se o tempo de paciência acabou
                if time.time() - self.time_ball_was_lost > self.LOST_BALL_TIMEOUT:
                    # Paciência esgotada, inicia a busca ativa
                    self.state = 'SEARCHING'
        
        return self.state

class BehaviorNode(Node):
    """
    O Cérebro do Robô.
    - Ouve os sensores interpretados.
    - Roda a StateMachine para decidir o que fazer.
    - Envia comandos para os especialistas (cabeça e motor de caminhada).
    """
    def __init__(self):
        super().__init__('behavior_node')
        self.state_machine = StateMachine()

        # --- Parâmetros ---
        self.declare_parameter('spin_search_speed', 0.5)
        self.declare_parameter('walk_forward_speed', 0.1)
        self.declare_parameter('kp_body_align', 0.8)

        self.spin_speed = self.get_parameter('spin_search_speed').get_parameter_value().double_value
        self.walk_speed = self.get_parameter('walk_forward_speed').get_parameter_value().double_value
        self.kp_align = self.get_parameter('kp_body_align').get_parameter_value().double_value

        # --- Variáveis de Sensores ---
        self.ball_found = False
        self.head_pan_angle = 0.0

        # --- Subscribers ---
        # Ouve os dados consolidados do ROSPacker
        self.create_subscription(StateMachineMsg, 'sensor_observer/state_machine_vars', self.sensor_data_callback, 10)
        # Ouve a posição atual da cabeça para o alinhamento do corpo
        self.create_subscription(JointState, '/goal_joint_states', self.head_feedback_callback, 10)

        # --- Publishers ---
        # Publica o estado global para outros nós (como o HUD ou depuração)
        self.state_publisher = self.create_publisher(CurrentStateMsg, '/transitions_and_states/state_machine', 10)
        # Comanda o especialista da cabeça
        self.head_control_pub = self.create_publisher(String, '/head_control/state', 10)
        # Comanda o motor de caminhada
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Timer Principal (O "coração" do cérebro) ---
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz
        self.get_logger().info("Cérebro do Robô (BehaviorNode) iniciado e operacional.")

    def sensor_data_callback(self, msg: StateMachineMsg):
        """Atualiza o estado dos sensores com base nos dados do ROSPacker."""
        self.ball_found = msg.ball_found

    def head_feedback_callback(self, msg: JointState):
        """Armazena o ângulo de pan da cabeça para o alinhamento."""
        try:
            index = msg.name.index('head_pan')
            self.head_pan_angle = msg.position[index]
        except (ValueError, IndexError):
            pass

    def control_loop(self):
        """O ciclo principal de Percepção -> Decisão -> Ação."""
        # 1. Decisão: Roda a StateMachine para obter o estado atual
        current_state = self.state_machine.update(self.ball_found, self.head_pan_angle)

        # Publica o estado atual para depuração
        state_msg = CurrentStateMsg()
        state_msg.current_state = current_state
        self.state_publisher.publish(state_msg)
        
        # 2. Ação: Envia os comandos corretos com base no estado
        head_command = String()
        twist_command = Twist()

        if current_state == 'SEARCHING':
            # Comanda a cabeça para procurar e o corpo para girar
            head_command.data = 'SEARCHING'
            twist_command.angular.z = self.spin_speed
            self.get_logger().info("Ação: Busca Ativa (girando corpo e cabeça)", throttle_duration_sec=1)

        elif current_state == 'WALKING':
            # Comanda a cabeça para rastrear e o corpo para andar e alinhar
            head_command.data = 'TRACKING'
            twist_command.linear.x = self.walk_speed
            twist_command.angular.z = self.kp_align * self.head_pan_angle # <-- Alinhamento do corpo!
            self.get_logger().info("Ação: Andando em direção à bola e alinhando", throttle_duration_sec=1)

        elif current_state == 'LOST_BALL_WALK':
            # Comanda a cabeça para rastrear (onde a bola estava) e o corpo para continuar reto
            head_command.data = 'TRACKING'
            twist_command.linear.x = self.walk_speed # Continua andando reto
            self.get_logger().warn("Ação: Bola perdida, andando reto por 2s...", throttle_duration_sec=1)

        # Envia os comandos para os especialistas
        self.head_control_pub.publish(head_command)
        self.cmd_vel_pub.publish(twist_command)

def main(args=None):
    rclpy.init(args=args)
    behavior_node = BehaviorNode()
    try:
        rclpy.spin(behavior_node)
    except KeyboardInterrupt:
        pass
    finally:
        behavior_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()