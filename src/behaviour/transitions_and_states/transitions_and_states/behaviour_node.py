#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
import time

# Mensagens para comunicação
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg
from std_msgs.msg import String as StringMsg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class StateMachine:
    """
    Máquina de estados que gerencia a sequência:
    Buscar -> Alinhar Corpo -> Pausar -> Andar.
    """
    def __init__(self):
        # MUDANÇA: Adicionados novos estados para a sequência
        self.state = 'SEARCHING'
        
        # --- Controles de Timeout ---
        self.LOST_BALL_TIMEOUT = 2.0
        self.PAUSE_DURATION = 1.0 # Duração da pausa em segundos
        
        self.time_ball_was_lost = None
        self.time_pause_started = None

    def update(self, ball_found, head_pan_angle, alignment_tolerance=0.1):
        """
        Executa a lógica de transição de estados.
        'alignment_tolerance' é o quão perto de zero o ângulo da cabeça precisa estar (em radianos).
        """
        previous_state = self.state

        # --- Lógica de Transição de Estados ---

        # Se a bola foi encontrada...
        if ball_found:
            self.time_ball_was_lost = None
            
            # Se estava procurando, a primeira coisa a fazer é alinhar o corpo.
            if self.state == 'SEARCHING' or self.state == 'LOST_BALL_WALK':
                self.state = 'ALIGNING_BODY'
            
            # Se está alinhando o corpo e o alinhamento está concluído...
            elif self.state == 'ALIGNING_BODY' and abs(head_pan_angle) < alignment_tolerance:
                self.state = 'PAUSING' # ...começa a pausa.
                self.time_pause_started = time.time() # Inicia o cronômetro da pausa
            
            # Se está pausando...
            elif self.state == 'PAUSING':
                # ...verifica se a pausa já terminou.
                if time.time() - self.time_pause_started > self.PAUSE_DURATION:
                    self.state = 'WALKING' # Pausa concluída, começa a andar.

            # Se o alinhamento do corpo for perdido durante a caminhada, volta a alinhar
            elif self.state == 'WALKING' and abs(head_pan_angle) > alignment_tolerance * 1.5: # Usa uma tolerância maior para evitar oscilações
                 self.state = 'ALIGNING_BODY'

        # Se a bola NÃO foi encontrada...
        else:
            # Se estava andando ou alinhando e acabou de perder a bola...
            if self.state in ['WALKING', 'ALIGNING_BODY', 'PAUSING']:
                self.state = 'LOST_BALL_WALK'
                self.time_ball_was_lost = time.time()

            # Se já está na fase de paciência...
            elif self.state == 'LOST_BALL_WALK':
                if self.time_ball_was_lost is None: self.time_ball_was_lost = time.time()
                
                if time.time() - self.time_ball_was_lost > self.LOST_BALL_TIMEOUT:
                    self.state = 'SEARCHING'
        
        if previous_state != self.state:
            print(f"[StateMachine] Transição {previous_state} -> {self.state}")
            
        return self.state

class BehaviorNode(Node):
    """
    O Cérebro do Robô. Orquestra a sequência de busca, alinhamento, pausa e caminhada.
    """
    def __init__(self):
        super().__init__('behavior_node')
        self.state_machine = StateMachine()

        # --- Parâmetros ---
        self.declare_parameter('spin_search_speed', 0.22)
        self.declare_parameter('walk_forward_speed', 0.06)
        self.declare_parameter('kp_body_align', 0.6)
        self.declare_parameter('alignment_tolerance_rad', 0.15) # ~5.7 graus

        self.spin_speed = self.get_parameter('spin_search_speed').get_parameter_value().double_value
        self.walk_speed = self.get_parameter('walk_forward_speed').get_parameter_value().double_value
        self.kp_align = self.get_parameter('kp_body_align').get_parameter_value().double_value
        self.alignment_tolerance = self.get_parameter('alignment_tolerance_rad').get_parameter_value().double_value

        # --- Variáveis de Sensores ---
        self.ball_found = False
        self.head_pan_angle = 0.0

        # --- Subscribers e Publishers ---
        self.create_subscription(StateMachineMsg, 'sensor_observer/state_machine_vars', self.sensor_data_callback, 10)
        self.create_subscription(JointState, '/goal_joint_states', self.head_feedback_callback, 10)

        self.state_publisher = self.create_publisher(CurrentStateMsg, '/transitions_and_states/state_machine', 10)
        self.head_control_pub = self.create_publisher(StringMsg, '/head_control/state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Timer Principal ---
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz
        self.get_logger().info("Cérebro do Robô (BehaviorNode) com nova sequência iniciado.")

    def sensor_data_callback(self, msg: StateMachineMsg):
        self.ball_found = msg.ball_found

    def head_feedback_callback(self, msg: JointState):
        try:
            index = msg.name.index('head_pan')
            self.head_pan_angle = msg.position[index]
        except (ValueError, IndexError):
            pass

    def control_loop(self):
        # 1. Decisão: Roda a StateMachine para obter o estado atual
        current_state = self.state_machine.update(
            self.ball_found, 
            self.head_pan_angle,
            self.alignment_tolerance
        )

        # Publica o estado para depuração
        state_msg = CurrentStateMsg(); state_msg.current_state = current_state.lower()
        self.state_publisher.publish(state_msg)
        
        # 2. Ação: Envia os comandos corretos com base no estado
        head_command = StringMsg()
        twist_command = Twist()

        if current_state == 'SEARCHING':
            head_command.data = 'SEARCHING'
            twist_command.angular.z = self.spin_speed
            self.get_logger().info("Ação: Busca Ativa", throttle_duration_sec=1)

        elif current_state == 'ALIGNING_BODY':
            # Mantém a cabeça travada na bola e gira o corpo para alinhar
            head_command.data = 'TRACKING'
            twist_command.angular.z = self.kp_align * self.head_pan_angle
            self.get_logger().info("Ação: Alinhando corpo com a cabeça", throttle_duration_sec=1)
        
        elif current_state == 'PAUSING':
            # Mantém a cabeça travada na bola e para o corpo completamente
            head_command.data = 'TRACKING'
            # Todos os campos de twist já são 0.0 por padrão
            self.get_logger().info("Ação: Pausando por 1s após alinhamento", throttle_duration_sec=0.5)

        elif current_state == 'WALKING':
            # Mantém a cabeça travada na bola e anda para frente
            head_command.data = 'TRACKING'
            twist_command.linear.x = self.walk_speed
            # Opcional: mantém um pequeno alinhamento enquanto anda
            # twist_command.angular.z = self.kp_align * self.head_pan_angle
            self.get_logger().info("Ação: Andando em direção à bola", throttle_duration_sec=1)

        elif current_state == 'LOST_BALL_WALK':
            # Mantém a cabeça olhando para frente e continua andando reto
            head_command.data = 'TRACKING'
            twist_command.linear.x = self.walk_speed
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