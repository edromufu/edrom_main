#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from modularized_bhv_msgs.msg import CurrentStateMsg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from edrom_msgs.msg import VisionData

class WalkingRoutine(Node):
    """
    Nó Gerente da Rotina de Caminhada e Alinhamento.
    Ativado pelos estados 'walking', 'aligning_body' e 'aligning_foot'.
    - Comanda o rastreamento da bola pela cabeça.
    - Comanda o corpo para andar, girar para alinhar ou fazer micro-passos.
    """
    def __init__(self):
        super().__init__('walking_routine_node')

        # --- Variáveis de Controle ---
        self.is_active = False
        self.current_fsm_state = "" # Guarda o estado atual da FSM principal

        # --- Parâmetros ---
        self.declare_parameter('walk_speed_x', 0.1,)
        self.declare_parameter('kp_body_align', 0.8, )
        self.declare_parameter('kp_foot_strafe', 0.001, )
        self.declare_parameter('kp_foot_forward', 0.001,)
        self.declare_parameter('kicking_hotspot_x', 320.0, )
        self.declare_parameter('kicking_hotspot_y', 400.0, )

        # Obtém os valores dos parâmetros
        self.walk_speed_x = self.get_parameter('walk_speed_x').get_parameter_value().double_value
        self.kp_body_align = self.get_parameter('kp_body_align').get_parameter_value().double_value
        self.kp_foot_strafe = self.get_parameter('kp_foot_strafe').get_parameter_value().double_value
        self.kp_foot_forward = self.get_parameter('kp_foot_forward').get_parameter_value().double_value
        self.kicking_hotspot_x = self.get_parameter('kicking_hotspot_x').get_parameter_value().double_value
        self.kicking_hotspot_y = self.get_parameter('kicking_hotspot_y').get_parameter_value().double_value
        
        # --- Variáveis de Sensores ---
        self.head_pan_angle = 0.0
        self.ball_x = 0.0
        self.ball_y = 0.0

        self.timer_period = 0.05  # 20 Hz

        # --- Subscribers ---
        # Ouve as ordens da StateMachine principal
        self.state_sub = self.create_subscription(
            CurrentStateMsg, '/transitions_and_states/state_machine', self.state_callback, 10)
        
        # Ouve o feedback da posição da cabeça para o alinhamento do corpo
        self.head_feedback_sub = self.create_subscription(
            JointState, '/goal_joint_states', self.head_feedback_callback, 10)
            
        # Ouve os dados da visão para o alinhamento do pé
        self.vision_sub = self.create_subscription(
            VisionData, 'vision2BhvTopic', self.vision_callback, 10)

        # --- Publishers ---
        # Comanda o especialista da cabeça
        self.head_control_pub = self.create_publisher(String, '/head_control/state', 10)
        # Comanda o corpo (motor de caminhada)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Timer ---
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info("Rotina de Caminhada e Alinhamento pronta e ouvindo ordens.")

    def state_callback(self, msg):
        """Ativa ou desativa a rotina e armazena o estado atual."""
        self.current_fsm_state = msg.current_state
        
        if self.current_fsm_state in ['walking', 'aligning_body', 'aligning_foot']:
            if not self.is_active:
                self.get_logger().info(f"Ordem '{self.current_fsm_state}' recebida! Ativando rotina.")
            self.is_active = True
        else:
            if self.is_active:
                self.get_logger().info(f"Ordem '{self.current_fsm_state}' recebida. Desativando rotina.")
            self.is_active = False

    def head_feedback_callback(self, msg: JointState):
        """Armazena o ângulo de pan da cabeça."""
        try:
            # Encontra o índice do 'head_pan' na mensagem
            index = msg.name.index('head_pan')
            self.head_pan_angle = msg.position[index]
        except (ValueError, IndexError):
            # Ignora a mensagem se 'head_pan' não for encontrado ou a lista for curta
            pass

    def vision_callback(self, msg: VisionData):
        """Armazena a posição da bola na imagem."""
        if msg.ball.found:
            self.ball_x = float(msg.ball.x)
            self.ball_y = float(msg.ball.y)

    def control_loop(self):
        """Se a rotina estiver ativa, executa a sub-rotina correspondente."""
        if not self.is_active:
            # Garante que nenhum comando de velocidade seja enviado se a rotina estiver inativa
            self.cmd_vel_pub.publish(Twist())
            return

        # --- Ações Comuns a Todos os Sub-estados Ativos ---
        # 1. Comanda o especialista da cabeça para ficar em modo de rastreamento
        head_command = String()
        head_command.data = "TRACKING"
        self.head_control_pub.publish(head_command)

        # --- Seleciona a Sub-Rotina com base no Estado ---
        if self.current_fsm_state == 'walking':
            self.execute_walk_forward()
        elif self.current_fsm_state == 'aligning_body':
            self.execute_body_alignment()
        elif self.current_fsm_state == 'aligning_foot':
            self.execute_foot_alignment()
        else:
            # Estado desconhecido ou de transição, para por segurança
            self.cmd_vel_pub.publish(Twist())
    
    def execute_walk_forward(self):
        """Comanda o robô para andar para frente."""
        twist_command = Twist()
        twist_command.linear.x = self.walk_speed_x
        self.cmd_vel_pub.publish(twist_command)
        self.get_logger().info("Executando: Andar para frente.", throttle_duration_sec=1)

    def execute_body_alignment(self):
        """Comanda o robô para girar no eixo até alinhar o corpo com a cabeça."""
        twist_command = Twist()
        # O giro é proporcional ao ângulo da cabeça. O objetivo é zerar esse ângulo.
        twist_command.angular.z = self.kp_body_align * self.head_pan_angle
        self.cmd_vel_pub.publish(twist_command)
        self.get_logger().info(f"Executando: Alinhar corpo (Ângulo da cabeça: {self.head_pan_angle:.2f})", throttle_duration_sec=1)

    def execute_foot_alignment(self):
        """Comanda micro-passos para alinhar o pé com a bola."""
        error_x = self.kicking_hotspot_x - self.ball_x
        error_y = self.kicking_hotspot_y - self.ball_y

        twist_command = Twist()
        # Passo lateral para corrigir o erro X
        twist_command.linear.y = self.kp_foot_strafe * error_x
        # Passo frontal/traseiro para corrigir o erro Y
        twist_command.linear.x = self.kp_foot_forward * error_y
        
        self.cmd_vel_pub.publish(twist_command)
        self.get_logger().info(f"Executando: Alinhar pé (Erro X: {error_x:.1f}, Erro Y: {error_y:.1f})", throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)
    walking_routine = WalkingRoutine()
    try:
        rclpy.spin(walking_routine)
    except KeyboardInterrupt:
        pass
    finally:
        walking_routine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()