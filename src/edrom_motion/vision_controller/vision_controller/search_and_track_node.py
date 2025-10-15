#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from modularized_bhv_msgs.msg import CurrentStateMsg 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from edrom_msgs.msg import VisionData
import numpy as np

class SearchAndTrackNode(Node):
    """
    Nó que implementa um padrão de busca complexo (pan e tilt) e
    o tracking da bola, publicando os comandos em /goal_joint_states.
    """
    def __init__(self):
        super().__init__('search_and_track_node')

        # --- Parâmetros ---
        self.declare_parameter('image_width', 640)
        self.declare_parameter('kp_tracking', 0.005)
        self.declare_parameter('pan_speed_rad_s', 0.5)
        self.declare_parameter('pan_min_limit_rad', -1.57)
        self.declare_parameter('pan_max_limit_rad', 1.57)
        self.declare_parameter('tilt_initial_rad', 0.0)
        self.declare_parameter('tilt_min_limit_rad', -0.52)
        self.declare_parameter('tilt_step_rad', 0.26)

        # Obtém os valores dos parâmetros
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.center_x = self.image_width / 2.0
        self.kp = self.get_parameter('kp_tracking').get_parameter_value().double_value
        self.pan_speed = self.get_parameter('pan_speed_rad_s').get_parameter_value().double_value
        self.pan_min = self.get_parameter('pan_min_limit_rad').get_parameter_value().double_value
        self.pan_max = self.get_parameter('pan_max_limit_rad').get_parameter_value().double_value
        self.tilt_initial = self.get_parameter('tilt_initial_rad').get_parameter_value().double_value
        self.tilt_min = self.get_parameter('tilt_min_limit_rad').get_parameter_value().double_value
        self.tilt_step = self.get_parameter('tilt_step_rad').get_parameter_value().double_value

        # --- Variáveis de Estado Internas ---
        self.current_state = "IDLE"
        self.last_ball_x = 0.0
        self.ball_is_found = False
        
        # Variáveis da Lógica de Busca
        self.pan_target_angle = 0.0
        self.tilt_target_angle = self.tilt_initial
        self.pan_direction = 1
        
        self.timer_period = 0.05

        # --- Publishers ---
        self.joint_state_pub = self.create_publisher(JointState, '/goal_joint_states', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Subscribers ---
        self.state_sub = self.create_subscription(CurrentStateMsg, 'transitions_and_states/state_machine', self.state_callback, 10)
        self.vision_sub = self.create_subscription(
            VisionData, 'self_parameters_vision2BhvTopic', self.vision_callback, 10)

        # --- Timer ---
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info("Nó de Busca e Tracking (com busca avançada) pronto.")

    def state_callback(self, msg):
        if self.current_state != "searching" and msg.current_state == "searching":
            self.get_logger().info("Entrando no estado de busca. Resetando posição da cabeça.")
            self.pan_target_angle = 0.0
            self.tilt_target_angle = self.tilt_initial
            self.pan_direction = 1

        if self.current_state != msg.current_state:
            self.get_logger().info(f"Mudando de estado: {self.current_state} -> {msg.current_state}")
            self.current_state = msg.current_state

    def vision_callback(self, msg: VisionData):
        self.ball_is_found = msg.ball.found
        if self.ball_is_found:
            self.last_ball_x = float(msg.ball.x)

    def control_loop(self):
        if self.current_state == "searching":
            self.execute_search()
        elif self.current_state == "tracking":
            self.execute_tracking()
        else:
            self.stop_all_motion()

    def execute_search(self):
        """Implementa um padrão de busca em 'serpente' com pan e tilt."""
        self.cmd_vel_pub.publish(Twist())

        # Atualiza a posição do pan
        pan_increment = self.pan_speed * self.timer_period
        self.pan_target_angle += pan_increment * self.pan_direction
        
        ## LÓGICA CORRIGIDA ##
        # Verifica se atingiu o limite DIREITO enquanto se movia para a DIREITA
        if self.pan_direction == 1 and self.pan_target_angle >= self.pan_max:
            self.pan_target_angle = self.pan_max # Garante que não ultrapasse
            self.pan_direction = -1 # Inverte a direção
            self.tilt_target_angle -= self.tilt_step # Desce o tilt

            if self.tilt_target_angle < self.tilt_min:
                self.tilt_target_angle = self.tilt_initial

        # Verifica se atingiu o limite ESQUERDO enquanto se movia para a ESQUERDA
        elif self.pan_direction == -1 and self.pan_target_angle <= self.pan_min:
            self.pan_target_angle = self.pan_min # Garante que não ultrapasse
            self.pan_direction = 1 # Inverte a direção
            self.tilt_target_angle -= self.tilt_step # Desce o tilt
            
            if self.tilt_target_angle < self.tilt_min:
                self.tilt_target_angle = self.tilt_initial

        # Cria e publica a mensagem JointState para a cabeça
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['head_pan', 'head_tilt']
        joint_state_msg.position = [self.pan_target_angle, self.tilt_target_angle]
        
        self.joint_state_pub.publish(joint_state_msg)
        
    def execute_tracking(self):
        """Move a base do robô e centraliza a cabeça."""
        self.center_head()
        if self.ball_is_found:
            error = self.center_x - self.last_ball_x
            angular_z = self.kp * error
            robot_command = Twist()
            robot_command.angular.z = angular_z
            self.cmd_vel_pub.publish(robot_command)
        else:
            self.cmd_vel_pub.publish(Twist()) 

    def stop_all_motion(self):
        """Para a base e centraliza a cabeça."""
        self.cmd_vel_pub.publish(Twist())
        self.center_head()

    def center_head(self):
        """Função auxiliar para publicar o comando de centralizar a cabeça."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['head_pan', 'head_tilt']
        joint_state_msg.position = [0.0, self.tilt_initial]
        self.joint_state_pub.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SearchAndTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all_motion()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()