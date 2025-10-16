#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from modularized_bhv_msgs.msg import CurrentStateMsg 
from sensor_msgs.msg import JointState
from edrom_msgs.msg import VisionData
import numpy as np

class SearchAndTrackNode(Node):
    """
    Nó especialista em controle de cabeça: implementa um padrão de busca 
    e um rastreamento (pan/tilt) da bola, publicando os comandos em /goal_joint_states.
    """
    def __init__(self):
        super().__init__('search_and_track_node')

        # --- Parâmetros ---
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('kp_head_pan', 0.0005)
        self.declare_parameter('kp_head_tilt', 0.0005)
        self.declare_parameter('pan_speed_rad_s', 0.5)
        self.declare_parameter('pan_min_limit_rad', -1.57)
        self.declare_parameter('pan_max_limit_rad', 1.57)
        self.declare_parameter('tilt_initial_rad', 0.0)
        self.declare_parameter('tilt_min_limit_rad', -0.52)
        self.declare_parameter('tilt_step_rad', 0.26)

        # Obtém os valores dos parâmetros
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.center_x = self.image_width / 2.0
        self.center_y = self.image_height / 2.0
        
        self.kp_pan = self.get_parameter('kp_head_pan').get_parameter_value().double_value
        self.kp_tilt = self.get_parameter('kp_head_tilt').get_parameter_value().double_value
        
        self.pan_speed = self.get_parameter('pan_speed_rad_s').get_parameter_value().double_value
        self.pan_min = self.get_parameter('pan_min_limit_rad').get_parameter_value().double_value
        self.pan_max = self.get_parameter('pan_max_limit_rad').get_parameter_value().double_value
        self.tilt_initial = self.get_parameter('tilt_initial_rad').get_parameter_value().double_value
        self.tilt_min = self.get_parameter('tilt_min_limit_rad').get_parameter_value().double_value
        self.tilt_step = self.get_parameter('tilt_step_rad').get_parameter_value().double_value

        # --- Variáveis de Estado ---
        self.internal_state = "IDLE"
        self.ball_is_found = False
        self.last_ball_x = 0.0
        self.last_ball_y = 0.0
        
        # CORREÇÃO: Usando um nome consistente para o ângulo alvo
        self.pan_target_angle = 0.0
        self.tilt_target_angle = self.tilt_initial
        self.pan_direction = 1
        
        self.timer_period = 0.05

        # --- Publishers e Subscribers ---
        self.joint_state_pub = self.create_publisher(JointState, '/goal_joint_states', 10)
        self.state_sub = self.create_subscription(CurrentStateMsg, 'transitions_and_states/state_machine', self.state_callback, 10)
        self.vision_sub = self.create_subscription(
            VisionData, 'vision2BhvTopic', self.vision_callback, 10)

        # --- Timer ---
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("Nó de Controle de Cabeça (Busca/Track) pronto.")

    def state_callback(self, msg):
        new_internal_state = ""
        received_state = msg.current_state

        if received_state == "searching":
            new_internal_state = "SEARCHING"
        elif received_state in ["walking", "idle_march", "aligning"]:
            new_internal_state = "TRACKING"
        else:
            new_internal_state = "IDLE"

        if self.internal_state != new_internal_state:
            self.get_logger().info(f"Estado da FSM '{received_state}' -> Comportamento mudou de '{self.internal_state}' para '{new_internal_state}'")
            self.internal_state = new_internal_state
            
            if self.internal_state == "SEARCHING":
                self.get_logger().info("Resetando posição da cabeça para iniciar a busca.")
                # CORREÇÃO: Usando a variável correta
                self.pan_target_angle = 0.0
                self.tilt_target_angle = self.tilt_initial
                self.pan_direction = 1

    def vision_callback(self, msg: VisionData):
        self.ball_is_found = msg.ball.found
        if self.ball_is_found:
            self.last_ball_x = float(msg.ball.x)
            self.last_ball_y = float(msg.ball.y)

    def control_loop(self):
        if self.internal_state == "SEARCHING":
            self.execute_search()
        elif self.internal_state == "TRACKING":
            self.execute_tracking()
        else:
            self.stop_all_motion()

    def execute_search(self):
        pan_increment = self.pan_speed * self.timer_period
        # CORREÇÃO: Usando a variável correta
        self.pan_target_angle += pan_increment * self.pan_direction
        
        if self.pan_direction == 1 and self.pan_target_angle >= self.pan_max:
            self.pan_target_angle = self.pan_max; self.pan_direction = -1
            self.tilt_target_angle -= self.tilt_step
            if self.tilt_target_angle < self.tilt_min: self.tilt_target_angle = self.tilt_initial
        elif self.pan_direction == -1 and self.pan_target_angle <= self.pan_min:
            self.pan_target_angle = self.pan_min; self.pan_direction = 1
            self.tilt_target_angle -= self.tilt_step
            if self.tilt_target_angle < self.tilt_min: self.tilt_target_angle = self.tilt_initial
        
        self.set_head_position(self.pan_target_angle, self.tilt_target_angle)
        
    def execute_tracking(self):
        if not self.ball_is_found:
            self.get_logger().warn("Modo Tracking, mas a bola não foi encontrada.", throttle_duration_sec=2.0)
            return

        error_x = self.center_x - self.last_ball_x
        error_y = self.center_y - self.last_ball_y

        pan_adjustment = -self.kp_pan * error_x
        tilt_adjustment = -self.kp_tilt * error_y
        
        # CORREÇÃO: Usando a variável correta
        self.pan_target_angle -= pan_adjustment
        self.tilt_target_angle += tilt_adjustment
        
        self.get_logger().info(
            f"Tracking Inc: Erro(x={error_x:.1f}, y={error_y:.1f}) -> "
            f"Alvo(pan={self.pan_target_angle:.3f}, tilt={self.tilt_target_angle:.3f})",
            throttle_duration_sec=0.5
        )

        self.pan_target_angle = np.clip(self.pan_target_angle, self.pan_min, self.pan_max)
        self.tilt_target_angle = np.clip(self.tilt_target_angle, self.tilt_min, self.tilt_initial)

        self.set_head_position(self.pan_target_angle, self.tilt_target_angle)

    def stop_all_motion(self):
        self.center_head()

    def center_head(self):
        # CORREÇÃO: Usando a variável correta
        self.pan_target_angle = 0.0
        self.tilt_target_angle = self.tilt_initial
        self.set_head_position(0.0, self.tilt_initial)
    
    def set_head_position(self, pan_rad, tilt_rad):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['head_pan', 'head_tilt']
        joint_state_msg.position = [pan_rad, tilt_rad]
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