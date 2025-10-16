#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # MUDANÇA: Usaremos uma String simples para os comandos
from sensor_msgs.msg import JointState
from edrom_msgs.msg import VisionData
import numpy as np

class HeadControllerNode(Node): # MUDANÇA: Nome da classe mais apropriado
    """
    Nó especialista em controle de cabeça. Ouve ordens em /head_control/state
    e executa a busca ou o rastreamento da bola.
    """
    def __init__(self):
        super().__init__('head_controller_node') # MUDANÇA: Nome do nó

        # --- Parâmetros (sem alterações) ---
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('kp_head_pan', -0.001)
        self.declare_parameter('kp_head_tilt', 0.001)
        self.declare_parameter('pan_speed_rad_s', 0.8)
        self.declare_parameter('pan_min_limit_rad', -1.57)
        self.declare_parameter('pan_max_limit_rad', 1.57)
        self.declare_parameter('tilt_initial_rad', -0.05)
        self.declare_parameter('tilt_min_limit_rad', -0.57)
        self.declare_parameter('tilt_step_rad', 0.26)
        
        # ... (obtenção de parâmetros, sem alterações) ...
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
        self.pan_search_angle = 0.0
        self.tilt_search_angle = self.tilt_initial
        self.pan_direction = 1
        self.timer_period = 0.05

        # --- Publishers e Subscribers ---
        self.joint_state_pub = self.create_publisher(JointState, '/goal_joint_states', 10)
        
        # MUDANÇA: Ouve as ordens do gerente da rotina, não da StateMachine principal
        self.state_sub = self.create_subscription(String, '/head_control/state', self.state_callback, 10)
        
        self.vision_sub = self.create_subscription(
            VisionData, 'vision2BhvTopic', self.vision_callback, 10)

        # --- Timer ---
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("Nó Especialista de Controle de Cabeça pronto.")

    def state_callback(self, msg):
        # MUDANÇA: Lógica de estado muito mais simples
        received_state = msg.data.upper() # Converte para maiúsculo: "searching" -> "SEARCHING"
        if self.internal_state != received_state:
            self.get_logger().info(f"Recebida nova ordem: '{received_state}'")
            self.internal_state = received_state
            
            if self.internal_state == "SEARCHING":
                self.pan_search_angle = 0.0
                self.tilt_search_angle = self.tilt_initial
                self.pan_direction = 1
    
    # ... (O resto do arquivo: vision_callback, control_loop, execute_search, execute_tracking, etc. permanece exatamente o mesmo)
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
        else: # IDLE
            self.stop_all_motion()

    def execute_search(self):
        pan_increment = self.pan_speed * self.timer_period
        self.pan_search_angle += pan_increment * self.pan_direction
        
        if self.pan_direction == 1 and self.pan_search_angle >= self.pan_max:
            self.pan_search_angle = self.pan_max; self.pan_direction = -1
            self.tilt_search_angle -= self.tilt_step
            if self.tilt_search_angle < self.tilt_min: self.tilt_search_angle = self.tilt_initial
        elif self.pan_direction == -1 and self.pan_search_angle <= self.pan_min:
            self.pan_search_angle = self.pan_min; self.pan_direction = 1
            self.tilt_search_angle -= self.tilt_step
            if self.tilt_search_angle < self.tilt_min: self.tilt_search_angle = self.tilt_initial
        
        self.set_head_position(self.pan_search_angle, self.tilt_search_angle)
        
    def execute_tracking(self):
        if not self.ball_is_found:
            return

        error_x = self.center_x - self.last_ball_x
        error_y = self.center_y - self.last_ball_y

        pan_target = -self.kp_pan * error_x
        tilt_target = -self.kp_tilt * error_y
        
        pan_target_clipped = np.clip(pan_target, self.pan_min, self.pan_max)
        tilt_target_clipped = np.clip(tilt_target, self.tilt_min, self.tilt_initial)

        self.set_head_position(pan_target_clipped, tilt_target_clipped)

    def stop_all_motion(self):
        self.center_head()

    def center_head(self):
        self.pan_search_angle = 0.0
        self.tilt_search_angle = self.tilt_initial
        self.set_head_position(0.0, self.tilt_initial)
    
    def set_head_position(self, pan_rad, tilt_rad):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['head_pan', 'head_tilt']
        joint_state_msg.position = [pan_rad, tilt_rad]
        self.joint_state_pub.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()