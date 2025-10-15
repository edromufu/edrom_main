#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from robocup_interfaces.msg import BallDetection

class SearchAndTrackNode(Node):
    """
    Nó que implementa os comportamentos de busca (movendo a câmera) e
    de tracking (movendo o robô) com base em um estado externo.
    """
    def __init__(self):
        super().__init__('search_and_track_node')

        # --- Parâmetros Configuráveis ---
        self.declare_parameter('image_width', 640)
        self.declare_parameter('kp_tracking', 0.005, description="Ganho proporcional para o controle de giro do robô.")
        self.declare_parameter('pan_speed', 0.02, description="Velocidade (rad/passo) da varredura da câmera no modo de busca.")
        self.declare_parameter('pan_limit', 1.5, description="Limite em radianos para o movimento de pan da câmera.")

        # Obtém os valores dos parâmetros
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.center_x = self.image_width / 2.0
        self.kp = self.get_parameter('kp_tracking').get_parameter_value().double_value
        self.pan_speed = self.get_parameter('pan_speed').get_parameter_value().double_value
        self.pan_limit_rad = self.get_parameter('pan_limit').get_parameter_value().double_value

        # --- Variáveis de Estado Internas ---
        self.current_state = "IDLE"  # Estado inicial é 'parado'
        self.last_ball_x = 0.0
        self.ball_is_found = False
        
        # --- Variáveis para a Lógica de Busca ---
        self.pan_target_angle = 0.0
        self.pan_direction = 1 # 1 para direita, -1 para esquerda

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.camera_servo_pub = self.create_publisher(Point, '/camera_servos/target_position', 10)

        # --- Subscribers ---
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10)
        self.vision_sub = self.create_subscription(
            BallDetection, '/vision/ball_detection', self.vision_callback, 10)

        # --- Timer para o Loop de Controle Principal ---
        self.timer = self.create_timer(0.05, self.control_loop) # Loop a 20 Hz

        self.get_logger().info("Nó de Busca e Tracking pronto.")

    def state_callback(self, msg):
        """Atualiza o estado atual com base na mensagem da máquina de estados."""
        if self.current_state != msg.data:
            self.get_logger().info(f"Mudando de estado: {self.current_state} -> {msg.data}")
            self.current_state = msg.data

    def vision_callback(self, msg):
        """Armazena o status e a posição da bola a partir da mensagem da visão."""
        self.ball_is_found = msg.found
        if self.ball_is_found:
            self.last_ball_x = msg.center.x

    def control_loop(self):
        """Executa a lógica correspondente ao estado atual a cada passo do timer."""
        if self.current_state == "SEARCHING":
            self.execute_search()
        elif self.current_state == "TRACKING":
            self.execute_tracking()
        else: # IDLE, ou qualquer outro estado não reconhecido
            self.stop_all_motion()

    def execute_search(self):
        """Algoritmo de busca: move a câmera em um padrão de varredura."""
        # Garante que a base do robô esteja parada
        self.cmd_vel_pub.publish(Twist())

        # Atualiza o ângulo alvo para o servo de pan
        self.pan_target_angle += self.pan_speed * self.pan_direction
        
        # Inverte a direção ao atingir os limites
        if self.pan_target_angle > self.pan_limit_rad or self.pan_target_angle < -self.pan_limit_rad:
            self.pan_direction *= -1

        # Publica o comando para os servos da câmera
        servo_command = Point()
        servo_command.x = self.pan_target_angle # x para pan
        servo_command.y = 0.0                   # y para tilt (mantém fixo)
        self.camera_servo_pub.publish(servo_command)
        
    def execute_tracking(self):
        """Algoritmo de tracking: move a base do robô para centralizar a bola."""
        # Centraliza a câmera para que o robô use o corpo para mirar
        self.camera_servo_pub.publish(Point()) 

        if self.ball_is_found:
            # Se a bola foi encontrada, calcula o erro e gira o robô
            error = self.center_x - self.last_ball_x
            angular_z = self.kp * error

            robot_command = Twist()
            robot_command.angular.z = angular_z
            self.cmd_vel_pub.publish(robot_command)
        else:
            # Se o estado é TRACKING mas a bola não está visível, o robô para.
            # A máquina de estados deve detectar essa perda e eventualmente mudar o estado.
            self.cmd_vel_pub.publish(Twist()) 

    def stop_all_motion(self):
        """Função de segurança que para todos os atuadores."""
        self.cmd_vel_pub.publish(Twist())
        self.camera_servo_pub.publish(Point())


def main(args=None):
    rclpy.init(args=args)
    node = SearchAndTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Garante que o robô pare ao fechar o nó
        node.stop_all_motion()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()