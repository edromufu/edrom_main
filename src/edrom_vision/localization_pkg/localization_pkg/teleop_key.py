# coding=utf-8
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pynput import keyboard
import sys

# Mapeamento de teclas para comandos [dx, dy, d_theta]
MOVE_CMDS = {
    'w': [2.0, 0.0, 0.0],  # Frente
    's': [-2.0, 0.0, 0.0], # Trás
    'a': [0.0, 1.5, 0.0],  # Lado esquerdo (strafe)
    'd': [0.0, -1.5, 0.0], # Lado direito (strafe)
    'q': [0.0, 0.0, 3.0],  # Girar anti-horário
    'e': [0.0, 0.0, -3.0], # Girar horário
}

class TeleopKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_key_node')
        # Tópico para publicar os comandos
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot/command', 10)
        self.get_logger().info("Nó de Controle por Teclado Iniciado.")
        self.get_logger().info("Use W/A/S/D/Q/E para mover. Pressione ESC para sair.")
        self.get_logger().info("!!! MANTENHA ESTE TERMINAL EM FOCO PARA OS COMANDOS FUNCIONAREM !!!")

    def on_press(self, key):
        try:
            char_key = key.char
            if char_key in MOVE_CMDS:
                cmd_data = MOVE_CMDS[char_key]
                msg = Float32MultiArray(data=[float(c) for c in cmd_data])
                self.publisher_.publish(msg)
        except AttributeError:
            if key == keyboard.Key.esc:
                rclpy.shutdown()
                return False

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyNode()
    with keyboard.Listener(on_press=node.on_press) as listener:
        rclpy.spin(node)
        listener.join()
    node.destroy_node()

if __name__ == '__main__':
    main()