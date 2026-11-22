#!/usr/bin/env python3
"""Recebe /button_event (std_msgs/Int8) e dispara ações no robô."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String

ACTIONS = {
    1: 'iniciar_jogo',
    2: 'calibrar',
    3: 'modo_teste',
    4: 'parar',
    5: 'reset',
}


class ButtonHandler(Node):
    def __init__(self):
        super().__init__('button_handler')
        self.create_subscription(Int8, 'button_event', self.on_button, 10)
        # Exemplo de saída: outro nó assina /robot_command e executa
        self.cmd_pub = self.create_publisher(String, 'robot_command', 10)

    def on_button(self, msg: Int8):
        action = ACTIONS.get(msg.data)
        if action is None:
            self.get_logger().warn(f'Botão desconhecido: {msg.data}')
            return
        self.get_logger().info(f'Botão {msg.data} -> {action}')
        self.cmd_pub.publish(String(data=action))
        # Para iniciar algo existente, chame aqui um serviço/ação
        # ou rode um launch (subprocess) conforme a necessidade.


def main():
    rclpy.init()
    rclpy.spin(ButtonHandler())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
