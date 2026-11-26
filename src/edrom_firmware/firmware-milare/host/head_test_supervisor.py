#!/usr/bin/env python3
"""Supervisor de teste da cabeça (botão 5), no lugar do behaviour.

O search_and_track_node começa em IDLE e só se mexe quando alguém publica
/head_control/state. Este nó faz esse papel para testar detecção e tracking:
  bola vista              -> TRACKING
  bola sumida por LOST_S  -> SEARCHING (varredura)
Usa o mesmo filtro de bola do search_and_track_node.
"""
import rclpy
import rclpy.executors
from object_finder.bola import melhor_bola
from rclpy.node import Node
from soccer_vision_2d_msgs.msg import BallArray
from std_msgs.msg import String

LOST_S = 1.5     # tempo sem bola até voltar a procurar


class HeadTestSupervisor(Node):
    def __init__(self):
        super().__init__('head_test_supervisor')
        self.pub = self.create_publisher(String, '/head_control/state', 10)
        self.create_subscription(BallArray, 'balls_in_image', self.on_balls, 10)
        self.last_seen = None
        self.state = None
        self.create_timer(0.2, self.tick)

    def on_balls(self, msg):
        if melhor_bola(msg.balls) is not None:
            self.last_seen = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        seen = (self.last_seen is not None and
                (now - self.last_seen).nanoseconds * 1e-9 < LOST_S)
        state = 'TRACKING' if seen else 'SEARCHING'
        if state != self.state:
            self.get_logger().info(f'cabeça: {state}')
            self.state = state
        self.pub.publish(String(data=state))  # republica: o tracker pode subir depois


def main():
    rclpy.init()
    node = HeadTestSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
