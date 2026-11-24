#!/usr/bin/env python3
"""Imprime a distância da bola ao robô devolvida pelo soccer_ipm de cada filtro.

Cada filtro tem o seu soccer_ipm (/<filtro>/balls_relative, veja o launch),
todos com as MESMAS detecções. Uma linha por mensagem recebida:

  mahony bola 1.243 m
  ekf    bola 1.240 m
"""
import math

import rclpy
from rclpy.node import Node
import soccer_vision_3d_msgs.msg as sv3dm


class IpmFilterPrint(Node):

    def __init__(self):
        super().__init__('ipm_filter_print')
        for f in self.declare_parameter('filters', ['mahony', 'ekf']).value:
            self.create_subscription(sv3dm.BallArray, f'/{f}/balls_relative',
                                     lambda m, f=f: self.on_balls(f, m), 1)

    def on_balls(self, f, msg):
        for b in msg.balls:
            print(f'{f:<14} bola {math.hypot(b.center.x, b.center.y):.3f} m', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = IpmFilterPrint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
