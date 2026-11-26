#!/usr/bin/env python3
"""Ponte botões -> ROS. Fica aberto (iniciado pelo edrom_buttond.py) e lê
comandos de uma linha no stdin, publicando na hora (sem a demora de abrir
um `ros2 topic pub` a cada aperto).

Comandos:
  VEL <vx> [vy] [wz]   publica /cmd_vel (m/s, rad/s); cancela um HALT pendente
  HALT <atraso_s>      publica /cmd_vel = 0 e, após <atraso_s>, /stop_walking
  STOP                 publica /stop_walking imediatamente
"""
import sys
import threading

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty

REPEAT = 3          # repete o cmd_vel (idempotente) para não perder na descoberta
REPEAT_DT = 0.1


class Helper(Node):
    def __init__(self):
        super().__init__('edrom_buttons_bridge')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_pub = self.create_publisher(Empty, '/stop_walking', 10)
        self.pending_stop = None
        self.lock = threading.Lock()

    def _wait_subs(self, pub, timeout=2.0):
        # espera o walking_engine "enxergar" o publisher (descoberta DDS)
        t = 0.0
        while pub.get_subscription_count() == 0 and t < timeout:
            threading.Event().wait(0.05)
            t += 0.05
        if pub.get_subscription_count() == 0:
            self.get_logger().warn(f'ninguém assina {pub.topic_name} (walking rodando?)')

    def vel(self, vx, vy=0.0, wz=0.0):
        with self.lock:
            if self.pending_stop:
                self.pending_stop.cancel()
                self.pending_stop = None
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self._wait_subs(self.vel_pub)
        for _ in range(REPEAT):
            self.vel_pub.publish(msg)
            threading.Event().wait(REPEAT_DT)
        self.get_logger().info(f'/cmd_vel = ({vx}, {vy}, {wz})')

    def stop(self):
        self._wait_subs(self.stop_pub)
        self.stop_pub.publish(Empty())
        self.get_logger().info('/stop_walking')

    def halt(self, delay):
        self.vel(0.0)
        with self.lock:
            self.pending_stop = threading.Timer(delay, self._delayed_stop)
            self.pending_stop.start()

    def _delayed_stop(self):
        with self.lock:
            self.pending_stop = None
        self.stop()


def _spin_quiet(executor):
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def main():
    rclpy.init()
    node = Helper()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=_spin_quiet, args=(executor,), daemon=True)
    spin.start()
    print('READY', flush=True)
    try:
        serve(node)
    except KeyboardInterrupt:
        pass  # o daemon encerra a ponte com SIGINT ao sair
    with node.lock:
        if node.pending_stop:
            node.pending_stop.cancel()
    executor.shutdown()
    spin.join(timeout=2)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def serve(node):
    for line in sys.stdin:
        parts = line.split()
        if not parts:
            continue
        try:
            if parts[0] == 'VEL':
                node.vel(*[float(x) for x in parts[1:4]])
            elif parts[0] == 'HALT':
                node.halt(float(parts[1]) if len(parts) > 1 else 0.0)
            elif parts[0] == 'STOP':
                node.stop()
        except Exception as e:  # comando malformado não derruba a ponte
            node.get_logger().error(f'comando inválido {line.strip()!r}: {e}')


if __name__ == '__main__':
    main()
