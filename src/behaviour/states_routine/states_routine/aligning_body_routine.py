#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from modularized_bhv_msgs.msg import CurrentStateMsg
from transitions_and_states.behaviour_parameters import BehaviourParameters


class BodyAligner(Node):
    """
    Rotina do estado 'aligning'.
    Gira o corpo no próprio eixo até que o pan da cabeça volte para o centro,
    enquanto a cabeça segue rastreando a bola (TRACKING).
    Pan positivo = cabeça olhando para a esquerda -> gira o corpo para a esquerda (z > 0).
    """

    def __init__(self):
        super().__init__('body_aligner')

        self.parameters = BehaviourParameters()

        self.declare_parameter('kp_body_align', 0.8)
        self.kp_body_align = self.get_parameter('kp_body_align').get_parameter_value().double_value

        self.create_subscription(
            CurrentStateMsg, self.parameters.currentStateTopic, self.flag_update, 10
        )
        self.create_subscription(
            JointState, '/goal_joint_states', self.head_callback, 10
        )

        self.body_rotation_pub = self.create_publisher(Twist, '/cmd_vel_align', 10)
        self.head_control_pub = self.create_publisher(String, '/head_control/state', 10)

        self.flag = False
        self.head_pan = 0.0

        self.timer = self.create_timer(0.05, self.update_alignment)  # 20 Hz

        self.get_logger().info("BodyAligner iniciado e aguardando comandos.")

    def flag_update(self, msg):
        active = msg.current_state == 'aligning'
        if active and not self.flag:
            # Garante que a cabeça siga a bola durante o alinhamento
            self.head_control_pub.publish(String(data='TRACKING'))
        self.flag = active

    def head_callback(self, msg):
        # /goal_joint_states também carrega juntas das pernas: busca pelo nome
        try:
            self.head_pan = msg.position[msg.name.index('head_pan')]
        except (ValueError, IndexError):
            pass

    def update_alignment(self):
        if not self.flag:
            return

        twist = Twist()
        max_z = self.parameters.maxSpeedAngularZ
        twist.angular.z = max(-max_z, min(max_z, self.kp_body_align * self.head_pan))

        self.body_rotation_pub.publish(twist)
        self.get_logger().info(
            f"Alinhando corpo: pan={self.head_pan:.2f} rad, z={twist.angular.z:.3f}",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    routine = BodyAligner()
    try:
        rclpy.spin(routine)
    except KeyboardInterrupt:
        pass
    finally:
        routine.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
