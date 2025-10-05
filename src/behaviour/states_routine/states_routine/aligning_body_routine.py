#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg
from transitions_and_states.behaviour_parameters import BehaviourParameters

class BodyAligner(Node):
    def __init__(self):
        super().__init__('body_aligner')

        # Inicializa parâmetros
        self.parameters = BehaviourParameters()

        # Subscriber para o estado atual da máquina de estados
        self.create_subscription(
            CurrentStateMsg, '/transitions_and_states/state_machine', self.flag_update, 10
        )

        # Subscriber para ângulo da cabeça (posição relativa)
        self.create_subscription(
            StateMachineMsg,self.parameters.stateMachineTopic,self.head_callback,10
        )

        # Publisher para rotação do corpo
        self.body_rotation_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Flags e variáveis internas
        self.flag = False
        self.rotation_speed = 0.5 # Verificar o valor da velocidade angular
        self.headRelativePos = 'Center'

        self.get_logger().info("BodyAligner iniciado e aguardando comandos.")

    def flag_update(self, msg):
        
        if msg.current_state == 'aligning':
            self.flag = True
        else:
            self.flag = False

    def head_callback(self, msg):
        if not self.flag:
            return  # só age quando o estado for 'aligning'

        self.headRelativePos = msg.hor_motor_out_of_center
        self.update_alignment()

    def update_alignment(self):
        """Decide se precisa girar corpo para alinhar."""
        twist = Twist()

        if self.headRelativePos == 'Left':
            twist.angular.z = self.rotation_speed
        elif self.headRelativePos == 'Right':
            twist.angular.z = -self.rotation_speed
        else:  # Center
            twist.angular.z = 0.0

        self.body_rotation_pub.publish(twist)
        self.get_logger().info(f"Alinhando corpo: cabeça {self.headRelativePos}, z={twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = BodyAligner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
