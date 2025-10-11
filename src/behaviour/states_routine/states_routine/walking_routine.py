#!/usr/bin/env python3
#coding=utf-8

'''
Recebe da máquina de estados:'walking_routine

Chama o serviço /movement_central/walking para comandar o robô a começar a andar.
'''

import rclpy
from rclpy.node import Node
from modularized_bhv_msgs.msg import CurrentStateMsg 
from geometry_msgs.msg import Twist
from transitions_and_states.behaviour_parameters import BehaviourParameters

class WalkingRoutine(Node):

    def __init__(self):
        super().__init__('walking_node')

        self.parameters = BehaviourParameters()            
   
        self.state_sub = self.create_subscription(
            CurrentStateMsg, self.parameters.currentStateTopic, self.flag_update, 10
        )

        self.walk_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_state = None
        self.last_state = None

    def run_walk(self):
        twist = Twist()

        if self.current_state == 'walking':
            twist.linear.x = self.parameters.maxSpeedLinearX
        else:
            twist.linear.x = 0.0
            
        self.walk_pub.publish(twist)
        self.get_logger().info(f"Comando de caminhada enviado: linear.x = {twist.linear.x}")
    def flag_update(self, msg):
        self.current_state = msg.current_state

        # Só age se o estado mudou
        if self.current_state != self.last_state:
            self.last_state = self.current_state
            self.get_logger().info(f"Estado atual: {self.current_state}")
            self.run_walk()


def main(args=None):
    rclpy.init(args=args)
    routine = WalkingRoutine()
    rclpy.spin(routine)
    routine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()