#!/usr/bin/env python3
#coding=utf-8

'''
Recebe da máquina de estados:'stand_still_routine'

Chama o serviço /movement_central/stand_still para comandar o robô a parar e ficar em pé de forma estável.
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from modularized_bhv_msgs.msg import CurrentStateMsg
from geometry_msgs.msg import Twist

from ament_index_python.packages import get_package_share_directory
import os

from transitions_and_states.behaviour_parameters import BehaviourParameters

class StandStillRoutine(Node):

    def __init__(self):
        super().__init__('idle_march_node')

        self.parameters = BehaviourParameters()
        
       
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Subscriber para o estado atual da máquina de estados
        self.state_sub = self.create_subscription(
            CurrentStateMsg, self.parameters.currentStateTopic, self.flag_update, qos_profile)
        
        # Publisher para rotação
        self.idle_march_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.flag = False

    def run_stand_still(self):
        if self.flag:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Comando de idle march enviado.")
            self.idle_march_pub.publish(twist)

    def flag_update(self, msg):
        message = msg.current_state 

        if message == 'idle_march':
            self.flag = True
        else:
            self.flag = False

def main(args=None):
    rclpy.init(args=args)
    routine = StandStillRoutine()
    rclpy.spin(routine)
    routine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()