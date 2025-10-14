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
from std_msgs.msg import Empty

from ament_index_python.packages import get_package_share_directory
import os
import sys

edrom_dir = '/home/' + os.getlogin() + '/edromufu/src/'
sys.path.append(edrom_dir + 'behaviour/transitions_and_states/src')

from behaviour_parameters import BehaviourParameters

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
        self.idle_march_pub = self.create_publisher(Empty, '/stop_walking', 10)

        self.flag = False
        self.timer = self.create_timer(self.parameters.timer_first_pose, self.run_stand_still)

    def run_stand_still(self):
        if self.flag:
            empty = Empty()
            
            self.idle_march_pub.publish(empty)

    def flag_update(self, msg):
        message = msg.current_state 

        if message == 'march':
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