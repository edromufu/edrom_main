#!/usr/bin/env python3
#coding=utf-8

'''
Recebe da máquina de estados:'walking_routine

Chama o serviço /movement_central/walking para comandar o robô a começar a andar.
'''


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import sys


from ament_index_python.packages import get_package_share_directory

from movement_utils.srv import WalkForward
from std_msgs.msg import String  


from transitions_and_states.src.behaviour_parameters  import BehaviourParameters

class WalkingRoutine(Node):

    def __init__(self):
        super().__init__('walking_node')

        self.parameters = BehaviourParameters()
        
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        
        self.move_client = self.create_client(WalkForward, '/movement_central/request_walk')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
   
        self.state_sub = self.create_subscription(
            String,  
            '/transitions_and_states/state_machine',
            self.flag_update,
            qos_profile
        )

        self.supFoot = 1
        self.stepNumber = 6
        self.flag = False 
        
    
        self.timer = self.create_timer(self.parameters.timer_walk, self.run_walk)

    def run_walk(self):
        if self.flag:
            self.get_logger().info('Routine Walk')
            request = WalkForward.Request()
            request.sup_foot = self.supFoot  # Populate the service request fields
            request.step_number = self.stepNumber
            self.move_client.call_async(request)
    
    def flag_update(self, msg):
        message = msg.data 

        if message == 'walking':
            self.flag = True
        else:
            self.flag = False

def main(args=None):
    rclpy.init(args=args)
    routine = WalkingRoutine()
    rclpy.spin(routine)
    routine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()