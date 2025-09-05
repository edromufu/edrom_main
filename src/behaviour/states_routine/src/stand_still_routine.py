#!/usr/bin/env python3
#coding=utf-8

'''
Recebe da máquina de estados:'stand_still_routine'

Chama o serviço /movement_central/stand_still para comandar o robô a parar e ficar em pé de forma estável.
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String 
from movement_utils.srv import Page 
from movement_utils.msg import AureaFirstPose 

from ament_index_python.packages import get_package_share_directory
import os


from transitions_and_states.src.behaviour_parameters import BehaviourParameters

class StandStillRoutine(Node):

    def __init__(self):
        super().__init__('stand_still_node')

        self.parameters = BehaviourParameters()
        
       
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
      
        self.move_client = self.create_client(Page, '/movement_central/request_page')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
       
        self.state_sub = self.create_subscription(
            String,
            '/transitions_and_states/state_machine',
            self.flag_update,
            qos_profile
        )

        self.flag = False 
        
       
        self.timer = self.create_timer(self.parameters.timer_first_pose, self.run_stand_still)

    def run_stand_still(self):
        if self.flag:
            self.get_logger().info('Routine Stand Still')
            request = Page.Request()
            request.page_name = 'aurea_first_pose' 
            self.move_client.call_async(request)
    
    def flag_update(self, msg):
        message = msg.data 

        if message == 'stand_still':
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