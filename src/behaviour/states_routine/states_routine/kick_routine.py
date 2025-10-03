#!/usr/bin/env python3
#coding=utf-8

'''
Recebe da máquina de estados:'kick_routine'

Chama o serviço /movement_central/kick para comandar o robô a executar um chute.
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from modularized_bhv_msgs.msg import CurrentStateMsg # Assuming this is the equivalent message type for currentStateMsg
from modularized_bhv_msgs.srv import MoveRequest as Page

class KickRoutine(Node):

    def __init__(self):
        super().__init__('kick_node')

        # Define QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ROS 2 service client
        self.move_client = self.create_client(Page, '/movement_central/request_page')

        # Wait for the service to be available
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # ROS 2 subscriber
        self.state_sub = self.create_subscription(
            CurrentStateMsg,
            '/transitions_and_states/state_machine',
            self.flag_update,
            qos_profile
        )

        self.flag = False
        self.last_decision = None
        self.request = None

        # Create a timer to handle the main logic loop
        self.timer = self.create_timer(0.1, self.main_loop_callback)
    
    def flag_update(self, msg):
        message =  msg.current_state

        if message == 'kick':
            self.flag = True
        else:
            self.flag = False
        
    def create_request(self):
        if self.flag:
            self.request = 'aurea_kick'
        else:
            self.request = None

    def main_loop_callback(self):
        self.create_request()
        
        if self.last_decision != self.request:
            self.last_decision = self.request
            if self.request is not None:
                self.get_logger().info(f"Sending request: {self.request}")
                
                # Create a service request object
                service_request = Page.Request()
                service_request.page_name = self.request # Assuming the service request has a 'page_name' field
                
                # Make an asynchronous service call
                future = self.move_client.call_async(service_request)
                rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    routine = KickRoutine()
    rclpy.spin(routine)
    routine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()