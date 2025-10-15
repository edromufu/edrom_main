#!/usr/bin/env python3
# coding=utf-8

"""
Recebe da máquina de estados:'getting_up_routine'

Chama o serviço /movement_central/request_page para comandar o robô a executar
a rotina de se levantar após uma queda.
"""

import os
import sys
import rclpy
from rclpy.node import Node

from modularized_bhv_msgs.srv import MoveRequest as Page
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg

from transitions_and_states.behaviour_parameters import BehaviourParameters


class GettingUpRoutine(Node):

    def __init__(self):
        super().__init__('getting_up_node')

        self.parameters = BehaviourParameters()

        # Cliente de serviço
        self.move_request_client = self.create_client(Page, '/movement_central/request_page')
        while not self.move_request_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço /movement_central/request_page...')

        # Subscreve aos tópicos
        self.fall_subscriber = self.create_subscription(
            StateMachineMsg, '/sensor_observer/state_machine_vars', self.fall_status_update, 10)

        self.create_subscription(
            CurrentStateMsg, '/transitions_and_states/state_machine', self.flag_update, 10)

        self.flag = False
        self.current_get_up_page = None

        # Timer
        self.create_timer(self.parameters.timerPage, self.run_get_up)

    def run_get_up(self):
        if self.flag and self.current_get_up_page:
            self.get_logger().info('Routine Get Up')
            request = Page.Request()
            request.page_name = self.current_get_up_page  # Ajuste conforme definição do srv
            self.move_request_client.call_async(request)

    def flag_update(self, msg: CurrentStateMsg):
        if msg.current_state == 'getting_up':
            self.flag = True
            self.get_logger().info('CurrentState recebida')
        else:
            self.flag = False

    def fall_status_update(self, msg: StateMachineMsg):
        if msg.fall_state in (self.parameters.left,
                              self.parameters.right,
                              self.parameters.back):
            self.get_logger().info('Estado da queda recebida')

            self.current_get_up_page = 'aurea_get_up_back'
        elif msg.fall_state == self.parameters.front:
            self.current_get_up_page = 'aurea_get_up_front'


def main(args=None):
    rclpy.init(args=args)
    
    node = GettingUpRoutine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
