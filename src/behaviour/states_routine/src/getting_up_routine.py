#!/usr/bin/env python3
#coding=utf-8

'''
Recebe da máquina de estados: 'getting_up_routine'
Chama o serviço /movement_central/get_up para comandar o robô a executar a rotina de se levantar após uma queda.
'''

import rclpy
import os
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


from behaviour.transitions_and_states.src.behaviour_parameters import BehaviourParameters

class GettingUpRoutine(Node):

    def __init__(self):
        super().__init__('getting_up_node')
        self.get_logger().info('Nó GettingUpRoutine inicializado.')

        self.parameters = BehaviourParameters()
        
        # Configuração QoS para comunicação
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ROS 2: Cliente de serviço
        self.move_request_cli = self.create_client(
            Page,  # Mude para o nome correto do serviço (ex: 'movement_utils/srv/Page')
            '/movement_central/request_page'
        )

        while not self.move_request_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço /movement_central/request_page não disponível, esperando...')
        
        # ROS 2: Subscribers
        self.create_subscription(
            StateMachineMsg,  # Mude para o nome correto da mensagem
            self.parameters.stateMachineTopic,
            self.fallStatusUpdate,
            qos_profile
        )
        self.create_subscription(
            CurrentStateMsg,  # Mude para o nome correto da mensagem
            '/transitions_and_states/state_machine',
            self.flagUpdate,
            qos_profile
        )
        
        self.flag = False
        self.currentGetUpPage = None
        
        # ROS 2: Timer
        self.create_timer(self.parameters.timerPage, self.runGetUp)

    def runGetUp(self):
        if self.flag:
            self.get_logger().info('Routine Get Up') 
            request = Page.Request() # Cria uma requisição do tipo 'Page'
            request.page_name = self.currentGetUpPage
            self.move_request_cli.call_async(request)
    
    def flagUpdate(self, msg):
        message = msg.current_state

        if message == 'getting_up':
            self.flag = True
        else:
            self.flag = False

    def fallStatusUpdate(self, msg):
        fall_state = msg.fall_state
        
        if fall_state in [self.parameters.left, self.parameters.right, self.parameters.back]:
            self.currentGetUpPage = 'aurea_get_up_back'
        elif fall_state == self.parameters.front:
            self.currentGetUpPage = 'aurea_get_up_front'

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