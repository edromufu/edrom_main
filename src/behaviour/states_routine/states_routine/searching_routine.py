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
from std_msgs.msg import Empty

from ament_index_python.packages import get_package_share_directory
import os

from transitions_and_states.behaviour_parameters import BehaviourParameters

class StandStillRoutine(Node):

    def __init__(self):
        super().__init__('searching_node')

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
        self.stop_walking_pub = self.create_publisher(Empty, '/stop_walking', 10)


        self.flag = False

    def run_stand_still(self):
    # Condição de entrada: a flag principal está ativa E a sequência ainda não começou.
        if self.flag and not self.stand_still_active:
            self.get_logger().info("Iniciando sequência de 'stand still' por 1.5 segundos...")
            self.stand_still_active = True
            self.stand_still_start_time = self.get_clock().now()

    # Se a sequência está ativa, executa a lógica de temporização.
        if self.stand_still_active:
            elapsed_time = self.get_clock().now() - self.stand_still_start_time
        
        # Condição DURANTE a sequência (menos de 1.5s)
            if elapsed_time < Duration(seconds=1.5):
                twist = Twist()
            # Todos os campos já são 0.0 por padrão, mas é bom ser explícito
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.idle_march_pub.publish(twist)
        
        # Condição de FIM da sequência (passou de 1.5s)
            else:
                self.get_logger().info("Tempo concluído. Publicando em /stop_walking e terminando a sequência.")
            
            # 1. Publica a mensagem Empty
                self.stop_walking_pub.publish(Empty())
            
            # 2. Reseta o estado para que a sequência possa ser chamada novamente no futuro
                self.stand_still_active = False
                self.stand_still_start_time = None
            
            # NOTA: Você provavelmente vai querer resetar a flag principal aqui também

    def flag_update(self, msg):
        message = msg.current_state 

        if message == 'searching':
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