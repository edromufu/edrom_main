#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import sys


from transitions_and_states.behaviour_parameters import BehaviourParameters
from sensor_msgs.msg import JointState


class NeckInterpreter(Node):
    """
    Recebe a posição dos motores da cabeça para verificar se a robô está
    em uma boa posição para o chute.
    """

    def __init__(self):
        # Construtor do nó ROS 2
        super().__init__('neck_interpreter')
        self.get_logger().info("Nó NeckInterpreter inicializado.")
        
        self.parameters = BehaviourParameters()
        # Define QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ROS 2: Criando o subscriber
        self.subscription = self.create_subscription(
            JointState, 
            "/goal_joint_states",
            self.callback_positions,
            qos_profile
        )

        # Variáveis de código
        self.horHeadPosition = 'none'
        self.verAngleAccomplished = False


    def getValues(self):
        """
        Retorna a interpretação da posição dos motores da cabeça.
        -> Output:
            - horHeadPosition: Informa a posição horizontal atual da cabeça
            - verAngleAccomplished: Informa se a cabeça está rotacionada verticalmente o suficiente para um bom chute
        """
        return self.horHeadPosition, self.verAngleAccomplished
    
    def callback_positions(self, msg):
        """
        Callback para processar os dados de posição dos motores da cabeça.
        -> Input:
            - msg: Mensagem recebida do tópico de posições dos motores da cabeça.
        """
        hor_motor_value = msg.position[0]
        ver_motor_value = msg.position[1]

        # Lógica de interpretação dos valores
        if (hor_motor_value < self.parameters.lookingLeftRad) and (hor_motor_value > self.parameters.lookingRightRad):
            self.horHeadPosition = 'Center'
        elif hor_motor_value > self.parameters.lookingLeftRad:
            self.horHeadPosition = 'Left'
        else:
            self.horHeadPosition = 'Right'

        if ver_motor_value < self.parameters.minVerRad2Kick:
            self.verAngleAccomplished = True
        else:
            self.verAngleAccomplished = False

def main(args=None):
    # Inicializa a biblioteca rclpy
    rclpy.init(args=args)
    
    # Cria a instância do nó
    neck_interpreter = NeckInterpreter()
    
    # Faz o nó "girar" (spin) para processar os callbacks.
    # Esta função bloqueia a execução até que o nó seja interrompido.
    try:
        rclpy.spin(neck_interpreter)
    except KeyboardInterrupt:
        pass
    finally:
        # Destrói o nó e desliga a biblioteca rclpy
        neck_interpreter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()