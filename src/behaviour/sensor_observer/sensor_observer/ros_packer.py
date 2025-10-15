#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import sys

#movimento
from .fall_interpreter import FallInterpreter
from .ball_interpreter import BallInterpreter
from .neck_interpreter import NeckInterpreter

# Importação da mensagem do ROS 2
# Presumimos que a mensagem foi gerada no pacote ROS 2
from modularized_bhv_msgs.msg import StateMachineMsg 

# A importação 'sys.path.append' não é a abordagem padrão do ROS 2. 
# A forma correta é gerenciar as dependências e o PYTHONPATH via setup.py do pacote.
edrom_dir = '/home/' + os.getlogin() + '/edromufu/src/'
sys.path.append(edrom_dir + 'behaviour/transitions_and_states/src')

from behaviour_parameters import BehaviourParameters

class ROSPacker(Node):
    """
    Inicia e utiliza as classes de intérpretes. A cada 10 Hz, 
    coleta os dados, os empacota e os publica em um tópico.
    """

    def __init__(self):
        # Inicializa o nó ROS 2
        super().__init__('ros_packer_node')
        self.get_logger().info('Nó ROSPacker iniciado.')

        # Inicialização da classe de parâmetros
        self.parameters = BehaviourParameters()

        # Configuração QoS para a publicação
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Instanciação dos intérpretes
        self.iBall = BallInterpreter() 
        self.iFall = FallInterpreter() 
        self.iNeck = NeckInterpreter() 
        # ROS 2: Criando o publisher
        self.pub_to_state_machine = self.create_publisher(
            StateMachineMsg,
            self.parameters.stateMachineTopic,
            qos_profile
        )

        # Inicialização da mensagem
        self.state_machine_vars = StateMachineMsg()

        # Variáveis de interpretação
        self.p_ball_position, self.p_ball_close, self.p_ball_found = self.iBall.getValues()
        self.p_fall_state = self.iFall.getValues()
        self.p_hor_motor_out_of_center, self.p_head_kick_check = self.iNeck.getValues() 

        self.sm_vars_last_value = [
            self.p_ball_position, self.p_ball_close, self.p_ball_found,
            self.p_fall_state,
            self.p_hor_motor_out_of_center, self.p_head_kick_check
        ]

        # ROS 2: Cria um timer que chama o callback `run_loop_callback` a 10 Hz
        self.timer = self.create_timer(0.1, self.run_loop_callback)
        self.get_logger().info('ROS_packer_node rodando a 10 Hz.')

    def run_loop_callback(self):
        """
        Callback do timer para atualizar e publicar os dados.
        """
        self.run_values_update()
        self.state_machine_flagger([
            self.p_ball_position, self.p_ball_close, self.p_ball_found,
            self.p_fall_state,
            self.p_hor_motor_out_of_center, self.p_head_kick_check
        ])

    def run_values_update(self):
        self.p_ball_position, self.p_ball_close, self.p_ball_found = self.iBall.getValues()
        self.p_fall_state = self.iFall.getValues()
        self.p_hor_motor_out_of_center, self.p_head_kick_check = self.iNeck.getValues()         

    def state_machine_flagger(self, sm_vars_current_value):
        if not sm_vars_current_value == self.sm_vars_last_value:
            self.sm_vars_last_value = sm_vars_current_value
            self.run_prints()
            self.publish_to_state_machine()

    def publish_to_state_machine(self):
        self.state_machine_vars.ball_position = self.p_ball_position
        self.state_machine_vars.ball_close = self.p_ball_close
        self.state_machine_vars.ball_found = self.p_ball_found
        self.state_machine_vars.fall_state = self.p_fall_state
        self.state_machine_vars.hor_motor_out_of_center = self.p_hor_motor_out_of_center
        self.state_machine_vars.head_kick_check = self.p_head_kick_check

        self.pub_to_state_machine.publish(self.state_machine_vars)
    
    def run_prints(self):
        print("----------------------------")
        print("Posicao da bola: ", self.p_ball_position)
        print("Encontrada: ", self.p_ball_found, "   | Bola proxima: ", self.p_ball_close)
        print("Posicao de robo (queda): ", self.p_fall_state)
        print("Posição horizontal da cabeça: ", self.p_hor_motor_out_of_center)
        print("Cabeca confirma o chute: ", self.p_head_kick_check)
        print("----------------------------")

def main(args=None):
    # Inicializa a biblioteca rclpy
    rclpy.init(args=args)
    
    # Cria a instância do nó
    ros_packer = ROSPacker()
    
    # Faz o nó "girar" (spin) para processar os callbacks.
    try:
        rclpy.spin(ros_packer)
    except KeyboardInterrupt:
        pass
    finally:
        # Destrói o nó e desliga a biblioteca rclpy
        ros_packer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()