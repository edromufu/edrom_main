#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor # MUDANÇA: Importar o executor
import os
import sys

# MUDANÇA: Os intérpretes são nós, então devem ser importados como tal
from .fall_interpreter import FallInterpreter
from .ball_interpreter import BallInterpreter
from .neck_interpreter import NeckInterpreter

from modularized_bhv_msgs.msg import StateMachineMsg 
from behaviour_parameters import BehaviourParameters

class ROSPacker(Node):
    """
    Coleta dados dos intérpretes a cada 10 Hz, os empacota e os 
    publica em um tópico se houver alguma mudança.
    """

    # MUDANÇA: O construtor agora recebe as instâncias dos intérpretes
    def __init__(self, ball_interpreter, fall_interpreter, neck_interpreter):
        super().__init__('ros_packer_node')
        self.get_logger().info('Nó ROSPacker iniciado.')

        self.parameters = BehaviourParameters()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # MUDANÇA: Usa as instâncias passadas em vez de criar novas
        self.iBall = ball_interpreter
        self.iFall = fall_interpreter
        self.iNeck = neck_interpreter

        self.pub_to_state_machine = self.create_publisher(
            StateMachineMsg,
            'sensor_observer/state_machine_vars', # Usando o nome do tópico que você mencionou
            qos_profile
        )

        self.state_machine_vars = StateMachineMsg()

        # Inicializa as variáveis com os valores iniciais dos intérpretes
        self.p_ball_position, self.p_ball_close, self.p_ball_found = self.iBall.getValues()
        self.p_fall_state = self.iFall.getValues()
        self.p_hor_motor_out_of_center, self.p_head_kick_check = self.iNeck.getValues() 

        self.sm_vars_last_value = [
            self.p_ball_position, self.p_ball_close, self.p_ball_found,
            self.p_fall_state,
            self.p_hor_motor_out_of_center, self.p_head_kick_check
        ]

        self.timer = self.create_timer(0.1, self.run_loop_callback)
        self.get_logger().info('ROSPacker rodando a 10 Hz.')

    def run_loop_callback(self):
        """
        Callback do timer para atualizar e publicar os dados.
        """
        self.run_values_update()
        
        sm_vars_current_value = [
            self.p_ball_position, self.p_ball_close, self.p_ball_found,
            self.p_fall_state,
            self.p_hor_motor_out_of_center, self.p_head_kick_check
        ]

        # A condição agora deve funcionar, pois os intérpretes estão atualizando seus valores
        if sm_vars_current_value != self.sm_vars_last_value:
            self.sm_vars_last_value = sm_vars_current_value
            self.run_prints()
            self.publish_to_state_machine()

    def run_values_update(self):
        self.p_ball_position, self.p_ball_close, self.p_ball_found = self.iBall.getValues()
        self.p_fall_state = self.iFall.getValues()
        self.p_hor_motor_out_of_center, self.p_head_kick_check = self.iNeck.getValues()         

    def publish_to_state_machine(self):
        self.state_machine_vars.ball_position = self.p_ball_position
        self.state_machine_vars.ball_close = self.p_ball_close
        self.state_machine_vars.ball_found = self.p_ball_found
        self.state_machine_vars.fall_state = self.p_fall_state
        self.state_machine_vars.hor_motor_out_of_center = self.p_hor_motor_out_of_center
        self.state_machine_vars.head_kick_check = self.p_head_kick_check

        self.pub_to_state_machine.publish(self.state_machine_vars)
        self.get_logger().info('Publicando em /state_machine_vars')
    
    def run_prints(self):
        print("----------------------------")
        print("Posicao da bola: ", self.p_ball_position)
        print("Encontrada: ", self.p_ball_found, "   | Bola proxima: ", self.p_ball_close)
        print("Posicao de robo (queda): ", self.p_fall_state)
        print("Posição horizontal da cabeça: ", self.p_hor_motor_out_of_center)
        print("Cabeca confirma o chute: ", self.p_head_kick_check)
        print("----------------------------")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # MUDANÇA: Instancia todos os nós que precisam rodar
        ball_interpreter = BallInterpreter()
        fall_interpreter = FallInterpreter()
        neck_interpreter = NeckInterpreter()
        ros_packer = ROSPacker(ball_interpreter, fall_interpreter, neck_interpreter)
        
        # MUDANÇA: Cria um executor e adiciona todos os nós a ele
        executor = MultiThreadedExecutor()
        executor.add_node(ros_packer)
        executor.add_node(ball_interpreter)
        executor.add_node(fall_interpreter)
        executor.add_node(neck_interpreter)

        # MUDANÇA: "Gira" (spin) o executor, que por sua vez "gira" todos os nós
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        # A limpeza é feita pelo executor, mas é bom garantir
        ros_packer.destroy_node()
        ball_interpreter.destroy_node()
        fall_interpreter.destroy_node()
        neck_interpreter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()