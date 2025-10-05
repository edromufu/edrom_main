#!/usr/bin/env python3
#coding=utf-8
'''
Verificar o funcionamento e integração com o restante do ambiente ROS2
'''

import rclpy
from .state_machine import StateMachine
from rclpy.node import Node
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg
from .behaviour_parameters import BehaviourParameters

class StateMachineReceiver(Node):

    def __init__(self):
        """
        Construtor:
        - Inicializa o objeto responsável pelas transições
        - Construção do subscriber do ROS responsável pelo recebimento das variáveis
        - Construção do publisher do ROS responsável por enviar o estado atual
        """
        super().__init__('state_machine_receiver')

        self.parameters = BehaviourParameters()
        self.state_machine = StateMachine()

        # Se inscrever no topico de retorno do KICK

        self.state_publisher = self.create_publisher(
            CurrentStateMsg,
            '/transitions_and_states/state_machine',
            10
        )

        self.create_subscription(
            StateMachineMsg,
            self.parameters.stateMachineTopic,
            self.call_state_machine_update,
            10
        )

    # Atualiza o estado da máquina chamando o método request_state_machine_update
    # e passando as variáveis da mensagem StateMachineMsg recebida
    def call_state_machine_update(self, stateMachineVars):
        state_msg = self.state_machine.request_state_machine_update(
            stateMachineVars.ball_position,
            stateMachineVars.ball_close,
            stateMachineVars.ball_found,
            stateMachineVars.fall_state,
            stateMachineVars.hor_motor_out_of_center,
            stateMachineVars.head_kick_check
        )

        self.state_publisher.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Cria a instância do receptor da StateMachine
    receiver = StateMachineReceiver()

    # Mantém o node ativo
    rclpy.spin(receiver)

    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()