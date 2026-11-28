#!/usr/bin/env python3
#coding=utf-8
'''
Node que recebe variáveis da máquina de estados e sinal de término de chute,
atualiza a StateMachine e publica o estado atual no tópico correspondente.
'''

import rclpy
from rclpy.node import Node
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg
from std_msgs.msg import Bool
from .state_machine import StateMachine
from .behaviour_parameters import BehaviourParameters


class StateMachineReceiver(Node):

    def __init__(self):
        """
        Construtor:
        - Inicializa o objeto responsável pelas transições
        - Cria subscribers para as variáveis de comportamento e para o status do chute
        - Publica o estado atual da máquina
        """
        super().__init__('state_machine_receiver')

        self.parameters = BehaviourParameters()
        self.state_machine = StateMachine()

        # Inicializa variáveis internas
        self.last_state_machine_msg = None
        self.kick_done = False

        # Publisher do estado atual
        self.state_publisher = self.create_publisher(
            CurrentStateMsg, '/transitions_and_states/state_machine', 10
        )

        # Subscribers
        self.create_subscription(
            StateMachineMsg, self.parameters.stateMachineTopic, self.state_machine_callback, 10
        )

        self.create_subscription(
            Bool, "/kick_done", self.kick_done_callback, 10
        )

        self.get_logger().info("StateMachineReceiver iniciado e aguardando mensagens...")
        # Timer para tentar atualizar o estado periodicamente



    # Recebe mensagem principal da máquina (com variáveis de percepção e estado)
    def state_machine_callback(self, msg):
        self.last_state_machine_msg = msg
        self.update_state()

    # Recebe flag indicando se o chute terminou
    def kick_done_callback(self, msg):
        self.kick_done = msg.data

    def update_state(self):
        # Só processa se já recebeu a mensagem principal
        if self.last_state_machine_msg is None:
            return

        stateMachineVars = self.last_state_machine_msg

        # Atualiza a máquina de estados com todos os dados
        state_msg = self.state_machine.request_state_machine_update(
            stateMachineVars.ball_position,
            stateMachineVars.ball_close,
            stateMachineVars.ball_found,
            stateMachineVars.fall_state,
            stateMachineVars.hor_motor_out_of_center,
            stateMachineVars.head_kick_check,
            self.kick_done
        )

        # Publica o estado atual
        self.state_publisher.publish(state_msg)
        self.get_logger().debug(f"Estado publicado: {state_msg.current_state}")


def main(args=None):
    rclpy.init(args=args)
    receiver = StateMachineReceiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
