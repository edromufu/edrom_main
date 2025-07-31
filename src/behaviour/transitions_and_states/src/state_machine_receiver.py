#!/usr/bin/env python3
#coding=utf-8
'''
Verificar o funcionamento e integração com o restante do ambiente ROS2
'''

import rclpy, os, sys
from state_machine import StateMachine

from modularized_bhv_msgs.msg import stateMachineMsg

edrom_dir = '/home/'+os.getlogin()+'/edromufu/src/'

sys.path.append(edrom_dir+'behaviour/transitions_and_states/src')
from behaviour_parameters import BehaviourParameters

class StateMachineReceiver(StateMachine):

    def __init__(self):
        """
        Construtor:
        - Inicializa o objeto responsável pelas transições
        - Construção do subscriber do ROS responsável pelo recebimento das variáveis
        """

        self.parameters = BehaviourParameters()

        self.state_machine = StateMachine()
        self.node.create_subscription(StateMachineMsg, self.parameters.stateMachineTopic, self.call_state_machine_update, 10)
    
    #Callback do subscriber das variáveis interpretados pelos módulos de interpretação
    def call_state_machine_update(self, stateMachineVars):
        """
        -> Funcao:
        Chamar a atualização de estado sempre que uma variável é alterada, atraves de:
            - Extrair as variaveis da mensagem recebida na ordem estabelecida
            - Chamar o metodo do objeto da state machine com as variaveis 
        -> Input:
            - stateMachineVars: Variavel associada a mensagem recebida no topico do ROS, contem todas as
            informações enviadas pelo ROS packer, da qual são extraidas para atualização   
        """

        self.state_machine.request_state_machine_update(stateMachineVars.ballPosition, stateMachineVars.ballClose, stateMachineVars.ballFound,
                                                        stateMachineVars.fallState,
                                                        stateMachineVars.horMotorOutOfCenter, stateMachineVars.headKickCheck)
    

def main(args=None):
    rclpy.init(args=args)
    
    # Inicializa o node no ROS 2
    node = rclpy.create_node('State_machine_node')

    # Cria a instância do receptor da StateMachine
    receiver = StateMachineReceiver(node)

    # Mantém o node ativo
    rclpy.spin(node)

    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()