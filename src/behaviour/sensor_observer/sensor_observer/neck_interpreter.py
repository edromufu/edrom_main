#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import sys

# Importa a mensagem do ROS 2. 
# Se 'head_motors_data' for uma mensagem personalizada, você precisará gerar o pacote e importá-la corretamente.
# Exemplo: from your_ros2_pkg.msg import HeadMotorsData
# Vamos usar o nome da mensagem fornecido na sua pergunta.
from movement_utils.msg import HeadMotorsData  

# A forma de importar módulos deve ser ajustada no setup.py do pacote.
# Por enquanto, mantemos a lógica, mas a importação 'sys.path.append' não é a abordagem recomendada em ROS 2.
edrom_dir = '/home/'+os.getlogin()+'/edromufu/src/'
sys.path.append(edrom_dir+'behaviour/transitions_and_states/src')

from behaviour.transitions_and_states.src.behaviour_parameters import BehaviourParameters

class NeckInterpreter(Node):
    """
    Recebe a posição dos motores da cabeça para verificar se a robô está
    em uma boa posição para o chute.
    """

    def __init__(self):
        # Construtor do nó ROS 2
        super().__init__('neck_interpreter')
        self.get_logger().info("Nó NeckInterpreter inicializado.")
        
        # O gerenciamento de parâmetros em ROS 2 é diferente. A classe BehaviourParameters
        # precisaria ser adaptada para o sistema de parâmetros do ROS 2, por exemplo, 
        # usando `self.declare_parameter()`. Para esta conversão, mantemos a estrutura original.
        self.parameters = BehaviourParameters()

        # Configuração QoS para comunicação
        # QoS (Quality of Service) é obrigatório em ROS 2 e define como os dados são transmitidos.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Garante que as mensagens serão entregues.
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ROS 2: Criando o subscriber
        # self.create_subscription(tipo_da_msg, nome_do_topico, callback, qos_profile)
        self.subscription = self.create_subscription(
            HeadMotorsData,  # Nome da mensagem, adaptado para ROS 2 (CamelCase)
            self.parameters.headPositionsTopic,
            self.callback_positions,
            qos_profile
        )

        # Variáveis de código
        self.horHeadPosition = 'none'
        self.verAngleAccomplished = False

        # Variáveis para a posição da cabeça (agora constantes de classe ou definidas em parâmetros)
        # Em ROS 2, é comum usar o sistema de parâmetros para esses valores
        # self.lookingLeftRad = 0.5 # Exemplo
        # self.lookingRightRad = -0.5 # Exemplo

    def get_values(self):
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
        hor_motor_value = msg.pos_vector[0]
        ver_motor_value = msg.pos_vector[1]

        # Lógica de interpretação dos valores
        if (hor_motor_value < self.parameters.lookingLeftRad) and (hor_motor_value > self.parameters.lookingRightRad):
            self.horHeadPosition = 'center'
        elif hor_motor_value > self.parameters.lookingLeftRad:
            self.horHeadPosition = 'left'
        else:
            self.horHeadPosition = 'right'

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