#!/usr/bin/env python3
# coding=utf-8

import numpy as np
import rclpy
import os
import sys
import time
from vision_msgs.msg import Webotsmsg
from movement_utils.srv import *
from movement_utils.msg import *
from geometry_msgs.msg import PoseStamped  # Import para o estado de queda

edrom_dir = '/home/' + os.getlogin() + '/edromufu/src/'

sys.path.append(edrom_dir + 'behaviour/transitions_and_states/src')
from behaviour_parameters import BehaviourParameters

class GoalkeeperBrain:
    def __init__(self):
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('goalkeeper_brain')

        self.parameters = BehaviourParameters()

        # Cria os serviços e indica se foram criados
        self.motorsFeedback = self.node.create_client(HeadFeedback, 'u2d2_comm/feedbackHead')
        self.pageCall = self.node.create_client(Page, 'movement_central/request_page')

        while not self.motorsFeedback.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')
        while not self.pageCall.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')

        # Inscrições nos tópicos
        self.node.create_subscription(Webotsmsg, self.parameters.vision2BhvTopic, self.updateBallParameters, 10)
        self.node.create_subscription(head_motors_data, self.parameters.headPositionsTopic, self.updateHorRotation, 10)
        
    # Novo Subscriber para estado de queda(nao testado)
        self.node.create_subscription(PoseStamped, self.parameters.fallStateTopic, self.updateFallState, 10)


        # Variáveis internas
        self.found = False
        self.x = 0
        self.y = 0
        self.timesFoundFalse = 0
        self.fallState = self.parameters.up  # Estado inicial como "em pé"

    def updateBallParameters(self, msg):
        """
        Callback para atualizar os parâmetros da bola a partir dos dados de visão
        """
        ballInfos = msg.ball

        if not ballInfos.found:
            self.timesFoundFalse += 1
            if self.timesFoundFalse == 3:
                self.found = False
                self.timesFoundFalse = 0
        else:
            self.found = True
            self.x = ballInfos.x
            self.y = ballInfos.y
            self.ballClose = self.y > self.parameters.yCenterBottomLimit
            self.timesFoundFalse = 0

    def updateHorRotation(self, msg):
        """
        Callback para atualizar a rotação horizontal da cabeça
        """
        self.HorRotation, self.VerRotation = msg.pos_vector

    def updateFallState(self, msg):
        """
        Callback para atualizar o estado de queda
        """
        # Define o estado de queda de acordo com a mensagem recebida
        self.fallState = msg.pose.position.x  # Representação do estado da queda

    def run(self):
        while rclpy.ok():
            # Verifica se o robô está em pé antes de realizar ações de defesa
            if self.fallState == self.parameters.up:
                if self.found:
                    self.pageCall('aurea_squat')

                elif self.found and self.ballClose:
                    # Decide a direção da defesa com base na rotação horizontal
                    if self.HorRotation < self.parameters.lookingLeftRad / 2:
                        self.pageCall('aurea_left_defense')
                        self.fall()
                    elif self.HorRotation > self.parameters.lookingRightRad / 2:
                        self.pageCall('aurea_right_defense')
                        self.fall()
            else:
                # Se o robô estiver em queda, chama a página de queda
                self.fall()

    def fall(self):
        """
        Chama a página de 'fallen_aurea' quando detecta uma queda
        """
        self.pageCall('fallen_aurea')

if __name__ == '__main__':
    goalkeeper = goalkeeper_brain()
    goalkeeper.run()
    rclpy.spin(goalkeeper.node)
