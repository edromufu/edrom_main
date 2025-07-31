#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
import math as m
import numpy as np
from controller import Supervisor

from modularized_bhv_msgs.srv import MoveRequest 

#Setando a grafia correta das requisições de page
KICK = 'kick'
GET_UP_FRONT = 'front_up'
GET_UP_BACK = 'back_up'
POSSIBLE_REQUESTS = [KICK, GET_UP_FRONT, GET_UP_BACK]

class RobotPagesExec(Node): # Herdar de Node

    def __init__(self, supervisor):
        """
        Construtor:
        - Faz a chamada de funções para definir as variáveis para checagem e reprodução das pages.
        """
        super().__init__('robot_pages_exec_node') # Inicializar o nó ROS2
        self.get_logger().info("RobotPagesExec node initialized.")

        self.general_supervisor = supervisor

        # Criação do serviço ROS2
        self.page_service = self.create_service(
            MoveRequest, # Tipo do serviço
            '/bhv2mov_communicator/page_requisitions',
            self.pageSimRobot
        )
        self.response = MoveRequestResponse() 

        self.init_kick_check()

    #Função chamada no construtor para habilitar as checagens e realização da page de chute
    def init_kick_check(self):
        """
        -> Funcao:
        Tornar possível a realização da page de chute, atraves de:
            - Capturar os nodes do Webots necessários para as checagens do chute;
            - Capturar os nodes necessários para aplicação da força no chute.
        """
        self.ball_node = self.general_supervisor.getFromDef('Ball')
        if self.ball_node is None:
            self.get_logger().error("Objeto 'Ball' (para o chute) não encontrado no mundo Webots!")
            # Considerar adicionar um return ou lançar uma exceção se a bola for crítica
            return
            
        self.ball_translation_field = self.ball_node.getField('translation')

        robot_3D_node = self.general_supervisor.getFromDef('Robot3D')
        if robot_3D_node is None:
            self.get_logger().error("Objeto 'Robot3D' (para o chute) não encontrado no mundo Webots!")
            return
            
        self.robot_translation_field = robot_3D_node.getField('translation')
        self.robot_rotation_field = robot_3D_node.getField('rotation')
    
    #Função chamada ao solicitar um chute para verificar sua possibilidade
    def kick(self):
        """
        -> Funcao:
        Avaliar a possibilidade de um chute, atraves de:
            - Verificar se a bola está na frente da robo;
            - Verificar se a bola está próxima à robô;
            - Encontrar o vetor da força do chute.
        """
        # Adicionar verificações para garantir que os campos foram inicializados
        if self.ball_node is None or self.ball_translation_field is None or \
           self.robot_translation_field is None or self.robot_rotation_field is None:
            self.get_logger().error("Campos necessários para a função kick não foram inicializados. Retornando False.")
            return False

        try:
            robot_x_z = [self.robot_translation_field.getSFVec3f()[0]] + [self.robot_translation_field.getSFVec3f()[2]]
            ball_x_z = [self.ball_translation_field.getSFVec3f()[0]] + [self.ball_translation_field.getSFVec3f()[2]]
            robot_theta = self.robot_rotation_field.getSFRotation()[3]

            robot_to_ball_vec = [ball_x_z[0]-robot_x_z[0],ball_x_z[1]-robot_x_z[1]]
            forward_vec = [-0.005*m.sin(robot_theta), -0.005*m.cos(robot_theta)]

            robot_to_ball_auxiliar_vec = [robot_to_ball_vec[0]+forward_vec[0],robot_to_ball_vec[1]+forward_vec[1]]

            distance_robot_to_ball = m.sqrt((robot_to_ball_vec[0]**2)+(robot_to_ball_vec[1]**2))
            is_robot_facing_ball =  distance_robot_to_ball - m.sqrt((robot_to_ball_auxiliar_vec[0]**2)+(robot_to_ball_auxiliar_vec[1]**2))

            # Verificação para evitar divisão por zero se a norma for zero
            norm_robot_to_ball_vec = np.linalg.norm(robot_to_ball_vec)
            if norm_robot_to_ball_vec == 0:
                self.get_logger().warn("Vetor do robô para a bola tem norma zero. Não é possível calcular vetor unitário.")
                return False
            unit_robot_to_ball_vec = robot_to_ball_vec / norm_robot_to_ball_vec

            norm_forward_vec = np.linalg.norm(forward_vec)
            if norm_forward_vec == 0:
                self.get_logger().warn("Vetor forward tem norma zero. Não é possível calcular vetor unitário.")
                return False
            unit_forward_vec = forward_vec / norm_forward_vec

            dot_product = np.dot(unit_robot_to_ball_vec, unit_forward_vec)
            # Garantir que dot_product está no intervalo [-1, 1] para arccos
            dot_product = np.clip(dot_product, -1.0, 1.0)
            angle = np.arccos(dot_product)

            if is_robot_facing_ball < 0 and angle < 0.7 and distance_robot_to_ball < (1+angle/4)*0.16:
                # O 'color' node precisa estar definido no Webots
                color_node = self.general_supervisor.getFromDef('color')
                if color_node:
                    color_node.getField('baseColor').setSFColor([0,1,0])
                else:
                    self.get_logger().warn("Node 'color' não encontrado para mudar a cor da bola.")

                self.ball_node.addForce([unit_robot_to_ball_vec[0],0,unit_robot_to_ball_vec[1]],False)
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f"Erro inesperado na função kick: {e}")
            return False
    
    #Função responsável pela execução das pages de acordo com as requisições
    def pageSimRobot(self, request, response): # ROS2 Services recebem request e response
        """
        -> Funcao:
        Realizar a chamada das funções de execução das pages de acordo com a requisição, atraves de:
            - Comparar a requisição com os padrões definidos;
            - Chamar a função adequada da situação.
        """
        self.get_logger().info(f"Received page request: {request.move_request}") # Log para ver a requisição

        calm_down = False
        if request.move_request in POSSIBLE_REQUESTS: 
            calm_down = True
            initial_time = self.general_supervisor.getTime()
        else:
            self.get_logger().warn(f"Requisição de movimento '{request.move_request}' não reconhecida.")
            response.success = False 
            return response

        if request.move_request == KICK:
            response.success = self.kick() 
        else:
            self.get_logger().warn(f"Lógica para {request.move_request} não implementada.")
            response.success = False 
        if calm_down:
            # Este loop de 'pass' é bloqueante e pode afetar o processamento de ROS2 callbacks
            while self.general_supervisor.getTime()-initial_time < 1:
                pass 

        return response 

