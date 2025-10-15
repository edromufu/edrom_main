#!/usr/bin/env python3
#coding=utf-8

# Importando as bibliotecas e módulos para ROS 2
import rclpy
import math as m
from controller import Supervisor
from modularized_bhv_msgs.msg import CurrentStateMsg
from modularized_bhv_msgs.srv import MoveRequest

# Setando a grafia correta das requisições para movimento de caminhada
FORWARD = 'walk_forward'
CLOCKWISE = 'rotate_clockwise'
COUNTER_CLOCKWISE = 'rotate_counter_clockwise'
POSSIBLE_REQUESTS = [FORWARD, CLOCKWISE, COUNTER_CLOCKWISE]

# Setando o step para cada tipo de movimento
ROTATION_STEP = 0.1309
WALK_STEP = 0.05

class Robot3DMover():
    def __init__(self, node_instance, supervisor):
        """
        Construtor:
        - Recebe a instância do nó ROS 2 e do supervisor do Webots.
        - Cria os subscribers e services usando a instância do nó.
        """
        self.node = node_instance
        self.general_supervisor = supervisor

        self.n_rotations = 1
        self.last_run = 0
        self.currentState = None
        
        self.req_dict = {'body_alignment': None, 'body_search': None, 'walking': None}

        self.state_subscriber = self.node.create_subscription(
            CurrentStateMsg, 
            '/transitions_and_states/state_machine', 
            self.flagUpdate, 
            10)

        self.move_service = self.node.create_service(
            MoveRequest, 
            '/bhv2mov_communicator/3D_move_requisitions', 
            self.moveSimRobot)
        
        self.init_3D()
        self.node.get_logger().info("Robot3DMover class initialized.")

    # Função de chamada recorrente no bhv_sim 
    def callClock(self):
        """
        -> Funcao:
            - Chamar o metodo para atualizacao interna da rotacao;
            - Executar a última requisição do estado atual solicitada. 
        """
        self.rotationUpdate()
        if (self.general_supervisor.getTime() - self.last_run) > 0.5:
            movement = self.req_dict.get(self.currentState, None)
            if movement:
                self.robot3DClock(movement)

    # Atualização da flag ( na lógica)
    def flagUpdate(self, message):
        self.currentState = message.currentState

    # Função de atualização da rotação dos motores ( na lógica)
    def robot3DClock(self, movement):
        #Definição do sentido de rotação
        if movement == CLOCKWISE:
            increment = -ROTATION_STEP
        elif movement == COUNTER_CLOCKWISE:
            increment = ROTATION_STEP
        else:
            increment = 0 
        for _ in range(self.n_rotations):  
            #Rotação horária ou anti-horária
            if movement == CLOCKWISE or movement == COUNTER_CLOCKWISE:
                new_rotation = self.sim_3D_rotation_field.getSFRotation()[:3]+[self.robot_rotation+increment]
                self.sim_3D_rotation_field.setSFRotation(new_rotation)
        
        #Andar em linha reta 
        if movement == FORWARD:
            x_increment = -WALK_STEP*m.sin(self.robot_rotation)
            z_increment = -WALK_STEP*m.cos(self.robot_rotation)

            new_translation = [self.sim_3D_translation_field.getSFVec3f()[0]+x_increment,self.sim_3D_translation_field.getSFVec3f()[1],self.sim_3D_translation_field.getSFVec3f()[2]+z_increment]
            self.sim_3D_translation_field.setSFVec3f(new_translation)
        
        self.last_run = self.general_supervisor.getTime()

    # Função chamada no loop para atualizar internamente a rotação atual da robo
    def rotationUpdate(self):
        self.robot_rotation = self.sim_3D_rotation_field.getSFRotation()[3]
    
    # Função chamada pelo construtor 
    def init_3D(self):
        sim_3D_robot_node = self.general_supervisor.getFromDef('Robot3D')
        if sim_3D_robot_node is None:
            self.node.get_logger().error("Node 'Robot3D' não encontrado.")
            return

        self.sim_3D_rotation_field = sim_3D_robot_node.getField('rotation')
        self.sim_3D_translation_field = sim_3D_robot_node.getField('translation')

        self.robot_rotation = self.sim_3D_rotation_field.getSFRotation()[3]
        self.robot_translation = self.sim_3D_translation_field.getSFVec3f()
    
    #Função de callback do serviço
    def moveSimRobot(self, request, response):
        """
        -> Funcao:
        Salvar as ultimas requisições de cada codigo como variavel deste codigo.
        """
        if self.currentState == 'walking':
             self.req_dict['walking'] = request.moveRequest
        
        # A lógica de n_rotations também precisaria ser um campo na mensagem de serviço.
        # if request.n_rotations:
        #    self.n_rotations = request.n_rotations
        # else:
        #    self.n_rotations = 1
        self.n_rotations = 1 # Simplificado por enquanto

        self.node.get_logger().info(f"Received 3D move request: {request.moveRequest}")

        response.success = True
        return response