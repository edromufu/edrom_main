#!/usr/bin/env python3
#coding=utf-8
'''
Verificar a adição do game controller e funcionamento no ros2
'''

from transitions import Machine
from modularized_bhv_msgs.msg import CurrentStateMsg

LEFT = 'Left'   # Strings de resposta do interpretador da bola
RIGHT = 'Right'
CENTER = 'Center'
BOTTOM = 'Bottom'
UP = 'Up'

from .behaviour_parameters import BehaviourParameters

class StateMachine:

    def __init__(self):
        """
        Construtor da máquina de estados
        """
        states = ['walking','stand_still','getting_up','kicking','idle_march','impossible']

        go_to_walking_transitions = [
            { 'trigger': 'go_to_walking', 'source': 'walking', 'dest': 'walking',
             'conditions': 'walking_condition', 'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_walking', 'source': 'stand_still', 'dest': 'walking',
             'conditions': 'walking_condition', 'unless': 'getting_up_condition'},

        ]
        
        go_to_stand_still_transitions = [
            { 'trigger': 'go_to_stand_still', 'source': 'stand_still', 'dest': 'stand_still',
             'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_stand_still', 'source': 'getting_up', 'dest': 'stand_still',
             'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_stand_still', 'source': 'kicking', 'dest': 'stand_still',
             'unless': 'getting_up_condition'},
        ]

        go_to_getting_up_transitions = [
            { 'trigger': 'go_to_getting_up', 'source': '*', 'dest': 'getting_up',
             'conditions': 'getting_up_condition'}
        ]

        go_to_kick_transitions = [ 
            { 'trigger': 'go_to_kick', 'source': 'stand_still', 'dest': 'kicking',
             'conditions': 'kick_condition', 'unless': 'getting_up_condition'}
        ]

        go_to_idle_march =[
            {'trigger': 'go_to_idle_march', 'source': 'walking', 'dest': 'idle_march', 
             'conditions': 'kick_condition', 'unless': 'getting_up_condition'}
        ]

        go_to_impossible_transitions = [
            {'trigger': 'go_to_walking', 'source': '*', 'dest': 'impossible', 
             'conditions': 'impossible_condition'},
            {'trigger': 'go_to_getting_up', 'source': '*', 'dest': 'impossible', 
             'conditions': 'impossible_condition'},
            {'trigger': 'go_to_stand_still', 'source': '*', 'dest': 'impossible', 
             'conditions': 'impossible_condition'},
            {'trigger': 'go_to_kick', 'source': '*', 'dest': 'impossible', 
             'conditions': 'impossible_condition'}
        ]

        all_transitions = (go_to_walking_transitions 
                            + go_to_getting_up_transitions 
                            + go_to_stand_still_transitions
                            + go_to_kick_transitions
                            + go_to_impossible_transitions)

        self.robot_state_machine = Machine(
            self, 
            states=states, 
            transitions=all_transitions, 
            initial='stand_still'
        )

        # flags internas (privadas)
        self._walking_condition = False
        self._getting_up_condition = False
        self._kick_condition = False
        self._impossible_condition = False

        self.state_msg = CurrentStateMsg()

    def request_state_machine_update(self, ball_position, ball_close, ball_found, fall_state, hor_motor_out_of_center, head_kick_check):
        """
        Atualiza as condições da máquina de estados e retorna o estado atual
        """
        # Atualiza flags de condição
        self.getting_up_condition_update(fall_state)
        self.walking_condition_update(ball_found)
        self.kick_condition_update(head_kick_check, ball_close)

        print(f'-------------------\nEstado {str(self.state)}')        
        self.update_state()

        return self.state_msg  # Agora só retorna a msg, sem publicar
    
    def update_state(self):

        if self.go_to_kick():
            print('Transição para o kicking\n-------------------\n')
            return True
        elif self.go_to_walking():
            print('Transição para o walking\n-------------------\n')
            return True
        elif self.go_to_getting_up():
            print('Transição para o getting_up\n-------------------\n')
            return True
        elif self.go_to_stand_still():
            print('Transição para o stand_still\n-------------------\n')
            return True
        else:
            return False
    
    # ----- FUNÇÕES UPDATE CONDITION -----
    def getting_up_condition_update(self, fall_state):
        self._getting_up_condition = (fall_state != UP)

    def walking_condition_update(self, ball_found):
        self._walking_condition = bool(ball_found)

    def kick_condition_update(self, head_kick_check, ball_close):
        self._kick_condition = (head_kick_check and ball_close)
        if self._kick_condition:
            print("Condição de chute satisfeita")   

    # ----- FUNÇÕES RETURN CONDITION (usadas pelo transitions) -----
    def walking_condition(self): 
        return self._walking_condition  
    
    def getting_up_condition(self): 
        return self._getting_up_condition  
    
    def kick_condition(self):
        return self._kick_condition
    
    def impossible_condition(self): 
        return self._impossible_condition


