#!/usr/bin/env python3
#coding=utf-8
'''
Verificar a adição do game controller e funcionamento no ros2
'''

import time
from modularized_bhv_msgs.msg import CurrentStateMsg

CENTER = 'Center'
LEFT = 'Left'
RIGHT = 'Right'
UP = 'Up'

class StateMachine:

    def __init__(self):
        """
        Construtor da máquina de estados (Refatorada para Utility AI via Lista)
        """
        self.state = 'idle_march'
        self.enter_time = time.time()

        # Variáveis nativas do código original para tempo de estabilização
        self.march_duration = 1.0
        self.idle_duration = 1.5

        # flags internas (privadas)
        self._walking_condition = False
        self._getting_up_condition = False
        self._kick_condition = False
        self._kick_done_condition = False
        self._aligning_condition = False    
        self._search_condition = False
        self._impossible_condition = False

        self.state_msg = CurrentStateMsg()

    def request_state_machine_update(self, ball_position, ball_close, ball_found, fall_state, 
                                     hor_motor_out_of_center, head_kick_check, kick_done):
        """
        Atualiza as condições da máquina de estados e retorna o estado atual
        """
        # Atualiza flags de condição originais
        self.search_condition_update(ball_found)
        self.getting_up_condition_update(fall_state)
        self.walking_condition_update(ball_found, ball_close)
        self.kick_condition_update(head_kick_check, ball_close, hor_motor_out_of_center)
        self.kick_done_condition_update(kick_done)
        self.aligning_condition_update(hor_motor_out_of_center)

        self.update_state()
        
        print(f'-------------------\nEstado {self.state}')        
        self.state_msg.current_state = str(self.state)
        return self.state_msg
    
    def change_state(self, new_state):
        """
        Método centralizador para transição de estados.
        Garante que o relógio de estabilização seja resetado corretamente.
        """
        if self.state != new_state:
            print(f'Transição feita: {self.state} -> {new_state}')
            self.state = new_state
            self.enter_time = time.time()

    def update_state(self):
        """
        Avalia todas as condições através da lista de controle.
        O estado com maior prioridade decide a ação atual.
        """
        control_list = []
        
        # prioridades
        class PRIORITY:
            GETTING_UP = 100
            KICK_IN_PROGRESS = 95  
            SEARCHING = 90
            STABILIZATION = 85
            KICKING = 75           
            ALIGNING = 65
            WALKING = 60
            POST_GETTING_UP = 40
            IDLE = 0

        # 1. Avalia estabilização (mantém o robô preso ao estado pelo tempo mínimo necessário)
        if self.state == 'idle_march' and (time.time() - self.enter_time) < self.march_duration:
            control_list.append(('idle_march', PRIORITY.STABILIZATION))
        elif self.state == 'idle' and (time.time() - self.enter_time) < self.idle_duration:
            control_list.append(('idle', PRIORITY.STABILIZATION))

        # 2. Condições absolutas de alta prioridade
        if self._getting_up_condition:
            control_list.append(('getting_up', PRIORITY.GETTING_UP))

        if self._kick_done_condition:
            control_list.append(('kicking', PRIORITY.KICK_IN_PROGRESS))

        if self._search_condition:
            control_list.append(('searching', PRIORITY.SEARCHING))

        if self.state == 'getting_up' and not self._getting_up_condition:
            control_list.append(('idle_march', PRIORITY.POST_GETTING_UP))

        # 3. Lógica de transição sequencial (Substitui os dicionários .get)
        if self._kick_condition:
            if self.state in ['walking', 'searching']:
                control_list.append(('idle_march', PRIORITY.KICKING))
            elif self.state in ['idle_march', 'aligning']:
                control_list.append(('idle', PRIORITY.KICKING))
            else:
                control_list.append(('kicking', PRIORITY.KICKING))

        elif self._aligning_condition:
            if self.state in ['walking', 'searching']:
                control_list.append(('idle_march', PRIORITY.ALIGNING))
            else:
                control_list.append(('aligning', PRIORITY.ALIGNING))

        # 4. Ação padrão de movimentação
        if self._walking_condition:
            control_list.append(('walking', PRIORITY.WALKING))

        # 5. Segurança Absoluta (Fallback)
        # Se nenhuma condição for verdadeira, o robô retorna por padrão para 'idle_march'
        control_list.append(('idle_march', PRIORITY.IDLE))

        # SELECIONANDO MAIOR PRIORIDADE DA LISTA
        max_val = -1
        novo_estado = self.state

        for estado, prioridade in control_list:
            if prioridade > max_val:
                max_val = prioridade
                novo_estado = estado

        # Executa a transição de estado de forma centralizada
        self.change_state(novo_estado)


    # ----- FUNÇÕES UPDATE CONDITION -----
    def search_condition_update(self, ball_found):
        self._search_condition = not ball_found

    def getting_up_condition_update(self, fall_state):
        self._getting_up_condition = (fall_state != UP)

    def kick_done_condition_update(self, kick_done):
        self._kick_done_condition = kick_done

    def walking_condition_update(self, ball_found, ball_close):
        self._walking_condition = (ball_found and not ball_close)

    def kick_condition_update(self, head_kick_check, ball_close, hor_motor_out_of_center):
        self._kick_condition = (head_kick_check and ball_close and hor_motor_out_of_center == CENTER)

    def aligning_condition_update(self, hor_motor_out_of_center):
        self._aligning_condition = (hor_motor_out_of_center != CENTER) 

    # ----- FUNÇÕES RETURN CONDITION-----
    def search_condition(self): 
        return self._search_condition

    def walking_condition(self): 
        return self._walking_condition  
    
    def getting_up_condition(self): 
        return self._getting_up_condition  
    
    def kick_condition(self):
        return self._kick_condition
    
    def kick_done_condition(self):
        return self._kick_done_condition    
    
    def aligning_condition(self): 
        return self._aligning_condition
    
    def impossible_condition(self): 
        return self._impossible_condition