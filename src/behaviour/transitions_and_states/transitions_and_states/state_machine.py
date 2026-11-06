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

        self.gc_state = 'PLAYING'
        self.gc_penalty = 0

    def update_game_controller(self, connected, state, penalty, player_found):
        """Atualiza as condições com valores recebidos pelo ROS, sem JSON."""
        if connected:
            self.gc_state = state
            # Não confunde ausência do jogador com penalidade zero nem mantém
            # uma penalidade antiga quando a configuração do jogo muda.
            self.gc_penalty = penalty if player_found else 0
            if not player_found:
                self.gc_state = 'INITIAL'
        else:
            # Em caso de perda de comunicação, assume comportamento normal para não travar em campo
            self.gc_state = 'PLAYING'
            self.gc_penalty = 0

    def request_state_machine_update(self, ball_position, ball_close, ball_found, fall_state,
                                     hor_motor_out_of_center, head_kick_check, kick_done,
                                     gc_connected=False, gc_state='PLAYING', gc_penalty=0,
                                     gc_player_found=False):
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
        self.update_game_controller(gc_connected, gc_state, gc_penalty, gc_player_found)

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
            ALINGNING = 80
            KICKING = 75
            WALKING = 60
            POST_GETTING_UP = 40
            IDLE = 0

        # Tempo de marcha
        if self.state == 'idle_march' and (time.time() - self.enter_time) < self.march_duration:
            control_list.append(('idle_march', PRIORITY.STABILIZATION))
        elif self.state == 'idle' and (time.time() - self.enter_time) < self.idle_duration:
            control_list.append(('idle', PRIORITY.STABILIZATION))

        # Primeiro definindo os estados de maior prioridade
        if self._getting_up_condition:
            control_list.append(('getting_up', PRIORITY.GETTING_UP))

        if self._kick_done_condition:
            control_list.append(('kicking', PRIORITY.KICK_IN_PROGRESS))

        if self._search_condition:
            control_list.append(('searching', PRIORITY.SEARCHING))

        if self.state == 'getting_up' and not self._getting_up_condition:
            control_list.append(('idle_march', PRIORITY.POST_GETTING_UP))

        # Conferindo se estou alinhado
        if self._aligning_condition:
            control_list.append(('aligning', PRIORITY.ALINGNING))

        # Caso eu possa chutar, chute
        if self._kick_condition:
            control_list.append(('kicking', PRIORITY.KICKING))

        # Movimento
        if self._walking_condition:
            control_list.append(('walking', PRIORITY.WALKING))

        # Novo fallback rígido e seguro
        control_list.append(('idle', PRIORITY.IDLE))

        # Mapeamento do que é permitido executar em cada estado do jogo
        allowed_behaviors = {
            'INITIAL': ['getting_up', 'idle'],
            'READY': ['getting_up', 'idle', 'searching', 'walking', 'aligning'],
            'SET': ['getting_up', 'idle'],
            'PLAYING': ['getting_up', 'idle_march', 'idle', 'searching', 'aligning', 'walking', 'kicking'],
            'FINISHED': ['getting_up', 'idle'],
            'PENALIZED': ['getting_up', 'idle']
        }

        # Penalte -> Oq a gente faz quando ta no penalte???
        current_gc_state = self.gc_state
        if self.gc_penalty != 0:
            current_gc_state = 'PENALIZED'
        print(current_gc_state)
        # Extrai quais estados a máquina pode assumir agora
        valid_states = allowed_behaviors.get(current_gc_state, ['getting_up', 'idle'])
        
        # Filtra a lista de controle mantendo apenas os estados permitidos no momento do jogo
        filtered_list = [(estado, prio) for estado, prio in control_list if estado in valid_states]

        if not filtered_list:
            filtered_list.append(('idle', PRIORITY.IDLE))
        print(filtered_list)
        # SELECIONANDO MAIOR PRIORIDADE DA LISTA
        max_val = -1
        novo_estado = self.state

        for estado, prioridade in filtered_list:
            if prioridade > max_val:
                max_val = prioridade
                novo_estado = estado

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
