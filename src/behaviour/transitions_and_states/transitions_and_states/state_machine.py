#!/usr/bin/env python3
#coding=utf-8

from transitions import Machine
import time

# --- Constantes para clareza ---
CENTER = 'Center'
UP = 'Up'

class StateMachine:
    """
    O cérebro do robô. Gerencia os estados de alto nível com base nos dados interpretados.
    """
    def __init__(self):
        # MUDANÇA: Estados simplificados para refletir a estratégia
        states = ['searching', 'walking', 'aligning_body', 'aligning_foot', 'kicking', 'getting_up']

        # MUDANÇA: Transições simplificadas
        transitions = [
            {'trigger': 'go_to_getting_up', 'source': '*', 'dest': 'getting_up', 'conditions': 'is_fallen'},
            {'trigger': 'go_to_searching', 'source': '*', 'dest': 'searching', 'conditions': 'ball_is_lost'},
            {'trigger': 'go_to_walking', 'source': ['searching', 'aligning_body', 'kicking'], 'dest': 'walking', 'conditions': 'can_walk'},
            {'trigger': 'go_to_aligning_body', 'source': 'walking', 'dest': 'aligning_body', 'conditions': 'needs_body_alignment'},
            {'trigger': 'go_to_aligning_foot', 'source': 'aligning_body', 'dest': 'aligning_foot', 'conditions': 'is_body_aligned'},
            {'trigger': 'go_to_kicking', 'source': 'aligning_foot', 'dest': 'kicking', 'conditions': 'is_foot_aligned_for_kick'},
        ]

        self.robot_state_machine = Machine(self, states=states, transitions=transitions, initial='searching')

        # --- Flags de condição ---
        self._ball_found = False
        self._ball_close = False
        self._fall_state = UP
        self._hor_motor_out_of_center = CENTER
        self._head_kick_check = False # Condição de "cabeça na posição de chute"
        self._kick_done = False
        self._ball_aligned_for_kick = False # Nova flag para alinhamento fino do pé

        # --- Controle de Timeout de Bola Perdida ---
        self.LOST_BALL_TIMEOUT = 3.0
        self.time_ball_last_seen = 0

    def request_state_machine_update(self, ball_found, ball_close, fall_state, hor_motor_out_of_center, head_kick_check, kick_done, ball_aligned_for_kick=False):
        """
        Recebe todos os dados interpretados e atualiza o estado do robô.
        """
        # 1. Atualiza as variáveis internas
        self._ball_found = ball_found
        self._ball_close = ball_close
        self._fall_state = fall_state
        self._hor_motor_out_of_center = hor_motor_out_of_center
        self._head_kick_check = head_kick_check # Presumindo que isso agora significa "cabeça na posição para chutar"
        self._kick_done = kick_done
        self._ball_aligned_for_kick = ball_aligned_for_kick

        if self._ball_found:
            self.time_ball_last_seen = time.time()

        # 2. Executa a lógica de transição de estado
        self.update_state()
        
        # 3. Retorna o estado atual para ser publicado
        return str(self.state)

    def update_state(self):
        """
        Executa a lógica de transição com base nas prioridades.
        """
        # PRIORIDADE 1: Levantar
        if self.is_fallen() and self.state != 'getting_up':
            self.go_to_getting_up()
            return
        
        # Após levantar, volta a procurar
        if self.state == 'getting_up' and not self.is_fallen():
            self.go_to_searching()
            return

        # PRIORIDADE 2: Procurar a bola (se perdida por timeout ou de início)
        if self.ball_is_lost() and self.state != 'searching':
            self.go_to_searching()
            return

        # --- AQUI COMEÇA A LÓGICA DO "SUPER-ESTADO" WALKING ---
        
        # Se estamos procurando e encontramos a bola, começamos a andar
        if self.state == 'searching' and self.can_walk():
            self.go_to_walking()
            return
            
        # Se estamos andando e a bola está longe, continuamos andando
        if self.state == 'walking' and self.can_walk():
            # Permanece em 'walking'
            return

        # Se estamos andando e a bola fica perto, precisamos alinhar o corpo
        if self.state == 'walking' and self.needs_body_alignment():
            self.go_to_aligning_body()
            return
        
        # Se estamos alinhando o corpo e conseguimos, vamos para o alinhamento do pé
        if self.state == 'aligning_body' and self.is_body_aligned():
            self.go_to_aligning_foot()
            return
        
        # Se o alinhamento do corpo falhar (bola saiu do centro), voltamos a andar
        if self.state == 'aligning_body' and not self.needs_body_alignment():
            self.go_to_walking()
            return
        
        # Se estamos alinhando o pé e conseguimos, estamos prontos para chutar
        if self.state == 'aligning_foot' and self.is_foot_aligned_for_kick():
            self.go_to_kicking()
            return
        
        # Se o alinhamento do pé falhar (bola saiu da posição ideal), voltamos a alinhar o corpo
        if self.state == 'aligning_foot' and not self.is_body_aligned():
            self.go_to_aligning_body()
            return
        
        # Após chutar, voltamos a andar para nos reposicionar
        if self.state == 'kicking' and self._kick_done:
            self.go_to_walking()
            return
    
    def kick_done_callback(self, msg):
        self.kick_done_flag = msg.data

    # --- Funções de Condição ---

    def is_fallen(self):
        return self._fall_state != UP

    def ball_is_lost(self):
        # Bola está perdida se nunca foi encontrada ou se o timeout expirou
        time_since_lost = time.time() - self.time_ball_last_seen
        return not self._ball_found and time_since_lost > self.LOST_BALL_TIMEOUT

    def can_walk(self):
        # Podemos andar se a bola foi encontrada e não está perto o suficiente para alinhar/chutar
        return self._ball_found and not self._ball_close
    
    def needs_body_alignment(self):
        # Precisamos alinhar o corpo se a bola está perto, mas não está no centro da visão
        return self._ball_found and self._ball_close and self._hor_motor_out_of_center != CENTER

    def is_body_aligned(self):
        # O corpo está alinhado quando a bola está perto E a cabeça está reta
        return self._ball_found and self._ball_close and self._hor_motor_out_of_center == CENTER

    def is_foot_aligned_for_kick(self):
        # A condição final para chutar (após o alinhamento fino)
        return self.is_body_aligned() and self._ball_aligned_for_kick