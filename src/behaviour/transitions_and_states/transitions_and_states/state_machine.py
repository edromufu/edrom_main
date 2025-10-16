#!/usr/bin/env python3
#coding=utf-8
'''
Implementa a lógica da máquina de estados para o comportamento do robô de futebol.
Esta classe é pura lógica Python e não depende diretamente do ROS.
'''

import time

# Constante para clareza
UP = 'Up'

class StateMachine:
    def __init__(self):
        """
        Construtor da máquina de estados.
        Define os estados e inicializa as variáveis de controle.
        """
        self.state = 'SEARCHING' # O robô sempre começa procurando a bola

        # --- Flags de condição internas, atualizadas a cada ciclo ---
        self._ball_found = False
        self._fall_state = UP

        # --- Controle de Timeout de Bola Perdida ---
        self.LOST_BALL_TIMEOUT = 2.0  # Tempo em segundos de "paciência"
        self.time_ball_was_lost = None # Guarda o timestamp de quando a bola foi perdida

        print("[StateMachine] Lógica da Máquina de Estados Funcional iniciada.")

    def request_state_machine_update(self, ball_found, fall_state, **kwargs):
        """
        Ponto de entrada principal. Recebe os dados, atualiza o estado e retorna o estado atual.
        O uso de **kwargs torna a função robusta a argumentos extras que não usamos ainda.
        """
        # 1. Atualiza as variáveis internas com os dados mais recentes
        self._ball_found = ball_found
        self._fall_state = fall_state
        
        # 2. Executa a lógica de decisão para atualizar o estado
        self.update_state()
        
        # 3. Retorna o estado atual como uma string para o chamador
        return self.state

    def update_state(self):
        """
        Executa a lógica de transição de estados com uma ordem clara de prioridades.
        """
        previous_state = self.state

        # PRIORIDADE 1: Levantar se o robô caiu
        if self._fall_state != UP:
            self.state = 'GETTING_UP'
            if previous_state != self.state: print(f"[StateMachine] Transição -> {self.state} (Robô caído!)")
            return
        
        # Se o robô estava se levantando e agora está de pé, volta a procurar
        if previous_state == 'GETTING_UP' and self._fall_state == UP:
            self.state = 'SEARCHING'
            print(f"[StateMachine] Transição {previous_state} -> {self.state} (Recuperação concluída)")
            return

        # --- Lógica de Jogo (quando o robô está de pé) ---

        # Se a bola foi encontrada...
        if self._ball_found:
            self.time_ball_was_lost = None # Reseta o timer de bola perdida
            
            # ... e o robô não estava andando, ele começa a andar.
            if self.state != 'WALKING':
                self.state = 'WALKING'
                print(f"[StateMachine] Transição {previous_state} -> {self.state} (Bola encontrada!)")
        
        # Se a bola NÃO foi encontrada...
        else:
            # ... e o robô estava andando (acabou de perder a bola)...
            if self.state == 'WALKING':
                self.state = 'LOST_BALL_WALK'
                self.time_ball_was_lost = time.time() # Inicia o cronômetro da "paciência"
                print(f"[StateMachine] Transição {previous_state} -> {self.state} (Bola perdida, iniciando busca esperançosa)")

            # ... e o robô já está na fase de paciência...
            elif self.state == 'LOST_BALL_WALK':
                # Garante que o timer foi iniciado
                if self.time_ball_was_lost is None:
                    self.time_ball_was_lost = time.time()
                
                elapsed_time = time.time() - self.time_ball_was_lost
                # ... verificamos se a paciência acabou.
                if elapsed_time > self.LOST_BALL_TIMEOUT:
                    self.state = 'SEARCHING' # Paciência esgotada, inicia a busca ativa
                    print(f"[StateMachine] Transição {previous_state} -> {self.state} (Timeout, iniciando busca global)")
            
            # Se o estado já é SEARCHING, ele permanece em SEARCHING.