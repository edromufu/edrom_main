#!/usr/bin/env python3
#coding=utf-8
'''
Verificar a adição do game controller e funcionamento no ros2
'''

from transitions import Machine
import time

CENTER = 'Center'
LEFT = 'Left'
RIGHT = 'Right'
UP = 'Up'


from modularized_bhv_msgs.msg import CurrentStateMsg

class StateMachine:

    def __init__(self):
        """
        Construtor da máquina de estados
        """
        states = ['walking','getting_up','kicking','idle','searching','idle_march', 'aligning','impossible']

        go_to_walking_transitions = [
            { 'trigger': 'go_to_walking', 'source': 'walking', 'dest': 'walking',
             'conditions': 'walking_condition', 'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_walking', 'source': 'idle', 'dest': 'walking',
             'conditions': 'walking_condition', 'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_walking', 'source': 'kicking', 'dest': 'walking',
             'conditions': 'walking_condition', 'unless': 'getting_up_condition'},
             { 'trigger': 'go_to_walking', 'source': 'searching', 'dest': 'walking',
             'conditions': 'walking_condition', 'unless': 'getting_up_condition'},
        ]
        
        go_to_idle_march_transitions = [
            { 'trigger': 'go_to_idle_march', 'source': 'idle_march', 'dest': 'idle_march',
             'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_idle_march', 'source': 'getting_up', 'dest': 'idle_march',
             'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_idle_march', 'source': 'walking', 'dest': 'idle_march',
             'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_idle_march', 'source': 'kicking', 'dest': 'idle_march',
             'conditions': 'kick_done_condition', 'unless': 'getting_up_condition'},
        ]
        
        go_to_idle_transitions = [
            { 'trigger': 'go_to_idle', 'source': 'idle_march', 'dest': 'idle',
              'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_idle', 'source': 'aligning', 'dest': 'idle',
              'unless': 'getting_up_condition'},
            { 'trigger': 'go_to_idle', 'source': 'kicking', 'dest': 'idle',
              'unless': 'getting_up_condition'}
        ]
        
        go_to_searching_transitions = [
            {'trigger': 'go_to_searching', 'source': '*', 'dest': 'searching',
             'conditions': 'search_condition', 'unless': 'getting_up_condition'}
        ]
        
        go_to_getting_up_transitions = [
            { 'trigger': 'go_to_getting_up', 'source': '*', 'dest': 'getting_up',
             'conditions': 'getting_up_condition'}
        ]

        go_to_aligning_transitions = [
            {'trigger': 'go_to_aligning', 'source': 'aligning', 'dest': 'aligning',
             'conditions': 'aligning_condition', 'unless': 'getting_up_condition'},
            {'trigger': 'go_to_aligning', 'source': 'idle_march', 'dest': 'aligning',
             'conditions': 'aligning_condition', 'unless': 'getting_up_condition'}
        ]

        go_to_kicking_transitions = [
            {'trigger': 'go_to_kicking', 'source': 'aligning', 'dest': 'kicking',
             'conditions': 'kick_condition', 'unless': 'getting_up_condition'},
            {'trigger': 'go_to_kicking', 'source': 'idle', 'dest': 'kicking',
             'conditions': 'kick_condition', 'unless': 'getting_up_condition'},
            {'trigger': 'go_to_kicking', 'source': 'kicking', 'dest': 'kicking',
             'conditions': 'kick_condition', 'unless': 'getting_up_condition'},
        ]

        go_to_impossible_transitions = [
            {'trigger': 'go_to_walking', 'source': '*', 'dest': 'impossible', 
             'conditions': 'impossible_condition'},
            {'trigger': 'go_to_getting_up', 'source': '*', 'dest': 'impossible', 
             'conditions': 'impossible_condition'},
            {'trigger': 'go_to_idle_march', 'source': '*', 'dest': 'impossible', 
             'conditions': 'impossible_condition'},
            {'trigger': 'go_to_kicking', 'source': '*', 'dest': 'impossible', 
             'conditions': 'impossible_condition'}
        ]

        all_transitions = (go_to_walking_transitions 
                            + go_to_getting_up_transitions
                            + go_to_kicking_transitions
                            + go_to_impossible_transitions
                            + go_to_idle_march_transitions
                            + go_to_aligning_transitions
                            + go_to_idle_transitions
                            + go_to_searching_transitions
                            )


        self.robot_state_machine = Machine(
            self, states=states, transitions=all_transitions, initial='idle_march'
        )
        try:
            from transitions.extensions import GraphMachine
            graph_transitions = [t for t in all_transitions if 'impossible' not in t['trigger']]
            self.graph = GraphMachine(model=self, states=states, transitions=graph_transitions, initial='idle')
            self.graph.get_graph().draw("state_machine_graph.png", prog='dot')
            print("✅ Gráfico da máquina de estados gerado: state_machine_graph.png")
        except Exception as e:
            print(f"⚠️ Não foi possível gerar o gráfico: {e}")

        # flags internas (privadas)
        self._walking_condition = False
        self._getting_up_condition = False
        self._kick_condition = False
        self._kick_done_condition = False
        self._aligning_condition = False    
        self._search_condition = False
        self._impossible_condition = False

        self.enter_time = None
        self.march_duration = 1
        self.idle_duration = 1.5

        self.state_msg = CurrentStateMsg()

    def request_state_machine_update(self, ball_position, ball_close, ball_found, fall_state, 
                                     hor_motor_out_of_center, head_kick_check, kick_done):
        """
        Atualiza as condições da máquina de estados e retorna o estado atual
        """

        # Atualiza flags de condição
        
        self.search_condition_update(ball_found)
        self.getting_up_condition_update(fall_state)
        self.walking_condition_update(ball_found, ball_close)
        self.kick_condition_update(head_kick_check, ball_close, hor_motor_out_of_center)
        self.kick_done_condition_update(kick_done)
        self.aligning_condition_update(hor_motor_out_of_center)

        print(f'-------------------\nEstado {str(self.state)}')        
        self.update_state()
        self.state_msg.current_state = str(self.state)
        return self.state_msg  # Agora só retorna a msg, sem publicar
    
    def update_state(self):
        """
        Atualiza o estado da máquina respeitando:
        - Prioridades
        - Transições válidas
        - Tempo de estabilização (idle_march)
        """

        # Proteção contra transições inválidas (MachineError)
        valid_triggers = self.robot_state_machine.get_triggers(self.state)

        #PRIORIDADE 1: GETTING UP 
        if self.getting_up_condition() and 'go_to_getting_up' in valid_triggers:
            self.go_to_getting_up()
            print(f'Transição de {self.state} para getting_up')
            return

        #PRIORIDADE 2: SEARCHING
        if self._search_condition and self.state != 'searching' and 'go_to_searching' in valid_triggers:
            self.go_to_searching()
            print(f'Transição de {self.state} para searching')
            return
        elif self.state == 'searching' and self.walking_condition() and 'go_to_walking' in valid_triggers:
            self.go_to_walking()
            print(f'Transição de {self.state} para walking')
            return
        
        #PRIORIDADE 3: FASE DE ESTABILIZAÇÃO 
        if self.state == 'idle_march':
            if self.enter_time is None:
                self.enter_time = time.time()

            #Verifica se o tempo mínimo de estabilização passou
            finish_march = (time.time() - self.enter_time) >= self.march_duration

            if finish_march:
                self.enter_time = None  #eset
                #Decide próximo passo: aligning → kicking → walking
                if self._aligning_condition and 'go_to_aligning' in valid_triggers:
                    self.go_to_aligning()
                    print('Transição idle_march → aligning')
                    return
                elif self._kick_condition and self.state == 'idle_march' and 'go_to_idle' in valid_triggers:
                    self.go_to_idle()
                    print('Transição idle_march → idle (pré-chute)')
                    return 
                elif self._walking_condition and 'go_to_walking' in valid_triggers:
                    self.go_to_walking()
                    print('Transição idle_march → walking')
                    return
            else:
                #Ainda estabilizando
                return        
    
        #PRIORIDADE 4: FASE DE ESTABILIZAÇÃO (idle)
        if self.state == 'idle':
            if self.enter_time is None:
                self.enter_time = time.time()
            if (time.time() - self.enter_time) >= self.idle_duration:
                self.enter_time = None
                if self._kick_condition and 'go_to_kicking' in valid_triggers:
                    self.go_to_kicking()
                    print("idle → kicking")
                    return
                
                elif self.state == 'kicking' and self._kick_done_condition and 'go_to_idle' in valid_triggers:
                    self.go_to_idle()
                    print("kicking → idle")
                    return
                elif self._walking_condition and 'go_to_walking' in valid_triggers:
                    self.go_to_walking()
                    print("idle → walking")
                    return
            return

        #PRIORIDADE 3: KICKING 
        if self._kick_condition:
            if self.state == 'aligning' and 'go_to_idle' in valid_triggers:
                self.go_to_idle()
                print('Transição aligning → idle (pré-chute)')
                return
            #Se ainda não passou por march, forçamos o caminho intermediário
            elif self.state == 'walking' and 'go_to_idle_march' in valid_triggers:
                self.go_to_idle_march()
                print('Transição walking → idle_march (pré-chute)')
                return

            elif self.state == 'idle' and 'go_to_kicking' in valid_triggers:
                self.go_to_kicking()
                print('Transição idle → kicking (pré-chute)')
                return
            
        #PRIORIDADE 4: ALIGNING 
        if self._aligning_condition:
            if self.state == 'idle_march' and 'go_to_aligning' in valid_triggers:
                self.go_to_aligning()
                print('Transição idle_march → aligning')
                return
           
            elif self.state == 'walking' and 'go_to_idle_march' in valid_triggers:
                self.go_to_idle_march()
                print('Transição walking → idle_march (pré-alinhamento)')
                return
            
            
        #PRIORIDADE 5: WALKING 
        if self._walking_condition:
            if self.state == 'idle' and 'go_to_walking' in valid_triggers:
                self.go_to_walking()
                print('Transição idle → walking')
                return
            elif self.state == 'aligning' and not self._aligning_condition and 'go_to_walking' in valid_triggers:
                self.go_to_walking()
                print('Transição aligning → walking')
                return
            
        #PRIORIDADE 6: FINAL DO CHUTE 
        if self._kick_done_condition and 'go_to_walking' in valid_triggers:
            self.go_to_walking()
            print('Chute finalizado → walking')
            return

        #PRIORIDADE 7: PÓS-LEVANTAR 
        if self.state =='getting_up' and 'go_to_idle_march' in valid_triggers:
            self.go_to_idle_march()
            print('Pós-ação → idle_march')
            return

        #SE NENHUMA CONDIÇÃO FOI ATENDIDA 
        print(f"Nenhuma transição feita")

    
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

    # ----- FUNÇÕES RETURN CONDITION (usadas pelo transitions) -----
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


