#!/usr/bin/env python3
#coding=utf-8

'''
Recebe o estado 'idle_march' da máquina de estados e executa uma rotina de 
parada segura: envia velocidade zero por 1 segundo e depois publica um 
comando para parar a engine de caminhada.
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from rclpy.duration import Duration

# Supondo que sua mensagem de estado venha deste pacote e tenha este nome.
# Se for diferente, ajuste a linha abaixo.
from modularized_bhv_msgs.msg import CurrentStateMsg 

class SearchingRoutine(Node):

    def __init__(self):
        super().__init__('searching_routine_node')

        # Parâmetros de QoS (Qualidade de Serviço)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber para o estado atual da máquina de estados
        # ATENÇÃO: Verifique se o nome do tópico 'self.parameters.currentStateTopic' está correto.
        # Para este exemplo, vou usar um nome explícito.
        self.state_sub = self.create_subscription(
            CurrentStateMsg, 
            '/transitions_and_states/state_machine', # Exemplo de nome de tópico
            self.state_callback, 
            qos_profile
        )
        
        # Publishers
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_walking_pub = self.create_publisher(Empty, '/stop_walking', 10)

        # Variáveis para controlar a lógica de temporização
        self.is_stand_still_required = False # Flag ativada pelo subscriber
        self.stand_still_active = False      # Flag que indica se a sequência de 1s está em andamento
        self.stand_still_start_time = None   # Guarda o momento em que a sequência começou

        # Timer que chama a função de controle principal 20 vezes por segundo (a 20 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Nó SearchingRoutine pronto e rodando.")

    def control_loop(self):
        """
        Esta função é chamada continuamente pelo timer e contém a lógica principal.
        """
        # Condição de entrada: o estado exige 'stand still' E a sequência ainda não começou.
        if self.is_stand_still_required and not self.stand_still_active:
            self.get_logger().info("Iniciando sequência de 'stand still' por 1 segundo...")
            self.stand_still_active = True
            self.stand_still_start_time = self.get_clock().now()

        # Se a sequência está ativa, executa a lógica de temporização.
        if self.stand_still_active:
            elapsed_time = self.get_clock().now() - self.stand_still_start_time
        
            # DURANTE a sequência (enquanto o tempo for menor que 1 segundo)
            if elapsed_time < Duration(seconds=1.0):
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                # Publica continuamente para garantir que o robô receba o comando de parar
                self.velocity_pub.publish(twist)
            
            # FIM da sequência (quando 1 segundo se passar)
            else:
                self.get_logger().info("Tempo concluído. Publicando em /stop_walking e terminando a sequência.")
                
                # 1. Publica a mensagem Empty para o nó de caminhada
                self.stop_walking_pub.publish(Empty())
                
                # 2. Reseta o estado para que a sequência não se repita
                self.stand_still_active = False
                self.stand_still_start_time = None
                
                # 3. Importante: Reseta a flag principal para que a sequência
                #    só seja acionada novamente se um novo comando 'idle_march' chegar.
                self.is_stand_still_required = False

    def state_callback(self, msg):
        """
        Callback que lê o estado da máquina de estados e ativa a flag para iniciar a rotina.
        """
        received_state = msg.current_state 

        # Ativa a flag APENAS se o estado for 'idle_march' e a sequência não estiver já rodando
        if received_state == 'searching' and not self.stand_still_active:
            self.is_stand_still_required = True
        
def main(args=None):
    rclpy.init(args=args)
    routine = SearchingRoutine()
    try:
        rclpy.spin(routine)
    except KeyboardInterrupt:
        pass
    finally:
        routine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()