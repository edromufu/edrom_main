#!/usr/bin/env python3
#coding=utf-8
'''
Node que recebe variáveis da máquina de estados e sinal de término de chute,
atualiza a StateMachine e publica o estado atual no tópico correspondente.
'''

import rclpy
from rclpy.node import Node
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg
from std_msgs.msg import Bool
from .state_machine import StateMachine
from .behaviour_parameters import BehaviourParameters

class StateMachineReceiver(Node):

    def __init__(self):
        """
        Construtor:
        - Inicializa o objeto responsável pelas transições
        - Cria subscribers para as variáveis de comportamento e para o status do chute
        - Publica o estado atual da máquina
        """
        # MUDANÇA: Nome do nó para ser único e descritivo
        super().__init__('behavior_brain_node')

        self.parameters = BehaviourParameters()
        self.state_machine = StateMachine()

        # Inicializa variáveis internas
        self.last_sensor_data_msg = None
        self.kick_done_flag = False

        # Publisher do estado atual
        self.state_publisher = self.create_publisher(
            CurrentStateMsg, '/transitions_and_states/state_machine', 10
        )

        # Subscriber para os dados consolidados dos sensores (do ROSPacker)
        self.create_subscription(
            StateMachineMsg, self.parameters.stateMachineTopic, self.sensor_data_callback, 10
        )

        # Subscriber que ouve o resultado final da rotina de chute
        self.create_subscription(Bool, '/kick_done', self.kick_done_callback, 10)

        self.get_logger().info("StateMachineReceiver (Cérebro) iniciado e aguardando mensagens...")

    def kick_done_callback(self, msg: Bool):
        """Atualiza a flag quando a rotina de chute sinaliza que terminou."""
        self.kick_done_flag = msg.data
        # Após receber a notificação, força uma reavaliação do estado
        self.update_state()

    def sensor_data_callback(self, msg: StateMachineMsg):
        """Recebe a mensagem principal com todos os dados dos sensores."""
        self.last_sensor_data_msg = msg
        self.update_state()

    def update_state(self):
        # Só processa se já recebeu alguma mensagem dos sensores
        if self.last_sensor_data_msg is None:
            return

        sensor_data = self.last_sensor_data_msg

        # Atualiza a máquina de estados com todos os dados e obtém a STRING do novo estado
        state_string_result = self.state_machine.request_state_machine_update(
            ball_found=sensor_data.ball_found,
            ball_close=sensor_data.ball_close,
            #ball_position=sensor_data.ball_position,
            fall_state=sensor_data.fall_state,
            hor_motor_out_of_center=sensor_data.hor_motor_out_of_center,
            head_kick_check=sensor_data.head_kick_check,
            kick_done=self.kick_done_flag
        )
        
        # --- CORREÇÃO PRINCIPAL ---
        # Cria o objeto de mensagem (o "envelope")
        msg_to_publish = CurrentStateMsg()
        # Coloca a string do estado dentro do campo correto
        msg_to_publish.current_state = state_string_result
        
        # Publica a mensagem completa
        self.state_publisher.publish(msg_to_publish)
        self.get_logger().info(f"Estado Atual Publicado: {msg_to_publish.current_state}", throttle_duration_sec=1)


def main(args=None):
    rclpy.init(args=args)
    receiver = StateMachineReceiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()