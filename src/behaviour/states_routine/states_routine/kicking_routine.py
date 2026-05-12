#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from modularized_bhv_msgs.msg import CurrentStateMsg
from std_msgs.msg import Bool

# Importe a definição da sua action de chute
from aurea_kick.action import Kick

class KickingRoutine(Node):
    """
    Nó especialista na rotina de chute.
    Quando ativado pelo estado 'kicking', chama o Action Server de chute
    e publica o resultado no tópico /kick_done.
    """
    def __init__(self):
        super().__init__('kicking_routine_node')

        # --- Variáveis de Controle ---
        self.is_active = False
        self.kick_in_progress = False
        self.goal_handle = None

        # --- Parâmetros ---
        self.declare_parameter('kick_foot', 'right', )
        self.declare_parameter('kick_power', 0.8,)

        self.kick_foot = self.get_parameter('kick_foot').get_parameter_value().string_value
        self.kick_power = self.get_parameter('kick_power').get_parameter_value().double_value

        # --- Subscriber ---
        # Ouve as ordens da StateMachine principal
        self.state_sub = self.create_subscription(
            CurrentStateMsg, 
            '/transitions_and_states/state_machine', 
            self.state_callback, 
            10)

        # --- Action Client ---
        # Ferramenta principal para executar o chute
        self._action_client = ActionClient(self, Kick, 'kick')

        # --- Publisher ---
        # Publica o resultado (conclusão) do chute para a StateMachine
        self.kick_done_pub = self.create_publisher(Bool, '/kick_done', 10)
        
        self.get_logger().info("Rotina de Chute ('kicking_routine') pronta e ouvindo ordens.")

    def state_callback(self, msg):
        """Ativa ou desativa a rotina com base no estado global."""
        if msg.current_state == 'kicking':
            if not self.is_active:
                self.get_logger().info("Ordem 'kicking' recebida! Ativando rotina de chute.")
                self.is_active = True
                # A ação de chutar é acionada apenas uma vez por ativação
                self.send_kick_goal()
        else:
            if self.is_active:
                self.get_logger().info(f"Ordem '{msg.current_state}' recebida. Desativando rotina de chute.")
                # Se o estado mudar no meio do chute, tentamos cancelar
                self.cancel_kick()
            self.is_active = False

    def send_kick_goal(self):
        """Prepara e envia a meta para o Action Server de chute."""
        if self.kick_in_progress:
            self.get_logger().warn("Tentativa de iniciar um chute enquanto outro já está em andamento.")
            return

        self.get_logger().info("Aguardando o servidor da action '/kick'...")
        self._action_client.wait_for_server()

        # 1. Sinaliza que o chute começou (e não está concluído)
        self.kick_in_progress = True
        self.kick_done_pub.publish(Bool(data=False))

        # 2. Cria a mensagem da meta
        goal_msg = Kick.Goal()
        goal_msg.foot = self.kick_foot
        goal_msg.power = self.kick_power
        
        self.get_logger().info(f"Enviando meta de chute: pé={goal_msg.foot}, força={goal_msg.power}")

        # 3. Envia a meta de forma assíncrona
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Verifica se a meta foi aceita pelo servidor."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Meta de chute foi REJEITADA pelo servidor.')
            self.kick_in_progress = False
            return

        self.get_logger().info('Meta de chute ACEITA. Aguardando resultado...')
        
        # Pede o resultado final da action
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Recebe o resultado final quando a action é concluída."""
        result = future.result().result
        
        # Supondo que a action tenha um campo 'success' no resultado
        if result.success:
            self.get_logger().info("Resultado: Chute concluído com SUCESSO!")
        else:
            self.get_logger().warn("Resultado: Chute FALHOU.")
        
        # 4. Sinaliza que o chute terminou
        self.kick_done_pub.publish(Bool(data=True))
        
        # Reseta as flags
        self.kick_in_progress = False
        self.goal_handle = None

    def cancel_kick(self):
        """Tenta cancelar um chute em andamento."""
        if self.goal_handle:
            self.get_logger().info("Cancelando chute em andamento...")
            self.goal_handle.cancel_goal_async()
            self.kick_in_progress = False
            self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    kicking_routine = KickingRoutine()
    try:
        rclpy.spin(kicking_routine)
    except KeyboardInterrupt:
        pass
    finally:
        kicking_routine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()