#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from modularized_bhv_msgs.msg import CurrentStateMsg
from vision_msgs.msg import Webotsmsg
from edrom_motion.aurea_kick.action import Kick
from transitions_and_states.behaviour_parameters import BehaviourParameters


class KickRoutine(Node):

    def __init__(self):
        super().__init__('kick_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.parameters = BehaviourParameters()
        self.kick_finished = None

        self.flag = False
        self.last_decision = None

        self.state_sub = self.create_subscription(
            CurrentStateMsg, self.parameters.currentStateTopic, self.flag_update, qos_profile
        )

        self.create_subscription(
            Webotsmsg, self.parameters.vision2BhvTopic, self.kick_decision_side, qos_profile
        )

        self.kick_action_client = ActionClient(self, Kick, '/movement_central/kick')

    def flag_update(self, msg):
        self.flag = (msg.current_state == 'kick')

    def kick_decision_side(self, msg):
        if not self.flag:
            return  # Só decide o chute quando o estado for 'kick'

        left_post_box = msg.Leftgoalpost.roi_width * msg.Leftgoalpost.roi_height
        right_post_box = msg.Rightgoalpost.roi_width * msg.Rightgoalpost.roi_height

        if left_post_box > right_post_box:
            self.last_decision = 'esquerda'
        elif right_post_box > left_post_box:
            self.last_decision = 'direita'
        else:
            return

        if not self.kick_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Servidor de ação de chute não disponível.")
            return

        goal = Kick.Goal()
        goal.leg_id = self.last_decision

        self.get_logger().info(f"Enviando chute com a perna {goal.leg_id}")
        future = self.kick_action_client.send_goal_async(goal)
        future.add_done_callback(self.kick_result_callback)

    def kick_result_callback(self, future):
        try:
            goal_handle = future.result()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.kick_result_done)
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar goal para o servidor de chute: {e}")

    def kick_result_done(self, future):
        try:
            result = future.result().result
            self.kick_finished = result.success
            self.get_logger().info(f"Chute finalizado. Sucesso: {self.kick_finished}")
        except Exception as e:
            self.get_logger().error(f"Erro ao obter resultado do chute: {e}")


def main(args=None):
    rclpy.init(args=args)
    routine = KickRoutine()
    rclpy.spin(routine)
    routine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
