#!/usr/bin/env python3
# coding=utf-8
"""
dsd_node.py — Nó ROS2 que hospeda a DSD original sem nenhuma modificação.

Toda a "cola" entre ROS2 e a DSD fica aqui:
  - conversão de strings do StateMachineMsg → enums do WorldState
  - leitura do WorldState após dsd.step() → publicação dos comandos ROS
  - timer que chama dsd.step() a cada tick (20 Hz)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from modularized_bhv_msgs.msg import StateMachineMsg, CurrentStateMsg

# Importa a DSD original — sem nenhuma modificação
from .dsd_state_machine import (
    SimpleDSD,
    WorldState,
    BallPosition,
    FallState,
    HeadHorizontalPosition,
)


# Helpers de conversão

def str_to_ball_position(value: str) -> BallPosition:
    """Converte string do ROSPacker para BallPosition enum."""
    for member in BallPosition:
        if member.value.lower() == value.lower():
            return member
    return BallPosition.NONE


def str_to_fall_state(value: str) -> FallState:
    """Converte string do ROSPacker para FallState enum."""
    for member in FallState:
        if member.value.lower() == value.lower():
            return member
    return FallState.UP


def str_to_head_position(value: str) -> HeadHorizontalPosition:
    """Converte string do ROSPacker para HeadHorizontalPosition enum."""
    for member in HeadHorizontalPosition:
        if member.value.lower() == value.lower():
            return member
    return HeadHorizontalPosition.CENTER


def action_name_to_legacy_state(action_name: str) -> str:
    """
    Mapeia o nome da ActionElement ativa para a string de estado legado.
    Mantém compatibilidade com KickingRoutine, GettingUpRoutine etc.
    """
    mapping = {
        "RecoveryAction":         "getting_up",
        "StandUpAction":          "getting_up",
        "SearchBallAction":       "searching",
        "GoToBallAction":         "walking",
        "AlignTrunkAction":       "aligning_body",
        "AlignHeadForKickAction": "aligning_body",
        "KickAction":             "kicking",
        "DoneAction":             "idle",
    }
    return mapping.get(action_name, "idle")


def get_getup_page(fall_state: FallState) -> str:
    """Retorna o nome da página de levantamento com base na direção da queda."""
    return "aurea_get_up_front" if fall_state == FallState.FRONT else "aurea_get_up_back"


# Nó ROS2

class DsdNode(Node):
    """Nó ROS2 que hospeda a DSD original."""

    def __init__(self):
        super().__init__("dsd_behavior_node")

        # Callback groups 
        self._cb_inputs = ReentrantCallbackGroup()
        self._cb_tick   = MutuallyExclusiveCallbackGroup()

        # Parâmetros 
        self.declare_parameter("tick_hz",       20.0) # Frequência de atualização da DSD e publicação de comandos
        self.declare_parameter("use_getup_srv", False) # Se True, chama serviço de levantamento; se False, apenas seta flag interna 
        tick_hz       = self.get_parameter("tick_hz").get_parameter_value().double_value
        self._use_getup_srv = self.get_parameter("use_getup_srv").get_parameter_value().bool_value

        # WorldState + DSD 
        self.world = WorldState()
        self.dsd   = SimpleDSD(self.world)

        # Controle interno para evitar múltiplos eventos de chute ou levantamento simultâneos
        self._kick_in_progress  = False
        self._getup_in_progress = False

        # Publishers 
        self._cmd_vel_pub   = self.create_publisher(Twist,           "/cmd_vel",                              10)
        self._head_pub      = self.create_publisher(String,          "/head_control/state",                   10)
        self._state_pub     = self.create_publisher(CurrentStateMsg, "/transitions_and_states/state_machine", 10)
        self._dsd_state_pub = self.create_publisher(CurrentStateMsg, "/transitions_and_states/dsd_state",     10)

        # Subscribers
        self.create_subscription(
            StateMachineMsg, "/sensor_observer/state_machine_vars",
            self._sensor_cb, 10, callback_group=self._cb_inputs,
        )
        self.create_subscription(
            JointState, "/goal_joint_states",
            self._head_fb_cb, 10, callback_group=self._cb_inputs,
        )
        self.create_subscription(
            Bool, "/kick_done",
            self._kick_done_cb, 10, callback_group=self._cb_inputs,
        )

        # Timer principal 
        self._timer = self.create_timer(
            1.0 / tick_hz,
            self._tick_cb,
            callback_group=self._cb_tick,
        )

        self.get_logger().info(f"DsdNode iniciado — {tick_hz:.0f} Hz")

    # Subscribers — atualizam WorldState com conversão de tipos aqui

    def _sensor_cb(self, msg: StateMachineMsg) -> None:
        """Converte StateMachineMsg → WorldState. Toda conversão de string fica aqui."""
        self.world.ball_found              = msg.ball_found
        self.world.ball_close              = msg.ball_close
        self.world.ball_position           = str_to_ball_position(msg.ball_position)
        self.world.fall_state              = str_to_fall_state(msg.fall_state)
        self.world.head_kick_check         = msg.head_kick_check
        self.world.hor_motor_out_of_center = str_to_head_position(msg.hor_motor_out_of_center)
        self.world.trunk_aligned           = msg.trunk_aligned

    def _head_fb_cb(self, msg: JointState) -> None:
        """Usa o ângulo de pan da cabeça como ball_angle para GoToBallAction."""
        try:
            idx = msg.name.index("head_pan")
            self.world.ball_angle = msg.position[idx]
        except (ValueError, IndexError):
            pass

    def _kick_done_cb(self, msg: Bool) -> None:
        """Recebe confirmação de chute concluído."""
        self.world.kick_done = msg.data
        if msg.data:
            self._kick_in_progress = False


    # Timer — tick principal

    def _tick_cb(self) -> None:
        # 1. Roda um passo da DSD original
        self.dsd.step()

        # 2. Lê o topo da pilha para saber o que está ativo
        top      = self.dsd.stack[-1] if self.dsd.stack else None
        top_name = top.name if top else "None"

        # 3. Deduz comandos a partir do nome da action ativa
        twist    = Twist()
        head_cmd = String()
        head_cmd.data = "IDLE"

        if top_name == "SearchBallAction":
            head_cmd.data = "SEARCHING"
            if self.world.hor_motor_out_of_center == HeadHorizontalPosition.LEFT:
                twist.angular.z = 0.4
            elif self.world.hor_motor_out_of_center == HeadHorizontalPosition.RIGHT:
                twist.angular.z = -0.4
            else:
                twist.angular.z = 0.4

        elif top_name == "GoToBallAction":
            head_cmd.data  = "TRACKING"
            twist.linear.x = 0.15
            if self.world.ball_position in (BallPosition.LEFT_TOP, BallPosition.LEFT_BOTTOM):
                twist.angular.z = 0.3
            elif self.world.ball_position in (BallPosition.RIGHT_TOP, BallPosition.RIGHT_BOTTOM):
                twist.angular.z = -0.3
            else:
                twist.angular.z = -0.8 * self.world.ball_angle

        elif top_name == "AlignTrunkAction":
            head_cmd.data   = "TRACKING"
            twist.linear.x  = 0.0
            twist.angular.z = 0.0

        elif top_name == "AlignHeadForKickAction":
            head_cmd.data   = "TRACKING"
            twist.linear.x  = 0.0
            twist.angular.z = 0.0

        elif top_name == "KickAction":
            head_cmd.data = "IDLE"
            if not self._kick_in_progress:
                self._kick_in_progress = True

        elif top_name in ("RecoveryAction", "StandUpAction"):
            head_cmd.data   = "IDLE"
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            if not self._getup_in_progress:
                self._request_getup()

        else:
            # Saiu da recuperação — libera a flag para o próximo evento de queda
            self._getup_in_progress = False

        # 4. Publica comandos
        self._cmd_vel_pub.publish(twist)
        self._head_pub.publish(head_cmd)

        # 5. Publica estado (compatibilidade com rotinas legadas + debug)
        legacy = CurrentStateMsg()
        legacy.current_state = action_name_to_legacy_state(top_name)
        self._state_pub.publish(legacy)

        dsd_msg = CurrentStateMsg()
        dsd_msg.current_state = top_name.lower()
        self._dsd_state_pub.publish(dsd_msg)

        # 6. Log throttled
        self.get_logger().info(
            f"[DSD] {' → '.join(e.name for e in self.dsd.stack)} | "
            f"v={twist.linear.x:.2f} w={twist.angular.z:.2f} head={head_cmd.data}",
            throttle_duration_sec=3.0,
        )

    # Levantamento

    def _request_getup(self) -> None:
        self._getup_in_progress = True
        page = get_getup_page(self.world.fall_state)
        self.get_logger().info(f"Levantamento solicitado: {page}")
        # _getup_in_progress permanece True até o robô voltar a FallState.UP
        # e RecoveryAction retornar "DONE", saindo da pilha da DSD.

    def _reset_getup(self) -> None:
        self._getup_in_progress = False


# Entry point

def main(args=None):
    rclpy.init(args=args)
    node = DsdNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()