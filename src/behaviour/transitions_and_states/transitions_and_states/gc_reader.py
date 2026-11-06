"""Receive GameController UDP broadcasts and publish typed ROS data on /gc/state."""
import socket
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from modularized_bhv_msgs.msg import (
    GameControllerMsg, GameControllerPlayer, GameControllerTeam,
)

from .gc_protocol import add_me, parse


def to_message(state, team_num, player_num):
    """Convert decoded wire data to ROS fields, without JSON serialization."""
    msg = GameControllerMsg(team_number=team_num, player_number=player_num)
    msg.connected = state['connected']
    if not msg.connected:
        return msg
    for field in ('packet', 'players_per_team', 'competition', 'stopped', 'phase',
                  'state', 'set_play', 'first_half', 'secs_remaining', 'secondary_time'):
        setattr(msg, field, state[field])
    msg.kicking_team = -1 if state['kicking_team'] is None else state['kicking_team']
    for team in state['teams']:
        fields = {key: value for key, value in team.items() if key != 'players'}
        msg.teams.append(GameControllerTeam(
            **fields, players=[GameControllerPlayer(**p) for p in team['players']]))
    me = state['me']
    msg.team_found = me is not None
    msg.player_found = me is not None and 'penalty' in me
    if me is not None:
        for field, value in me.items():
            setattr(msg, field, value)
    return msg


class GcReader(Node):
    def __init__(self, **kwargs):
        super().__init__('gc_reader', **kwargs)
        self.team_num = self.declare_parameter('team_num', 0).value
        self.player_num = self.declare_parameter('player_num', 1).value
        port = self.declare_parameter('port', 3838).value
        if not 0 <= self.team_num <= 254 or not 1 <= self.player_num <= 20:
            raise ValueError('team_num deve estar entre 0 e 254; player_num entre 1 e 20.')
        if not 0 <= port <= 65535:
            raise ValueError('Porta UDP inválida.')
        self.pub = self.create_publisher(GameControllerMsg, '/gc/state', 10)
        self.last_rx = None
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.rx.bind(('', port))
            self.rx.setblocking(False)
        except OSError:
            self.rx.close()
            raise
        # Wall time is required even when the simulation clock is paused.
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.receive, clock=clock)
        self.create_timer(0.5, self.check_connection, clock=clock)
        self.get_logger().info(
            f'GameController UDP {self.rx.getsockname()[1]} -> /gc/state; '
            f'time={self.team_num}, jogador={self.player_num}')

    def receive(self):
        # Bound work per callback so malformed traffic cannot starve the executor.
        for _ in range(64):
            try:
                data, _ = self.rx.recvfrom(4096)
            except BlockingIOError:
                return
            state = parse(data)
            if state is None:
                continue
            add_me(state, self.team_num, self.player_num)
            self.last_rx = time.monotonic()
            self.pub.publish(to_message(state, self.team_num, self.player_num))

    def check_connection(self):
        if self.last_rx is None or time.monotonic() - self.last_rx >= 3.0:
            self.pub.publish(to_message({'connected': False}, self.team_num, self.player_num))

    def destroy_node(self):
        self.rx.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GcReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
