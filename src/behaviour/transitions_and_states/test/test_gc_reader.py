"""Protocol fixtures and real local UDP/ROS delivery, without a GameController GUI."""
import socket
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from modularized_bhv_msgs.msg import GameControllerMsg

from transitions_and_states.gc_protocol import add_me, parse
from transitions_and_states.gc_reader import GcReader, to_message


def packet():
    # Independently assembled offsets from the PDF (18 + 70 + 70).
    data = bytearray(158)
    data[:4] = b'RGme'
    data[4:14] = bytes([20, 255, 2, 1, 0, 0, 3, 0, 1, 7])
    data[14:16] = (-15).to_bytes(2, 'little', signed=True)
    data[16:18] = (30).to_bytes(2, 'little', signed=True)
    data[18:24] = bytes([7, 0, 1, 1, 4, 0])
    data[26:28] = (1000).to_bytes(2, 'little')
    data[31:34] = bytes([12, 60, 2])  # player 2
    data[88:94] = bytes([9, 1, 0, 1, 3, 0])
    return data


def test_protocol_and_player_selection():
    state = parse(packet())
    assert state['state'] == 'PLAYING'
    assert state['secs_remaining'] == -15
    assert state['secondary_time'] == 30
    add_me(state, 7, 2)
    assert state['me']['penalty_name'] == 'SENT_OFF'
    assert state['me']['penalty'] == 12
    assert state['me']['cautions'] == 2
    assert state['me']['message_budget'] == 1000
    assert state['me']['has_kickoff'] is True
    assert (state['me']['score'], state['me']['opponent_score']) == (4, 3)
    add_me(state, 8, 1)
    assert state['me'] is None
    add_me(state, 7, 20)
    assert 'penalty' not in state['me']


@pytest.mark.parametrize('offset,value', [(0, 0), (4, 19), (6, 21), (6, 0)])
def test_reject_invalid_packets(offset, value):
    data = packet()
    data[offset] = value
    assert parse(data) is None
    assert parse(data[:-1]) is None
    assert parse(data + b'\0') is None


def test_unknown_enums_and_no_kickoff():
    data = packet()
    data[10] = 99
    data[13] = 255
    state = parse(data)
    assert state['state'] == '99'
    assert state['kicking_team'] is None


def test_udp_ros_timeout_and_reconnection():
    rclpy.init()
    reader = GcReader(parameter_overrides=[Parameter('port', value=0),
                                          Parameter('team_num', value=7),
                                          Parameter('player_num', value=2)])
    observer = rclpy.create_node('gc_reader_test_observer')
    messages = []
    observer.create_subscription(GameControllerMsg, '/gc/state', messages.append, 10)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = ('127.0.0.1', reader.rx.getsockname()[1])

    def wait_for(predicate, send=False):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if send:
                sender.sendto(packet(), destination)
            rclpy.spin_once(reader, timeout_sec=0.02)
            rclpy.spin_once(observer, timeout_sec=0.02)
            if predicate():
                return
        pytest.fail('Expected ROS message was not received')

    try:
        wait_for(lambda: any(m.connected and m.player_found and m.penalty == 12 for m in messages), True)
        messages.clear()
        # Drain any queued datagrams, then simulate the elapsed timeout.
        reader.receive()
        reader.last_rx = time.monotonic() - 3.1
        sender.sendto(b'invalid', destination)
        reader.receive()
        reader.check_connection()
        wait_for(lambda: any(not m.connected for m in messages))
        messages.clear()
        wait_for(lambda: any(m.connected for m in messages), True)
    finally:
        sender.close()
        observer.destroy_node()
        reader.destroy_node()
        rclpy.shutdown()


def test_typed_message_preserves_fields_and_absent_player():
    state = parse(packet())
    add_me(state, 7, 2)
    msg = to_message(state, 7, 2)
    assert msg.connected and msg.team_found and msg.player_found
    assert msg.state == 'PLAYING' and msg.penalty == 12
    assert msg.secs_remaining == -15 and msg.secondary_time == 30
    assert msg.teams[0].players[1].penalty_name == 'SENT_OFF'
    assert msg.score == 4 and msg.opponent_score == 3
    assert msg.message_budget == 1000 and msg.has_kickoff
    add_me(state, 8, 1)
    msg = to_message(state, 8, 1)
    assert not msg.team_found and not msg.player_found
    add_me(state, 7, 20)
    msg = to_message(state, 7, 20)
    assert msg.team_found and not msg.player_found
    state['kicking_team'] = None
    assert to_message(state, 7, 20).kicking_team == -1
    msg = to_message({'connected': False}, 7, 2)
    assert not msg.connected and not msg.player_found and msg.teams == []
