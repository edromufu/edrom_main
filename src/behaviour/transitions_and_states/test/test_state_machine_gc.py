"""Exercise the installed decision code and UDP -> ROS -> state-machine integration."""
import socket
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from modularized_bhv_msgs.msg import CurrentStateMsg, StateMachineMsg, GameControllerMsg

from transitions_and_states.gc_reader import GcReader
from transitions_and_states.state_machine import StateMachine
from transitions_and_states.state_machine_receiver import StateMachineReceiver
from test_gc_reader import packet


def update(machine, **changes):
    inputs = dict(ball_position='Center', ball_close=False, ball_found=True,
                  fall_state='Up', hor_motor_out_of_center='Center',
                  head_kick_check=False, kick_done=False)
    inputs.update(changes)
    return machine.request_state_machine_update(**inputs).current_state


def test_stabilization_walk_search_and_fall(monkeypatch):
    now = [100.0]
    monkeypatch.setattr('transitions_and_states.state_machine.time.time', lambda: now[0])
    machine = StateMachine()
    assert update(machine) == 'idle_march'
    now[0] += 1.1
    assert update(machine) == 'walking'
    assert update(machine, ball_found=False) == 'searching'
    assert update(machine, ball_found=False, fall_state='Front') == 'getting_up'


def test_kick_after_initial_stabilization(monkeypatch):
    now = [100.0]
    monkeypatch.setattr('transitions_and_states.state_machine.time.time', lambda: now[0])
    machine = StateMachine()
    close_ball = dict(ball_close=True, head_kick_check=True)
    assert update(machine, **close_ball) == 'idle_march'
    now[0] += 1.1
    assert update(machine, **close_ball) == 'kicking'


@pytest.mark.parametrize('connected,state,penalty,found,expected', [
    (False, 'SET', 12, False, 'walking'),
    (True, 'INITIAL', 0, True, 'idle'),
    (True, 'READY', 0, True, 'walking'),
    (True, 'SET', 0, True, 'idle'),
    (True, 'PLAYING', 0, True, 'walking'),
    (True, 'FINISHED', 0, True, 'idle'),
    (True, 'PLAYING', 12, True, 'idle'),
    (True, 'PLAYING', 0, False, 'idle'),
])
def test_game_controller_filters_decisions(connected, state, penalty, found, expected):
    machine = StateMachine()
    machine.enter_time -= 2
    gc = dict(gc_connected=connected, gc_state=state, gc_penalty=penalty,
              gc_player_found=found)
    assert update(machine, **gc) == expected
    assert update(machine, fall_state='Front', **gc) == 'getting_up'


def test_receiver_caches_gc_until_sensor_update():
    rclpy.init()
    receiver = StateMachineReceiver()
    try:
        receiver.game_controller_callback(GameControllerMsg(
            connected=True, state='SET', penalty=12, player_found=True))
        assert receiver.gc_state == 'SET' and receiver.gc_penalty == 12
        assert receiver.state_machine.gc_state == 'PLAYING'
        receiver.state_machine_callback(StateMachineMsg(
            ball_found=True, fall_state='Up', hor_motor_out_of_center='Center'))
        assert receiver.state_machine.gc_state == 'SET'
        assert receiver.state_machine.gc_penalty == 12
        assert receiver.state_machine.state == 'idle'
    finally:
        receiver.destroy_node()
        rclpy.shutdown()


def test_udp_to_decision_node_and_real_timeout():
    rclpy.init()
    reader = GcReader(parameter_overrides=[Parameter('port', value=0),
                                          Parameter('team_num', value=7),
                                          Parameter('player_num', value=2)])
    receiver = StateMachineReceiver()
    observer = rclpy.create_node('decision_integration_test')
    executor = SingleThreadedExecutor()
    for node in (reader, receiver, observer):
        executor.add_node(node)
    states = []
    observer.create_subscription(CurrentStateMsg, '/transitions_and_states/state_machine',
                                 lambda msg: states.append(msg.current_state), 10)
    sensors = observer.create_publisher(StateMachineMsg, receiver.parameters.stateMachineTopic, 10)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = ('127.0.0.1', reader.rx.getsockname()[1])

    def wait_for(predicate, action=None, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if action is not None:
                action()
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        pytest.fail('Integration condition not reached')

    try:
        data = packet()  # player 2 is penalized
        wait_for(lambda: receiver.gc_connected,
                 lambda: sender.sendto(data, destination))
        assert receiver.gc_penalty == 12
        msg = StateMachineMsg(ball_position='Center', ball_found=True, ball_close=False,
                              fall_state='Up', hor_motor_out_of_center='Center',
                              head_kick_check=False)
        receiver.state_machine.enter_time -= 2
        wait_for(lambda: 'idle' in states, lambda: sensors.publish(msg))
        assert receiver.state_machine.gc_penalty == 12

        # Remove the penalty: typed fields must reach the next decision update.
        data[31] = 0
        wait_for(lambda: receiver.gc_penalty == 0,
                 lambda: sender.sendto(data, destination))
        receiver.state_machine.enter_time -= 2
        states.clear()
        wait_for(lambda: 'walking' in states, lambda: sensors.publish(msg))

        data[10] = 2  # SET
        wait_for(lambda: receiver.gc_state == 'SET',
                 lambda: sender.sendto(data, destination))
        states.clear()
        wait_for(lambda: 'idle' in states, lambda: sensors.publish(msg))
        msg.fall_state = 'Front'
        wait_for(lambda: 'getting_up' in states, lambda: sensors.publish(msg))

        wait_for(lambda: not receiver.gc_connected)
        assert time.monotonic() - reader.last_rx >= 3.0
        msg.fall_state = 'Up'
        states.clear()
        wait_for(lambda: 'walking' in states, lambda: sensors.publish(msg))
        wait_for(lambda: receiver.gc_connected,
                 lambda: sender.sendto(data, destination))
    finally:
        sender.close()
        executor.shutdown()
        for node in (observer, receiver, reader):
            node.destroy_node()
        rclpy.shutdown()
