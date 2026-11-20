"""Regression tests using real ROS messages and a simulated Supervisor API."""

import math
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Twist

from bhv_simulator.field_bridge import FieldBridge
from bhv_simulator.field_geometry import field_position, field_yaw, integrate_velocity


def orientation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [c, 0, s, 0, 1, 0, -s, 0, c]


@pytest.fixture
def bridge():
    node, supervisor = Mock(), Mock()
    node.declare_parameter.side_effect = lambda name, default: Mock(value=default)
    supervisor.getTime.return_value = 1.25
    ball, robot = Mock(), Mock()
    supervisor.getFromDef.side_effect = {'Ball': ball, 'Robot3D': robot}.get
    ball.getPosition.return_value = [2.0, 0.055, -3.0]
    robot.getPosition.return_value = [0.0, 0.25, 0.0]
    robot.getOrientation.return_value = orientation(0.0)
    fields = {'translation': Mock(), 'rotation': Mock()}
    fields['translation'].getSFVec3f.return_value = [0.0, 0.25, 0.0]
    robot.getField.side_effect = fields.get
    node.create_publisher.side_effect = lambda *args: Mock()
    return FieldBridge(node, supervisor)


def test_global_position_and_simulation_timestamp(bridge):
    bridge.publish()
    ball = bridge.ball_pub.publish.call_args.args[0]
    pose = bridge.pose_pub.publish.call_args.args[0]
    clock = bridge.clock_pub.publish.call_args.args[0]
    assert (ball.point.x, ball.point.y, ball.point.z) == (3, -2, 0.055)
    assert ball.header.frame_id == pose.header.frame_id == 'map'
    assert ball.header.stamp == pose.header.stamp == clock.clock
    assert (clock.clock.sec, clock.clock.nanosec) == (1, 250_000_000)
    assert pose.pose.position.z == 0
    assert pose.pose.orientation.w == 1
    # Moving the ball in Webots is observed on the next publication.
    bridge.ball.getPosition.return_value = [-1.0, 0.1, 2.0]
    bridge.publish()
    assert bridge.ball_pub.publish.call_args.args[0].point.x == -2


@pytest.mark.parametrize('yaw', [0, math.pi / 2, -math.pi / 2, math.pi])
def test_heading_and_forward_motion(yaw):
    assert field_yaw(orientation(yaw)) == pytest.approx(yaw)
    x, y, _ = integrate_velocity(0, 0, yaw, 1, 0, 0, 1)
    assert (x, y) == pytest.approx((math.cos(yaw), math.sin(yaw)))
    assert field_position([-y, 0.25, -x])[:2] == pytest.approx((x, y))


def test_body_lateral_and_circular_motion():
    assert integrate_velocity(0, 0, math.pi / 2, 0, 1, 0, 1)[:2] == pytest.approx((-1, 0))
    assert integrate_velocity(0, 0, 0, 1, 0, 1, math.pi / 2) == pytest.approx((1, 1, math.pi / 2))


def test_motion_timeout_and_invalid_command(bridge):
    bridge.move(0.1)
    bridge.translation.setSFVec3f.assert_not_called()
    command = Twist()
    command.linear.x = 1.0
    bridge.command(command)
    bridge.move(0.1)
    assert bridge.translation.setSFVec3f.call_args.args[0] == pytest.approx([0, 0.25, -0.1])
    bridge.translation.setSFVec3f.reset_mock()
    bridge.supervisor.getTime.return_value = 1.75
    bridge.move(0.1)
    bridge.translation.setSFVec3f.assert_not_called()
    command.linear.x = float('nan')
    bridge.command(command)
    bridge.move(0.1)
    bridge.translation.setSFVec3f.assert_not_called()


def test_missing_def_fails_at_startup():
    supervisor = Mock()
    supervisor.getFromDef.return_value = None
    with pytest.raises(RuntimeError, match='DEF Ball'):
        FieldBridge(Mock(), supervisor)


def test_obstacle_centres_and_radii_are_read_from_supervisor(bridge):
    obstacle = Mock()
    obstacle.getPosition.return_value = [-0.8, 0.35, -2.1]
    cylinder = obstacle.getField.return_value.getSFNode.return_value
    cylinder.getField.side_effect = lambda name: Mock(getSFFloat=lambda: {'radius': 0.2, 'height': 0.7}[name])
    bridge.obstacles = [obstacle]
    bridge.publish()
    marker = bridge.obstacles_pub.publish.call_args.args[0].markers[0]
    assert (marker.pose.position.x, marker.pose.position.y) == (2.1, 0.8)
    assert marker.scale.x == marker.scale.y == 0.4
    assert marker.scale.z == 0.7
