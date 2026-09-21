"""Do not publish from the timer after Ctrl+C invalidates the ROS context."""
from types import SimpleNamespace
from unittest.mock import Mock

from bhv_simulator import trajectory_planner


def test_shutdown_does_not_publish(monkeypatch):
    monkeypatch.setattr(trajectory_planner.rclpy, 'ok', lambda: False)
    node = SimpleNamespace(inputs={}, path_pub=Mock(), velocity_pub=Mock())
    trajectory_planner.TrajectoryPlanner.update(node)
    node.path_pub.publish.assert_not_called()
    node.velocity_pub.publish.assert_not_called()
