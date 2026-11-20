#!/usr/bin/env python3
"""Run ROS callbacks and Supervisor updates on the Webots controller thread."""

import os

import rclpy
from rclpy.node import Node
from controller import Supervisor

from .field_bridge import FieldBridge
from .sensors_update import RobotSensors


class BhvIndependentSim(Node):
    def __init__(self):
        super().__init__('bhv_simulator_node')
        self.general_supervisor = Supervisor()
        self.timestep = int(self.general_supervisor.getBasicTimeStep())
        self.field_bridge = FieldBridge(self, self.general_supervisor)
        # Camera rendering is unnecessary for ground-truth trajectory tests.
        sensors = self.declare_parameter('enable_sensors', os.getenv('BHV_SIM_ENABLE_SENSORS', 'false').lower() == 'true').value
        self.robot_sensors = RobotSensors(self, self.general_supervisor) if sensors else None

    def start(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.field_bridge.move(self.timestep / 1000.0)
            if self.general_supervisor.step(self.timestep) == -1:
                break
            self.field_bridge.publish()
            if self.robot_sensors is not None:
                self.robot_sensors.callClock()


def main(args=None):
    rclpy.init(args=args)
    simulator = None
    try:
        simulator = BhvIndependentSim()
        simulator.start()
    except KeyboardInterrupt:
        pass
    finally:
        if simulator is not None:
            simulator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
