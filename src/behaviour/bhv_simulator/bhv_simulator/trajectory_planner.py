"""ROS positions -> copied Theta* planner (when needed) -> velocities at 20 Hz."""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

from .trajectory_navigation import RouteTracker, ROBOT_RADIUS, MARGIN


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        radius = self.declare_parameter('robot_radius', ROBOT_RADIUS).value
        margin = self.declare_parameter('safety_margin', MARGIN).value
        self.speed = self.declare_parameter('max_linear_speed', 0.5).value
        if not all(math.isfinite(v) and v > 0 for v in (radius, margin, self.speed)):
            raise ValueError('Raio, margem e velocidade devem ser positivos e finitos.')
        self.tracker, self.inputs = RouteTracker(radius, margin), {}
        self.create_subscription(PoseStamped, '/simulation/robot_pose', lambda m: self.receive('robot', m), 1)
        self.create_subscription(PointStamped, '/simulation/ball_position', lambda m: self.receive('ball', m), 1)
        self.create_subscription(MarkerArray, '/simulation/obstacles', lambda m: self.receive('obstacles', m), 1)
        self.path_pub = self.create_publisher(Path, '/planning/path', 1)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_timer(0.05, self.update)  # ROS /clock: same time base as Webots.
        self.create_timer(0.1, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_path_publish = float('-inf')

    def receive(self, name, msg):
        self.inputs[name] = msg, time.monotonic()

    def watchdog(self):
        if len(self.inputs) != 3 or any(time.monotonic()-t > 1.0 for _, t in self.inputs.values()):
            self.tracker.clear()
            if rclpy.ok():
                self.velocity_pub.publish(Twist())
                self.path_pub.publish(Path())

    def update(self):
        if not rclpy.ok():
            return
        command, result, path = Twist(), Path(), []
        try:
            if len(self.inputs) != 3:
                return
            robot, ball, obs = (self.inputs[k][0] for k in ('robot', 'ball', 'obstacles'))
            result.header = robot.header
            now = self.get_clock().now().nanoseconds / 1e9
            headers = [robot.header, ball.header] + [m.header for m in obs.markers]
            if any(h.frame_id != robot.header.frame_id or
                   not -0.1 <= now - (h.stamp.sec+h.stamp.nanosec/1e9) <= 0.3 for h in headers):
                return
            if any(time.monotonic()-t > 1.0 for _, t in self.inputs.values()):
                return
            start, goal = (robot.pose.position.x, robot.pose.position.y), (ball.point.x, ball.point.y)
            obstacles = [dict(x=m.pose.position.x, y=m.pose.position.y, raio=max(m.scale.x, m.scale.y)/2)
                         for m in obs.markers]
            q = robot.pose.orientation
            if not all(math.isfinite(v) for v in (*start, *goal, q.x, q.y, q.z, q.w,
                                                  *(v for o in obstacles for v in o.values()))):
                return
            if any(o['raio'] <= 0 for o in obstacles):
                return
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            path = self.tracker.update(start, goal, obstacles, now)
            command.linear.x, command.angular.z = self.tracker.command(start, yaw, goal, path, obstacles, self.speed)
            if now < self.last_path_publish or now-self.last_path_publish >= 0.2 or not path:
                for x, y in path:
                    pose = PoseStamped()
                    pose.header = result.header
                    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                    pose.pose.orientation.w = 1.0
                    result.poses.append(pose)
                self.path_pub.publish(result)
                self.last_path_publish = now
        except ValueError as error:
            self.tracker.clear()
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
        finally:
            if rclpy.ok():
                if not path:
                    self.path_pub.publish(result)
                self.velocity_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.velocity_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
