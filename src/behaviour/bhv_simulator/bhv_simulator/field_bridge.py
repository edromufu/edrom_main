"""Supervisor ground truth and kinematic motion for planner experiments."""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray

from .field_geometry import field_position, field_yaw, integrate_velocity


class FieldBridge:
    def __init__(self, node, supervisor):
        self.node = node
        self.supervisor = supervisor
        self.ball = supervisor.getFromDef('Ball')
        self.robot = supervisor.getFromDef('Robot3D')
        if self.ball is None or self.robot is None:
            raise RuntimeError('O mundo deve conter DEF Ball e DEF Robot3D.')
        self.obstacles = []
        index = 1
        while True:
            obstacle = supervisor.getFromDef(f'Obstacle{index}')
            if obstacle is None:
                break
            self.obstacles.append(obstacle)
            index += 1
        self.obstacles_pub = node.create_publisher(MarkerArray, '/simulation/obstacles', 1)
        self.translation = self.robot.getField('translation')
        self.rotation = self.robot.getField('rotation')
        self.frame = node.declare_parameter('field_frame', 'map').value
        self.timeout = node.declare_parameter('cmd_vel_timeout', 0.3).value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('cmd_vel_timeout deve ser positivo e finito.')
        self.velocity = (0.0, 0.0, 0.0)
        self.last_command = float('-inf')
        self.ball_pub = node.create_publisher(PointStamped, '/simulation/ball_position', 1)
        self.pose_pub = node.create_publisher(PoseStamped, '/simulation/robot_pose', 1)
        self.clock_pub = node.create_publisher(Clock, '/clock', 10)
        self.command_sub = node.create_subscription(Twist, '/cmd_vel', self.command, 1)

    def command(self, msg):
        velocity = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(value) for value in velocity):
            self.velocity = (0.0, 0.0, 0.0)
            self.node.get_logger().warning('cmd_vel não finito: movimento interrompido.')
            return
        self.velocity = velocity
        self.last_command = self.supervisor.getTime()

    def move(self, dt):
        now = self.supervisor.getTime()
        if not 0 <= now - self.last_command < self.timeout:
            return
        if not any(self.velocity):
            return
        x, y, _ = field_position(self.robot.getPosition())
        yaw = field_yaw(self.robot.getOrientation())
        x, y, yaw = integrate_velocity(x, y, yaw, *self.velocity, dt)
        # Robot3D is a top-level Transform; preserve its height above the floor.
        height = self.translation.getSFVec3f()[1]
        self.translation.setSFVec3f([-y, height, -x])
        self.rotation.setSFRotation([0.0, 1.0, 0.0, yaw])

    def publish(self):
        nanoseconds = round(self.supervisor.getTime() * 1_000_000_000)
        stamp = Time(sec=nanoseconds // 1_000_000_000,
                     nanosec=nanoseconds % 1_000_000_000)
        self.clock_pub.publish(Clock(clock=stamp))
        ball = PointStamped()
        ball.header.stamp = stamp
        ball.header.frame_id = self.frame
        ball.point.x, ball.point.y, ball.point.z = field_position(self.ball.getPosition())
        self.ball_pub.publish(ball)
        pose = PoseStamped()
        pose.header = ball.header
        pose.pose.position.x, pose.pose.position.y, _ = field_position(self.robot.getPosition())
        yaw = field_yaw(self.robot.getOrientation())
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        self.pose_pub.publish(pose)

        markers = MarkerArray()
        for index, obstacle in enumerate(self.obstacles):
            marker = Marker()
            marker.header = ball.header
            marker.ns, marker.id = 'obstacles', index
            marker.type, marker.action = Marker.CYLINDER, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = field_position(obstacle.getPosition())
            marker.pose.orientation.w = 1.0
            cylinder = obstacle.getField('boundingObject').getSFNode()
            marker.scale.x = marker.scale.y = 2 * cylinder.getField('radius').getSFFloat()
            marker.scale.z = cylinder.getField('height').getSFFloat()
            marker.color.r, marker.color.a = 1.0, 1.0
            markers.markers.append(marker)
        self.obstacles_pub.publish(markers)
