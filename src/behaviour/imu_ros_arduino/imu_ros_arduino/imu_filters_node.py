#!/usr/bin/env python3
"""Uma IMU (MPU-6050): amostras brutas por UDP (edrom_buttond.py) -> EKF -> ROS.

Rode uma instância por IMU, cada uma na sua porta (veja imu_filters.yaml):
  role:=head    roll/pitch da cabeça + yaw = pan da junta. Publica
                  /imu/head (sensor_msgs/Imu, orientação da cabeça)
                  TF base_footprint -> camera_optical_frame (para o soccer_ipm)
  role:=torso   tópicos lidos pelo fall_interpreter:
                  /behaviour/imu_accel (m/s²), /behaviour/imu_gyro (°/s),
                  /behaviour/imu_roll (graus: x roll, y pitch)
                tópicos do controlador de torso do aurea_walk:
                  /imu/ekf/rpy (Vector3Stamped, graus: x roll, y pitch)
                  /imu/data_raw (sensor_msgs/Imu, giro em rad/s)

Deixe a IMU parada nos primeiros ~2 s: é quando o bias do giro é calibrado.
"""
import socket
import threading
import time

from geometry_msgs.msg import TransformStamped, Vector3, Vector3Stamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from imu_ros_arduino.attitude import DEG, GRAVITY, EspClock, ImuPipeline, parse_udp_line
from imu_ros_arduino.camera_model import (
    CAMERA_DEFAULTS, CameraModel, matrix_to_quaternion, rpy_matrix)

PUBLISH_PERIOD_NS = 10_000_000          # 100 Hz
BASE_FRAME = 'base_footprint'
CAMERA_FRAME = 'camera_optical_frame'   # frame_id do usb_cam.yaml


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_filters')
        p = self.declare_parameter
        self.role = p('role', 'torso').value
        if self.role not in ('head', 'torso'):
            raise SystemExit("role deve ser 'head' ou 'torso'")
        port = p('udp_port', 5600).value
        self.stamp_offset_ns = int(p('stamp_offset_ms', 0.0).value * 1e6)
        self.pipeline = ImuPipeline(p('axes', ['x', 'y', 'z']).value, log=self.get_logger().info)
        self.clock_esp = EspClock()
        self.last_pub_ns = 0

        if self.role == 'head':
            self.pan_joint = p('pan_joint', 'head_pan').value
            self.camera = CameraModel({k: p(k, v).value for k, v in CAMERA_DEFAULTS.items()})
            self.pan = float('nan')
            self.create_subscription(JointState, p('joint_states_topic', '/joint_states_feedback').value,
                                     self.on_joints, 10)
            self.head_pub = self.create_publisher(Imu, '/imu/head', 10)
            self.tf_pub = TransformBroadcaster(self)
        else:
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
            self.accel_pub = self.create_publisher(Vector3, '/behaviour/imu_accel', qos)
            self.gyro_pub = self.create_publisher(Vector3, '/behaviour/imu_gyro', qos)
            self.roll_pub = self.create_publisher(Vector3, '/behaviour/imu_roll', qos)
            # controlador de torso (walking_engine_node, SensorDataQoS)
            self.rpy_pub = self.create_publisher(
                Vector3Stamped, p('rpy_topic', '/imu/ekf/rpy').value, qos_profile_sensor_data)
            self.imu_pub = self.create_publisher(
                Imu, p('imu_topic', '/imu/data_raw').value, qos_profile_sensor_data)

        self.stats = {'n': 0, 'lost': 0, 'last_seq': None, 't0': time.time()}
        self.create_timer(10.0, self.log_stats)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)   # folga para pausas do Python
        self.sock.bind(('127.0.0.1', int(port)))
        self.sock.settimeout(0.5)
        threading.Thread(target=self.rx_loop, daemon=True).start()
        self.get_logger().info(f'IMU {self.role}: esperando amostras em udp://127.0.0.1:{port}. '
                               'Deixe a IMU PARADA para calibrar o giro.')

    def on_joints(self, msg):
        if self.pan_joint in msg.name:
            self.pan = msg.position[msg.name.index(self.pan_joint)]

    def rx_loop(self):
        while rclpy.ok():
            try:
                data = self.sock.recv(256)
            except socket.timeout:
                continue
            except OSError:
                return
            sample = parse_udp_line(data)
            if sample is None:
                continue
            try:
                self.on_sample(*sample)
            except Exception as e:   # não deixa a thread morrer por uma amostra
                self.get_logger().error(f'erro processando amostra: {e!r}', throttle_duration_sec=5.0)

    def on_sample(self, seq, t_us32, ax, ay, az, gx, gy, gz, pc_ns):
        st = self.stats
        if st['last_seq'] is not None:
            st['lost'] += (seq - st['last_seq'] - 1) % 65536
        st['last_seq'] = seq
        st['n'] += 1

        t_us, stamp_ns, restarted = self.clock_esp.update(t_us32, pc_ns)
        if restarted:
            self.get_logger().warn('ESP32 reiniciou (relógio voltou)')
        stamp_ns += self.stamp_offset_ns

        out = self.pipeline.process(t_us, (ax, ay, az), (gx, gy, gz))
        if out is None or stamp_ns - self.last_pub_ns < PUBLISH_PERIOD_NS:
            return
        self.last_pub_ns = stamp_ns
        stamp = Time(nanoseconds=int(stamp_ns)).to_msg()
        if self.role == 'head':
            self.publish_head(stamp, *out)
        else:
            self.publish_torso(stamp, *out)

    def publish_head(self, stamp, a, g):
        ekf = self.pipeline.ekf
        roll, pitch = ekf.roll, ekf.pitch

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_head_link'
        q = matrix_to_quaternion(rpy_matrix(roll, pitch, self.camera.head_yaw(self.pan)))
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        sr, sp = ekf.std()
        msg.orientation_covariance[0] = sr * sr
        msg.orientation_covariance[4] = sp * sp
        msg.orientation_covariance[8] = 1e-4          # yaw vem do encoder do pan
        msg.angular_velocity = Vector3(x=g[0], y=g[1], z=g[2])
        acc = a * GRAVITY
        msg.linear_acceleration = Vector3(x=acc[0], y=acc[1], z=acc[2])
        self.head_pub.publish(msg)

        R, t = self.camera.pose(roll, pitch, self.pan)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = BASE_FRAME
        tf.child_frame_id = CAMERA_FRAME
        tf.transform.translation = Vector3(x=t[0], y=t[1], z=t[2])
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = matrix_to_quaternion(R)
        self.tf_pub.sendTransform(tf)

    def publish_torso(self, stamp, a, g):
        ekf = self.pipeline.ekf
        acc = a * GRAVITY
        rpy_deg = Vector3(x=ekf.roll / DEG, y=ekf.pitch / DEG, z=0.0)
        self.accel_pub.publish(Vector3(x=acc[0], y=acc[1], z=acc[2]))
        self.gyro_pub.publish(Vector3(x=g[0] / DEG, y=g[1] / DEG, z=g[2] / DEG))
        self.roll_pub.publish(rpy_deg)

        rpy = Vector3Stamped()
        rpy.header.stamp = stamp
        rpy.header.frame_id = 'imu_torso_link'
        rpy.vector = rpy_deg
        self.rpy_pub.publish(rpy)

        msg = Imu()
        msg.header = rpy.header
        msg.orientation_covariance[0] = -1.0          # orientação vai em /imu/ekf/rpy
        msg.angular_velocity = Vector3(x=g[0], y=g[1], z=g[2])
        msg.linear_acceleration = Vector3(x=acc[0], y=acc[1], z=acc[2])
        self.imu_pub.publish(msg)

    def log_stats(self):
        st = self.stats
        if st['n'] == 0:
            self.get_logger().warn(f'IMU {self.role}: nenhuma amostra (edrom-buttons rodando? '
                                   'MPU ligado? veja journalctl -u edrom-buttons)')
        else:
            msg = f'IMU {self.role}: {st["n"] / (time.time() - st["t0"]):.0f} amostras/s, {st["lost"]} perdidas'
            if not self.pipeline.calibrated:
                msg += ' (calibrando o giro: deixe a IMU parada)'
            self.get_logger().info(msg)
        st.update(n=0, lost=0, t0=time.time())

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
