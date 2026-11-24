#!/usr/bin/env python3
"""Nó da IMU: recebe as amostras brutas do MPU-6050 (via edrom_buttond.py,
UDP local), roda TODOS os filtros sobre as mesmas amostras e publica.

Publica:
  /imu/data_raw              sensor_msgs/Imu  (corpo, SI, giro sem bias, sem orientação)
  /imu/<filtro>/data         sensor_msgs/Imu  com orientação (yaw = 0)
  /imu/<filtro>/rpy          geometry_msgs/Vector3Stamped, graus (x roll, y pitch)
  /behaviour/imu_accel|gyro|roll   tópicos antigos do imu_read (filtro escolhido)
  TF base_footprint -> camera_optical_frame (filtro escolhido), usado pelo soccer_ipm
  /imu2/data_raw, /imu2/<filtro>/data, /imu2/<filtro>/rpy   segundo MPU-6050 (0x69,
                             linhas "IMU2"), pipeline próprio, eixos em imu2_axes.
                             Tópicos antigos e TF continuam só com a IMU1.
Serviço:
  ~/recalibrate_gyro (std_srvs/Trigger): recalibra o bias (das duas IMUs) com o robô parado
"""
import socket
import threading
import time

from geometry_msgs.msg import TransformStamped, Vector3, Vector3Stamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from imu_ros_arduino.attitude import (
    DEFAULTS, DEG, EspClock, FILTER_NAMES, GRAVITY, ImuPipeline, parse_udp_line)
from imu_ros_arduino.camera_model import (
    CAMERA_DEFAULTS, CameraModel, matrix_to_quaternion, rpy_matrix)


def stamp_msg(ns):
    return Time(nanoseconds=int(ns)).to_msg()


class ImuFiltersNode(Node):

    def __init__(self):
        super().__init__('imu_filters')
        node_defaults = {
            'udp_host': '127.0.0.1',
            'udp_port': 5600,
            'filters': list(FILTER_NAMES),
            'filter': 'ekf',                 # alimenta tópicos antigos e o TF
            'publish_rate_hz': 100.0,
            'stamp_offset_ms': 0.0,          # soma no stamp (latência da IMU vs câmera)
            'publish_tf': True,
            'base_frame': 'base_footprint',
            'camera_frame': 'camera_optical_frame',
            'imu_frame': 'imu_link',
            'joint_states_topic': '/joint_states_feedback',
            'head_pan_joint': 'head_pan',
            'head_tilt_joint': 'head_tilt',
            'legacy_accel_topic': '/behaviour/imu_accel',
            'legacy_gyro_topic': '/behaviour/imu_gyro',
            'legacy_roll_topic': '/behaviour/imu_roll',
            'imu2_axes': list(DEFAULTS['axes']),   # 'axes' da IMU2 (0x69), mesmo formato
        }
        params = {}
        for name, default in {**DEFAULTS, **CAMERA_DEFAULTS, **node_defaults}.items():
            params[name] = self.declare_parameter(name, default).value
        self.p = params

        self.filter_names = [f for f in params['filters'] if f in FILTER_NAMES]
        self.selected = params['filter']
        if self.selected not in self.filter_names:
            raise SystemExit(f'filter={self.selected} não está em filters={self.filter_names}')

        self.pipeline = ImuPipeline(params, self.filter_names, log=self.get_logger().info)
        self.camera = CameraModel(params)
        self.clock_esp = EspClock()
        self.stamp_offset_ns = int(params['stamp_offset_ms'] * 1e6)
        self.pub_period_ns = int(1e9 / params['publish_rate_hz'])
        self.last_pub_ns = 0
        self.recalib = threading.Event()
        # IMU2 (0x69): pipeline próprio, mesmos filtros/calibração, eixos de imu2_axes
        log2 = self.get_logger().info
        self.pipeline2 = ImuPipeline({**params, 'axes': params['imu2_axes']}, self.filter_names,
                                     log=lambda m: log2(f'IMU2: {m}'))
        self.clock_esp2 = EspClock()
        self.last_pub_ns2 = 0
        self.recalib2 = threading.Event()

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.raw_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.filter_pubs = {
            n: (self.create_publisher(Imu, f'/imu/{n}/data', 10),
                self.create_publisher(Vector3Stamped, f'/imu/{n}/rpy', 10))
            for n in self.filter_names}
        self.raw_pub2 = self.create_publisher(Imu, '/imu2/data_raw', 10)
        self.filter_pubs2 = {
            n: (self.create_publisher(Imu, f'/imu2/{n}/data', 10),
                self.create_publisher(Vector3Stamped, f'/imu2/{n}/rpy', 10))
            for n in self.filter_names}
        self.accel_pub = self.create_publisher(Vector3, params['legacy_accel_topic'], sensor_qos)
        self.gyro_pub = self.create_publisher(Vector3, params['legacy_gyro_topic'], sensor_qos)
        self.roll_pub = self.create_publisher(Vector3, params['legacy_roll_topic'], sensor_qos)
        self.tf_pub = TransformBroadcaster(self) if params['publish_tf'] else None
        self.tf_per_filter = {n: self.create_publisher(TFMessage, f'/{n}/tf', 10)
                              for n in self.filter_names}

        self.pan = self.tilt = float('nan')
        if params['joint_states_topic']:
            self.create_subscription(JointState, params['joint_states_topic'], self.on_joints, 10)
        self.create_service(Trigger, '~/recalibrate_gyro', self.on_recalibrate)

        self.stats = {'n': 0, 'lost': 0, 'last_seq': None, 't0': time.time()}
        self.stats2 = {'n': 0, 'lost': 0, 'last_seq': None, 't0': time.time()}
        self.create_timer(10.0, self.log_stats)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # folga para pausas do Python (GIL/GC) sem perder amostras
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((params['udp_host'], int(params['udp_port'])))
        self.sock.settimeout(0.5)
        threading.Thread(target=self.rx_loop, daemon=True).start()
        self.get_logger().info(
            f'esperando amostras em udp://{params["udp_host"]}:{params["udp_port"]} '
            f'(filtros {self.filter_names}, escolhido: {self.selected}). '
            'Deixe o robô PARADO para calibrar o giro.')

    # ---------------------------------------------------------------- entradas
    def on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == self.p['head_pan_joint']:
                self.pan = pos
            elif name == self.p['head_tilt_joint']:
                self.tilt = pos

    def on_recalibrate(self, request, response):
        self.recalib.set()
        self.recalib2.set()
        response.success = True
        response.message = 'recalibrando: deixe o robô parado por ~2 s'
        return response

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

    # ---------------------------------------------------------- processamento
    def on_sample(self, idx, seq, t_us32, ax, ay, az, gx, gy, gz, pc_ns):
        # idx 0 = IMU (0x68), 1 = IMU2 (0x69); cada uma com seu estado
        if idx == 0:
            st, clock_esp, recalib, pipeline = (
                self.stats, self.clock_esp, self.recalib, self.pipeline)
        else:
            st, clock_esp, recalib, pipeline = (
                self.stats2, self.clock_esp2, self.recalib2, self.pipeline2)
        tag = '' if idx == 0 else 'IMU2: '
        if st['last_seq'] is not None:
            st['lost'] += (seq - st['last_seq'] - 1) % 65536
        st['last_seq'] = seq
        st['n'] += 1

        t_us, stamp_ns, restarted = clock_esp.update(t_us32, pc_ns)
        if restarted:
            self.get_logger().warn(f'{tag}ESP32 reiniciou (relógio voltou)')
        stamp_ns += self.stamp_offset_ns
        if recalib.is_set():
            recalib.clear()
            pipeline.recalibrate()
            self.get_logger().info(f'{tag}recalibrando o giro: deixe o robô parado')

        out = pipeline.process(t_us, (ax, ay, az), (gx, gy, gz))
        last_pub_ns = self.last_pub_ns if idx == 0 else self.last_pub_ns2
        if out is None or stamp_ns - last_pub_ns < self.pub_period_ns:
            return
        if idx == 0:
            self.last_pub_ns = stamp_ns
        else:
            self.last_pub_ns2 = stamp_ns
        self.publish(stamp_ns, *out, idx=idx)

    def publish(self, stamp_ns, a, g, dt, idx=0):
        if idx == 0:
            pipeline, raw_pub, filter_pubs = self.pipeline, self.raw_pub, self.filter_pubs
            frame = self.p['imu_frame']
        else:
            pipeline, raw_pub, filter_pubs = self.pipeline2, self.raw_pub2, self.filter_pubs2
            frame = 'imu2_link'
        stamp = stamp_msg(stamp_ns)
        acc = a * GRAVITY

        raw = Imu()
        raw.header.stamp = stamp
        raw.header.frame_id = frame
        raw.linear_acceleration = Vector3(x=acc[0], y=acc[1], z=acc[2])
        raw.angular_velocity = Vector3(x=g[0], y=g[1], z=g[2])
        raw.orientation_covariance[0] = -1.0
        raw_pub.publish(raw)

        angles = pipeline.angles()
        for name, (roll, pitch) in angles.items():
            imu_pub, rpy_pub = filter_pubs[name]
            msg = Imu()
            msg.header = raw.header
            msg.linear_acceleration = raw.linear_acceleration
            msg.angular_velocity = raw.angular_velocity
            q = matrix_to_quaternion(rpy_matrix(roll, pitch, 0.0))
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
            if name == 'ekf':
                sr, sp = pipeline.filters['ekf'].std()
                msg.orientation_covariance[0] = sr * sr
                msg.orientation_covariance[4] = sp * sp
            msg.orientation_covariance[8] = 1e6   # yaw não é estimado
            imu_pub.publish(msg)
            rpy = Vector3Stamped()
            rpy.header = raw.header
            rpy.vector = Vector3(x=roll / DEG, y=pitch / DEG, z=0.0)
            rpy_pub.publish(rpy)

        if idx != 0:   # IMU2: só /imu2/*; tópicos antigos e TF são da IMU1
            return

        # Tópicos antigos (imu_read): m/s², °/s e roll em graus no campo x
        roll, pitch = angles[self.selected]
        self.accel_pub.publish(Vector3(x=acc[0], y=acc[1], z=acc[2]))
        self.gyro_pub.publish(Vector3(x=g[0] / DEG, y=g[1] / DEG, z=g[2] / DEG))
        self.roll_pub.publish(Vector3(x=roll / DEG, y=pitch / DEG, z=0.0))

        if self.tf_pub:
            # Um TF por filtro em /<filtro>/tf (cada soccer_ipm lê o seu); o escolhido também vai em /tf
            for name, (r_, p_) in angles.items():
                R, t = self.camera.pose(r_, p_, self.pan, self.tilt)
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = self.p['base_frame']
                tf.child_frame_id = self.p['camera_frame']
                tf.transform.translation = Vector3(x=t[0], y=t[1], z=t[2])
                q = matrix_to_quaternion(R)
                r = tf.transform.rotation
                r.x, r.y, r.z, r.w = q
                self.tf_per_filter[name].publish(TFMessage(transforms=[tf]))
                if name == self.selected:
                    self.tf_pub.sendTransform(tf)

    def log_stats(self):
        st = self.stats
        el = time.time() - st['t0']
        if st['n'] == 0:
            self.get_logger().warn(
                'nenhuma amostra da IMU (edrom-buttons rodando? MPU ligado? veja '
                'journalctl -u edrom-buttons)')
        else:
            rate = st['n'] / el
            msg = f'{rate:.0f} amostras/s, {st["lost"]} perdidas'
            if not self.pipeline.calibrated:
                msg += ' (calibrando o giro: deixe o robô parado)'
            self.get_logger().info(msg)
        st.update(n=0, lost=0, t0=time.time())

        st = self.stats2
        el = time.time() - st['t0']
        if st['n'] == 0:
            self.get_logger().warn('IMU2: nenhuma amostra (MPU-6050 0x69 ligado? AD0 em 3V3?)')
        else:
            msg = f'IMU2: {st["n"] / el:.0f} amostras/s, {st["lost"]} perdidas'
            if not self.pipeline2.calibrated:
                msg += ' (calibrando o giro: deixe o robô parado)'
            self.get_logger().info(msg)
        st.update(n=0, lost=0, t0=time.time())

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuFiltersNode()
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
