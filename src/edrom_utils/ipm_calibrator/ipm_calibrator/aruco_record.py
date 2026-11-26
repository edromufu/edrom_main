"""
Grava os ArUcos detectados em cada frame da câmera, junto com o acelerômetro do
IMU da cabeça e os ângulos comandados de head_pan / head_tilt.

Cada linha do arquivo .jsonl é um registro:
    {"tipo": "camera_info", "K": [...], "D": [...]}
    {"tipo": "frame", "stamp_ns": ..., "ids": [...], "cantos": [[[u, v] x 4], ...],
     "acc": [ax, ay, az] | null, "juntas": [pan, tilt] | null}

Os cantos estão em pixels, com distorção, na ordem do OpenCV. acc é a média das
últimas leituras do acelerômetro (m/s², referencial do IMU). juntas são os
últimos ângulos comandados em /goal_joint_states (rad).

varredura:=True comanda a cabeça por uma grade pan x tilt (robô na first pose,
modo POSE do botão 2). Em cada pose espera espera_s e
grava frames_por_pose frames; fora disso não grava nada.
"""

from collections import deque
from datetime import datetime
import json
from pathlib import Path

from ament_index_python.packages import get_package_prefix
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState


class ArucoRecord(Node):

    def __init__(self):
        super().__init__('aruco_record')
        dicionario = self.declare_parameter('dicionario', 'DICT_5X5_250').value
        pasta = self.declare_parameter('pasta_saida', '').value
        self.mostrar = self.declare_parameter('mostrar', True).value
        imu_topic = self.declare_parameter('imu_topic', '/imu/data_raw').value

        dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dicionario))
        if hasattr(cv2.aruco, 'ArucoDetector'):  # OpenCV >= 4.7
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detectar = cv2.aruco.ArucoDetector(dic, params).detectMarkers
        else:  # OpenCV 4.6
            params = cv2.aruco.DetectorParameters_create()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detectar = lambda img: cv2.aruco.detectMarkers(img, dic, parameters=params)

        if not pasta:
            # <ws>/install/ipm_calibrator -> <ws>/.../src/edrom_utils/ipm_calibrator/dados
            # O src pode estar em <ws>/src ou um nível abaixo (ex.: <ws>/visao-cbr26/src,
            # quando o colcon build é rodado na raiz do repositório).
            ws = Path(get_package_prefix('ipm_calibrator')).parents[1]
            pacote = Path('edrom_utils/ipm_calibrator')
            candidatos = [ws / 'src' / pacote, *sorted(ws.glob(f'*/src/{pacote}'))]
            fonte = next((c for c in candidatos if (c / 'package.xml').is_file()), candidatos[0])
            pasta = fonte / 'dados'
        Path(pasta).mkdir(parents=True, exist_ok=True)
        self.caminho = Path(pasta) / f'aruco_{datetime.now():%Y%m%d_%H%M%S}.jsonl'
        self.arquivo = open(self.caminho, 'w', buffering=1)  # grava linha a linha

        self.bridge = CvBridge()
        self.camera_info_gravado = False
        self.n_gravados = 0
        self.acc = deque(maxlen=20)
        self.juntas = None
        self.gravando = True
        self.frames_na_pose = 0

        self.create_subscription(CameraInfo, 'camera_info', self.ao_receber_camera_info, 10)
        self.create_subscription(
            Image, 'image_raw', self.ao_receber_imagem, qos_profile_sensor_data)
        self.create_subscription(Imu, imu_topic, self.ao_receber_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, '/goal_joint_states', self.ao_receber_juntas, 10)

        if self.declare_parameter('varredura', False).value:
            pans = self.declare_parameter('pan_poses', [-1.0, -0.5, 0.0, 0.5, 1.0]).value
            tilts = self.declare_parameter('tilt_poses', [-0.5, -0.25, 0.0, 0.25, 0.5]).value
            self.espera = float(self.declare_parameter('espera_s', 2.0).value)
            self.frames_por_pose = int(self.declare_parameter('frames_por_pose', 15).value)
            self.tempo_max = float(self.declare_parameter('tempo_max_pose_s', 8.0).value)
            self.poses = [(float(p), float(t)) for t in tilts for p in pans] + [(0.0, 0.0)]
            self.pose_idx = -1
            self.gravando = False
            self.pub = self.create_publisher(JointState, '/goal_joint_states', 10)
            self.create_timer(0.05, self.passo_varredura)
            self.proxima_pose()

        self.get_logger().info(f'Gravando em {self.caminho} (IMU em {imu_topic})')

    def gravar(self, registro):
        self.arquivo.write(json.dumps(registro) + '\n')

    def ao_receber_camera_info(self, msg):
        if not self.camera_info_gravado:
            self.gravar({'tipo': 'camera_info', 'K': list(msg.k), 'D': list(msg.d)})
            self.camera_info_gravado = True

    def ao_receber_imu(self, msg):
        a = msg.linear_acceleration
        self.acc.append((a.x, a.y, a.z))

    def ao_receber_juntas(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if 'head_pan' in pos and 'head_tilt' in pos:
            self.juntas = [float(pos['head_pan']), float(pos['head_tilt'])]

    # ---------------------------------------------------------------- varredura
    def proxima_pose(self):
        self.pose_idx += 1
        self.gravando = False
        self.frames_na_pose = 0
        self.t_pose = self.get_clock().now()
        if self.pose_idx < len(self.poses):
            self.juntas = list(self.poses[self.pose_idx])

    def passo_varredura(self):
        if self.pose_idx >= len(self.poses):
            return
        msg = JointState(name=['head_pan', 'head_tilt'], position=self.juntas)
        self.pub.publish(msg)

        if self.pose_idx == len(self.poses) - 1:  # última = volta ao centro
            self.get_logger().info('Varredura concluída. Ctrl+C para sair.')
            self.pose_idx += 1
            return

        t = (self.get_clock().now() - self.t_pose).nanoseconds * 1e-9
        if not self.gravando and t >= self.espera:
            self.gravando = True
            self.acc.clear()  # média do IMU só com a cabeça parada
            self.get_logger().info(
                f'Pose {self.pose_idx + 1}/{len(self.poses) - 1}: '
                f'pan={self.juntas[0]:+.2f} tilt={self.juntas[1]:+.2f}')
        if self.frames_na_pose >= self.frames_por_pose or t >= self.tempo_max:
            if self.frames_na_pose < self.frames_por_pose:
                self.get_logger().warn(f'Pose {self.pose_idx + 1}: poucos ArUcos, pulando')
            self.proxima_pose()

    # ------------------------------------------------------------------ imagem
    def ao_receber_imagem(self, msg):
        imagem = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cantos, ids, _ = self.detectar(cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY))

        if ids is not None and self.gravando:
            self.gravar({
                'tipo': 'frame',
                'stamp_ns': msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                'ids': ids.ravel().tolist(),
                'cantos': [c.reshape(4, 2).astype(float).round(3).tolist() for c in cantos],
                'acc': np.mean(self.acc, axis=0).round(5).tolist() if self.acc else None,
                'juntas': self.juntas,
            })
            self.n_gravados += 1
            self.frames_na_pose += 1

        if self.mostrar:
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(imagem, cantos, ids)
            texto = f'frames gravados: {self.n_gravados}' + ('' if self.acc else ' | SEM IMU')
            cv2.putText(imagem, texto, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('aruco_record', imagem)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = ArucoRecord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arquivo.close()
        print(f'{node.n_gravados} frames gravados em {node.caminho}')


if __name__ == '__main__':
    main()
