"""
Grava os ArUcos detectados em cada frame da câmera.

Cada linha do arquivo .jsonl é um registro:
    {"tipo": "camera_info", "K": [...], "D": [...]}
    {"tipo": "frame", "stamp_ns": ..., "ids": [...], "cantos": [[[u, v] x 4], ...]}

stamp_ns é o instante de captura da imagem (header.stamp). Os cantos estão em
pixels, com distorção, na ordem do OpenCV: superior esquerdo, superior direito,
inferior direito, inferior esquerdo.
"""

from datetime import datetime
import json
from pathlib import Path

from ament_index_python.packages import get_package_prefix
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

# Rotação da janela (graus, sentido horário) -> código do cv2.rotate
ROTACOES = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


class ArucoRecord(Node):

    def __init__(self):
        super().__init__('aruco_record')
        dicionario = self.declare_parameter('dicionario', 'DICT_5X5_250').value
        pasta = self.declare_parameter('pasta_saida', '').value
        self.mostrar = self.declare_parameter('mostrar', True).value
        # Só gira a janela (câmera montada girada). Os cantos gravados ficam
        # sempre em pixels da imagem crua, que é o que K e D descrevem.
        self.rotacao = int(self.declare_parameter('image_rotation', 0).value) % 360
        if self.rotacao not in (0, *ROTACOES):
            self.get_logger().error(
                f'image_rotation={self.rotacao} inválido (use 0, 90, 180 ou 270); usando 0')
            self.rotacao = 0

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

        self.create_subscription(CameraInfo, 'camera_info', self.ao_receber_camera_info, 10)
        self.create_subscription(
            Image, 'image_raw', self.ao_receber_imagem, qos_profile_sensor_data)
        self.get_logger().info(f'Gravando em {self.caminho}')

    def gravar(self, registro):
        self.arquivo.write(json.dumps(registro) + '\n')

    def ao_receber_camera_info(self, msg):
        if not self.camera_info_gravado:
            self.gravar({'tipo': 'camera_info', 'K': list(msg.k), 'D': list(msg.d)})
            self.camera_info_gravado = True

    def ao_receber_imagem(self, msg):
        imagem = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cantos, ids, _ = self.detectar(cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY))

        if ids is not None:
            self.gravar({
                'tipo': 'frame',
                'stamp_ns': msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                'ids': ids.ravel().tolist(),
                'cantos': [c.reshape(4, 2).astype(float).round(3).tolist() for c in cantos],
            })
            self.n_gravados += 1

        if self.mostrar:
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(imagem, cantos, ids)
            if self.rotacao:
                imagem = cv2.rotate(imagem, ROTACOES[self.rotacao])
            cv2.putText(imagem, f'frames gravados: {self.n_gravados}', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
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
