"""
Republica a imagem da câmera girada 180 graus (câmera montada de cabeça para baixo).

Assina `image_raw` e publica `image_rotated`, com o mesmo header.
Usado pelo camera_calibrator para mostrar a imagem em pé durante a calibração.
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageRotate(Node):

    def __init__(self):
        super().__init__('image_rotate')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'image_rotated', qos_profile_sensor_data)
        self.create_subscription(
            Image, 'image_raw', self.ao_receber_imagem, qos_profile_sensor_data)

    def ao_receber_imagem(self, msg):
        imagem = self.bridge.imgmsg_to_cv2(msg)
        girada = self.bridge.cv2_to_imgmsg(cv2.rotate(imagem, cv2.ROTATE_180), msg.encoding)
        girada.header = msg.header
        self.pub.publish(girada)


def main():
    rclpy.init()
    node = ImageRotate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
