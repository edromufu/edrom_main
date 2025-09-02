#!/usr/bin/env python3

# coding=utf-8

import rclpy
from rclpy.node import Node
import time
import cv2
from cv_bridge import CvBridge
import numpy as np

import object_finder.running_inference as ri

from sensor_msgs.msg import Image as ROS_Image
from vision_msgs.msg import Detection2DArray
# Importe sua mensagem personalizada se ainda precisar dela
# from object_finder.msg import Webotsmsg

class Visao(Node):

    def __init__(self, nome_no):
        super().__init__(nome_no)
        self.get_logger().info('Nó de visão iniciado. Aguardando frames da câmera...')

        self.bridge = CvBridge()
        self.model = ri.set_model_input()
        
        # Subscriber para o tópico de imagem do Webots.
        # Substitua 'AUREA' e 'CAM' pelos nomes corretos do seu robô e câmera, se necessário.
        self.camera_subscriber = self.create_subscription(
            ROS_Image,
            '/camera/image',
            self.image_callback,
            10
        )
        self.get_logger().info('Inscrito no tópico de imagem: /camera/image')

        # Publisher para a imagem processada com as bounding boxes.
        self.processed_image_publisher = self.create_publisher(
            ROS_Image,
            'processed_image_topic',
            10
        )
        self.get_logger().info('Publicando imagem processada em: processed_image_topic')
    
    def image_callback(self, ros_image_msg):
        start_time = time.time()
        
        try:
            current_frame = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f'Falha na conversão da imagem: {e}')
            return

        # Roda a inferência. O 'running_inference.py' já se encarrega do redimensionamento.
        classes, scores, boxes, inference_frame = ri.detect_model(self.model, current_frame)
        
        finish_time = time.time()
        fps = 1 / (finish_time - start_time)
        
        self.get_logger().info(f'FPS total: {fps:.2f}')
        
        # Publica a imagem com as caixas de detecção.
        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(inference_frame, "bgr8")
            self.processed_image_publisher.publish(processed_image_msg)
        except Exception as e:
            self.get_logger().error(f'Falha ao publicar imagem processada: {e}')

def main(args=None):
    rclpy.init(args=args)
    no_visao = Visao('vision_node')
    rclpy.spin(no_visao)
    no_visao.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
