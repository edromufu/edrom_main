# object_finder_onnx/finder_onnx.py
# (Este código é o nosso script unificado, mas simplificado para usar APENAS ONNX)
# (As funções publish_results, setup_object, etc. são idênticas)
import rclpy, os, sys
from rclpy.node import Node
import time, cv2
from cv_bridge import CvBridge
import numpy as np
# Importa a inferencia deste pacote
import object_finder_onnx.running_inference_onnx as ri
# Importa as mensagens do outro pacote
from edrom_msgs.msg import VisionData, Detection, DetectionArray
from sensor_msgs.msg import Image as ROS_Image

class Visao(Node):
    def __init__(self,nome_no):
        super().__init__(nome_no)
        self.get_logger().info('Nó de Visão ONNX Iniciado')
        self.use_simulation = self.declare_parameter('use_simulation', False).get_parameter_value().bool_value
        self.output_img = self.declare_parameter('vision.img_output', True).get_parameter_value().bool_value
        self.model = ri.set_model_input() # Simplificado

        if self.use_simulation:
            self.get_logger().info('>> RODANDO ONNX EM MODO SIMULAÇÃO <<')
            self.bridge = CvBridge()
            self.processed_image_publisher = self.create_publisher(ROS_Image, 'processed_image_topic', 10)
            self.camera_subscriber = self.create_subscription(ROS_Image, '/camera/image', self.image_callback, 10)
        else:
            self.get_logger().info('>> RODANDO ONNX EM MODO REAL <<')
            self.publisher = self.create_publisher(VisionData, 'self_parameters_vision2BhvTopic', 100)
            self.camera_idx = self.declare_parameter('vision.camera_idx', 0).get_parameter_value().integer_value
            self.initialize_webcam_and_loop()

    def image_callback(self, ros_image_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image_msg, "bgr8")
            self.process_frame(frame)
        except Exception as e: self.get_logger().error(f'Falha na conversão: {e}')

    def initialize_webcam_and_loop(self):
        self.cap = cv2.VideoCapture(self.camera_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f"Câmera {self.camera_idx} não abriu"); rclpy.shutdown(); return
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret: continue
            self.process_frame(frame)
            if cv2.waitKey(1) == ord("q"):
                self.cap.release(); cv2.destroyAllWindows(); rclpy.shutdown(); break

    def process_frame(self, frame):
        self.classes, self.scores, self.boxes, self.inference_frame = ri.detect_model(self.model, frame) # Simplificado
        if self.use_simulation:
            processed_image_msg = self.bridge.cv2_to_imgmsg(self.inference_frame, "bgr8")
            self.processed_image_publisher.publish(processed_image_msg)
        else:
            self.publish_results()
        if self.output_img:
            cv2.imshow("Visao EDROM - ONNX", self.inference_frame)
            cv2.waitKey(1)

    def publish_results(self):
        # Esta função é idêntica à do outro pacote
        objects_msg = VisionData()
        # ... (lógica de preenchimento)
        self.publisher.publish(objects_msg)

    def setup_object(self, obj_data):
        # Idêntico
        obj = Detection(); [obj.found, obj.x, obj.y, obj.roi_width, obj.roi_height, _] = obj_data
        return obj

    def create_multi_objects(self,detection_list):
        # Idêntico
        multi_objects_msg = DetectionArray()
        multi_objects_msg.found = True
        multi_objects_msg.detections = [self.setup_object(det) for det in detection_list]
        return multi_objects_msg

def main(args=None):
    rclpy.init(args=args)
    no_visao = Visao('VisaoONNX')
    rclpy.spin(no_visao)
    no_visao.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
