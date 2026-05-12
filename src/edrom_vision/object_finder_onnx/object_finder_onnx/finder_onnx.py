#!/usr/bin/env python3
# coding=utf-8

# object_finder_onnx/finder_onnx.py
# (Este código é o nosso script unificado, mas simplificado para usar APENAS ONNX)

import rclpy
import os
import sys
from rclpy.node import Node
import time
import cv2
from cv_bridge import CvBridge
import numpy as np

# Importa a inferencia deste pacote
import object_finder_onnx.running_inference_onnx as ri
# Importa as mensagens do outro pacote
from edrom_msgs.msg import VisionData, Detection, DetectionArray
from sensor_msgs.msg import Image as ROS_Image


class Visao(Node):
    def __init__(self, nome_no):
        super().__init__(nome_no)
        self.get_logger().info('Nó de Visão ONNX Iniciado')
        self.use_simulation = self.declare_parameter('use_simulation', False).get_parameter_value().bool_value
        self.output_img = self.declare_parameter('vision.img_output', True).get_parameter_value().bool_value
        self.model = ri.set_model_input()  # Simplificado

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
        except Exception as e:
            self.get_logger().error(f'Falha na conversão: {e}')

    def initialize_webcam_and_loop(self):
        self.cap = cv2.VideoCapture(self.camera_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f"Câmera {self.camera_idx} não abriu")
            rclpy.shutdown()
            return
            
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                continue
            self.process_frame(frame)
            if cv2.waitKey(1) == ord("q"):
                self.cap.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()
                break

    def process_frame(self, frame):
        # Mede o tempo para calcular o FPS
        start_time = time.time()

        self.classes, self.scores, self.boxes, self.inference_frame = ri.detect_model(self.model, frame)  # Simplificado
        
        # Calcula o FPS
        end_time = time.time()
        self.fps = 1 / (end_time - start_time)

        if self.use_simulation:
            processed_image_msg = self.bridge.cv2_to_imgmsg(self.inference_frame, "bgr8")
            self.processed_image_publisher.publish(processed_image_msg)
        else:
            self.publish_results()

        if self.output_img:
            cv2.imshow("Visao EDROM - ONNX", self.inference_frame)
            cv2.waitKey(1)

    def publish_results(self):
        """Publica a mensagem VisionData com a lógica de mapeamento de classes."""
        objects_msg = VisionData()
        objects_msg.searching = True
        objects_msg.fps = int(self.fps if hasattr(self, 'fps') else 0)

        ball_objects, goalpost_objects, robot_objects = [], [], []
        l_intersection_objects, t_intersection_objects, x_intersection_objects = [], [], []
        crossbar_objects = []

        if hasattr(self, 'boxes') and self.boxes:
            for i in range(len(self.boxes)):
                class_id = self.classes[i]
                score = self.scores[i]
                
                # Filtro de confiança específico para L-Intersection (ID 3)
                if class_id == 3 and score < 0.60:
                    continue

                results = [True, int(self.boxes[i][0]), int(self.boxes[i][1]), int(self.boxes[i][2]), int(self.boxes[i][3]), score]

                if class_id == 0: ball_objects.append(results)
                elif class_id == 1: goalpost_objects.append(results)
                elif class_id == 2: robot_objects.append(results)
                elif class_id == 3: l_intersection_objects.append(results)
                elif class_id == 4: t_intersection_objects.append(results)
                elif class_id == 5: x_intersection_objects.append(results)
                elif class_id == 6: crossbar_objects.append(results)

        if ball_objects:
            ball_objects.sort(key=lambda obj: obj[5], reverse=True)
            objects_msg.ball = self.setup_object(ball_objects[0])

        if robot_objects:
            robot_objects.sort(key=lambda obj: obj[5], reverse=True)
            objects_msg.robot = self.setup_object(robot_objects[0])

        if goalpost_objects:
            goalpost_objects.sort(key=lambda obj: obj[1])
            objects_msg.leftgoal = self.setup_object(goalpost_objects[0])
            if len(goalpost_objects) > 1:
                objects_msg.rightgoal = self.setup_object(goalpost_objects[-1])

        if x_intersection_objects: objects_msg.x_intersection = self.create_multi_objects(x_intersection_objects)
        if l_intersection_objects: objects_msg.l_intersection = self.create_multi_objects(l_intersection_objects)
        if t_intersection_objects: objects_msg.t_intersection = self.create_multi_objects(t_intersection_objects)

        self.publisher.publish(objects_msg)

    def setup_object(self, obj_data):
        obj = Detection()
        [obj.found, obj.x, obj.y, obj.roi_width, obj.roi_height, _] = obj_data
        return obj

    def create_multi_objects(self, detection_list):
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
