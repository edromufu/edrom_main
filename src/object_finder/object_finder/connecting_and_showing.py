#!/usr/bin/env python3
# coding=utf-8

import rclpy
import os
import sys
from rclpy.node import Node
import time
import cv2
from cv_bridge import CvBridge
import numpy as np

import object_finder.running_inference as ri

# Importa tanto as mensagens customizadas quanto a de imagem padrão do ROS
from edrom_msgs.msg import VisionData, Detection, DetectionArray
from sensor_msgs.msg import Image as ROS_Image

sys.setrecursionlimit(100000)


class Visao(Node):

    def __init__(self, nome_no):
        super().__init__(nome_no)
        self.get_logger().info('Nó de Visão Unificado Iniciado')

        # --- Parâmetros e Modelo (comum a ambos os modos) ---
        self.output_img = self.declare_parameter('vision.img_output', True).get_parameter_value().bool_value
        self.use_simulation = self.declare_parameter('use_simulation', False).get_parameter_value().bool_value
        self.model = ri.set_model_input(is_simulation=self.use_simulation)
        self.searching = True
        self.current_frame = None

        # --- LÓGICA CONDICIONAL DE INICIALIZAÇÃO ---
        if self.use_simulation:
            self.get_logger().info('>> RODANDO EM MODO SIMULAÇÃO (Publicando Imagem Processada) <<')
            self.bridge = CvBridge()
            self.processed_image_publisher = self.create_publisher(ROS_Image, 'processed_image_topic', 10)
            self.camera_subscriber = self.create_subscription(
                ROS_Image, '/camera/image', self.image_callback, 10
            )
        else:
            self.get_logger().info('>> RODANDO EM MODO REAL (Publicando Mensagens de Dados) <<')
            self.publisher = self.create_publisher(VisionData, 'self_parameters_vision2BhvTopic', 100)

            self.camera_idx = self.declare_parameter('vision.camera_idx', 0).get_parameter_value().integer_value
            self.ajuste = self.declare_parameter('vision.ajuste', False).get_parameter_value().bool_value
            self.bright = self.declare_parameter('vision.brilho', 4).get_parameter_value().integer_value

            self.initialize_webcam_and_loop()

    def image_callback(self, ros_image_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding="bgr8")
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Falha na conversão da imagem: {e}')

    def initialize_webcam_and_loop(self):
        self.cap = cv2.VideoCapture(self.camera_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir a câmera no índice {self.camera_idx}"); rclpy.shutdown(); return

        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)
        if self.ajuste: self.ajuste_camera()  # A chamada está aqui

        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret: self.get_logger().warn("Falha ao capturar frame da webcam."); continue
            self.process_frame(frame)
            if cv2.waitKey(1) == ord("q"):
                self.cap.release(); cv2.destroyAllWindows()
                self.get_logger().warn('Tecla "q" pressionada. Encerrando.'); rclpy.shutdown(); break

    def process_frame(self, frame):
        self.current_frame = frame
        self.classes, self.scores, self.boxes, self.inference_frame = ri.detect_model(self.model, self.current_frame)

        if self.use_simulation:
            try:
                processed_image_msg = self.bridge.cv2_to_imgmsg(self.inference_frame, "bgr8")
                self.processed_image_publisher.publish(processed_image_msg)
            except Exception as e:
                self.get_logger().error(f'Falha ao publicar imagem processada: {e}')
        else:
            self.publish_results()

        if self.output_img:
            cv2.imshow("Visao EDROM", self.inference_frame)
            cv2.waitKey(1)

    def publish_results(self):
        """Publica a mensagem VisionData. Usado apenas no modo REAL."""
        objects_msg = VisionData()
        objects_msg.searching = self.searching
        objects_msg.fps = int(self.fps if hasattr(self, 'fps') else 0)

        # Listas para separar as detecções por classe
        ball_objects, robot_objects, right_goal_objects, left_goal_objects = [], [], [], []
        x_intersection_objects, l_intersection_objects, t_intersection_objects, center_objects = [], [], [], []

        # Itera sobre todas as detecções encontradas pelo modelo
        if hasattr(self, 'boxes') and self.boxes:
            for i in range(len(self.boxes)):
                # Cria uma lista com os dados da detecção atual
                results = [True, int(self.boxes[i][0]), int(self.boxes[i][1]), int(self.boxes[i][2]), int(self.boxes[i][3]), self.scores[i]]
                class_id = self.classes[i]

                # Adiciona a detecção na lista da sua respectiva classe
                if class_id == 0: ball_objects.append(results)
                elif class_id == 1: robot_objects.append(results)
                elif class_id == 2: right_goal_objects.append(results)
                elif class_id == 3: left_goal_objects.append(results)
                elif class_id == 4: l_intersection_objects.append(results)
                elif class_id == 5: t_intersection_objects.append(results)
                elif class_id == 6: x_intersection_objects.append(results)
                elif class_id == 7: center_objects.append(results)

        # Ordena as listas para pegar os objetos mais relevantes
        ball_objects.sort(key=lambda obj: obj[5], reverse=True)  # Maior confiança primeiro
        robot_objects.sort(key=lambda obj: obj[5], reverse=True)
        right_goal_objects.sort(key=lambda obj: obj[1])  # Menor X (mais à esquerda) primeiro
        left_goal_objects.sort(key=lambda obj: obj[1])

        # Preenche a mensagem com os objetos selecionados
        if ball_objects: objects_msg.ball = self.setup_object(ball_objects[0])
        if robot_objects: objects_msg.robot = self.setup_object(robot_objects[0])
        if right_goal_objects: objects_msg.rightgoal = self.setup_object(right_goal_objects[-1])  # Pega o mais à direita
        if left_goal_objects: objects_msg.leftgoal = self.setup_object(left_goal_objects[0])  # Pega o mais à esquerda

        # Preenche as listas de intersecções
        if x_intersection_objects: objects_msg.x_intersection = self.create_multi_objects(x_intersection_objects)
        if l_intersection_objects: objects_msg.l_intersection = self.create_multi_objects(l_intersection_objects)
        if t_intersection_objects: objects_msg.t_intersection = self.create_multi_objects(t_intersection_objects)
        if center_objects: objects_msg.center = self.create_multi_objects(center_objects)

        self.publisher.publish(objects_msg)

    def setup_object(self, obj_data):
        obj = Detection()
        # A detecção contém [found, x, y, w, h, score]
        [obj.found, obj.x, obj.y, obj.roi_width, obj.roi_height, _] = obj_data
        return obj

    def create_multi_objects(self, detection_list):
        multi_objects_msg = DetectionArray()
        multi_objects_msg.found = True

        detections = []
        for det_data in detection_list:
            # Cria um objeto Detection para cada item na lista
            det_obj = self.setup_object(det_data)
            detections.append(det_obj)

        # Atribui a lista de detecções ao campo 'detections' da mensagem
        multi_objects_msg.detections = detections
        return multi_objects_msg

    def ajuste_camera(self):
        """Permite o ajuste manual de brilho da câmera no modo real."""
        print("Ajuste de Brilho: '=' para aumentar, '-' para diminuir. 'w' para continuar.")
        while rclpy.ok():
            key = cv2.waitKey(1)
            if key == ord('w'):
                break

            # Atualiza o frame para visualização
            ret, frame = self.cap.read()
            if not ret: continue

            if key == ord('='):
                self.bright += 10
                self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)
            if key == ord('-'):
                self.bright -= 10
                self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)

            # Mostra o brilho atual no frame
            brilho_atual = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
            cv2.putText(frame, f'Brilho: {brilho_atual}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Ajuste de Brilho", frame)

        cv2.destroyWindow("Ajuste de Brilho")
        self.ajuste = False


def main(args=None):
    rclpy.init(args=args)
    no_visao = Visao('Visao')
    rclpy.spin(no_visao)
    no_visao.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
