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

from edrom_msgs.msg import VisionData, Detection, DetectionArray, Landmark, LandmarkArray
from sensor_msgs.msg import Image as ROS_Image
from std_msgs.msg import Header

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
            # ... (código do modo simulação permanece o mesmo) ...
            self.get_logger().info('>> RODANDO EM MODO SIMULAÇÃO (Publicando Imagem Processada) <<')
            self.bridge = CvBridge()
            self.processed_image_publisher = self.create_publisher(ROS_Image, 'processed_image_topic', 10)
            self.camera_subscriber = self.create_subscription(
                ROS_Image, '/camera/image', self.image_callback, 10
            )
        else:
            # ... (código do modo real permanece o mesmo) ...
            self.get_logger().info('>> RODANDO EM MODO REAL (Publicando Mensagens de Dados) <<')
            self.publisher = self.create_publisher(VisionData, 'self_parameters_vision2BhvTopic', 100)
            self.camera_idx = self.declare_parameter('vision.camera_idx', 0).get_parameter_value().integer_value
            self.ajuste = self.declare_parameter('vision.ajuste', False).get_parameter_value().bool_value
            self.bright = self.declare_parameter('vision.brilho', 4).get_parameter_value().integer_value
            self.initialize_webcam_and_loop()

        # --- ATUALIZADO: Carregamento da Matriz de IPM para Localização ---
        self.localization_publisher = self.create_publisher(LandmarkArray, 'vision/landmarks', 10)
        
        # --- MUDANÇA AQUI ---
        self.M_ipm = None # Renomeado de self.H para self.M_ipm para clareza
        ipm_matrix_path = "ipm_matrix.npy" # Carrega o arquivo que geramos
        try:
            self.M_ipm = np.load(ipm_matrix_path)
            self.get_logger().info(f"Matriz de IPM '{ipm_matrix_path}' carregada com sucesso!")
        except FileNotFoundError:
            self.get_logger().warn(f"ARQUIVO DE MATRIZ IPM '{ipm_matrix_path}' NÃO ENCONTRADO. Landmarks não serão publicados.")


    def image_callback(self, ros_image_msg):
        # ... (código permanece o mesmo) ...
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding="bgr8")
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Falha na conversão da imagem: {e}')

    def initialize_webcam_and_loop(self):
        # ... (código permanece o mesmo) ...
        self.cap = cv2.VideoCapture(self.camera_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir a câmera no índice {self.camera_idx}"); rclpy.shutdown(); return

        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)
        if self.ajuste: self.ajuste_camera()

        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret: self.get_logger().warn("Falha ao capturar frame da webcam."); continue
            self.process_frame(frame)
            if cv2.waitKey(1) == ord("q"):
                self.cap.release(); cv2.destroyAllWindows()
                self.get_logger().warn('Tecla "q" pressionada. Encerrando.'); rclpy.shutdown(); break

    def process_frame(self, frame):
        # ... (código permanece o mesmo) ...
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
            self.publish_localization_data()

        if self.output_img:
            cv2.imshow("Visao EDROM", self.inference_frame)
            cv2.waitKey(1)

    # --- ATUALIZADO: Função que converte pixel para coordenadas RELATIVAS AO ROBÔ ---
    def transform_pixel_to_world(self, bounding_box):
        # Usa a nova matriz M_ipm
        if self.M_ipm is None:
            return None, None

        x, y, w, h = bounding_box
        # O ponto de âncora na base do objeto é a melhor escolha
        anchor_pixel = (x + w / 2, y + h)
        pixel_coords = np.array([[anchor_pixel]], dtype=np.float32)
        
        # Usa a matriz de IPM para transformar o pixel em coordenadas do robô (em cm)
        robot_coords = cv2.perspectiveTransform(pixel_coords, self.M_ipm)
        
        # Interpreta o resultado do IPM: (distância para frente, distância para esquerda)
        forward_dist_cm = robot_coords[0][0][0]
        leftward_dist_cm = robot_coords[0][0][1]
        
        # Converte as coordenadas cartesianas (frente, lado) em polares (distância, ângulo)
        distance_cm = np.sqrt(forward_dist_cm**2 + leftward_dist_cm**2)
        angle_rad = np.arctan2(leftward_dist_cm, forward_dist_cm)
        
        return distance_cm, angle_rad

    # --- ATUALIZADO: Função de publicação agora usa a matriz correta ---
    def publish_localization_data(self):
        # Verifica se a matriz M_ipm foi carregada
        if self.M_ipm is None or not hasattr(self, 'boxes') or not self.boxes:
            return

        landmarks_msg = LandmarkArray()
        landmarks_msg.header = Header(stamp=self.get_clock().now().to_msg())
        
        detected_landmarks = []
        for i in range(len(self.boxes)):
            box = self.boxes[i]
            # A chamada para a função de transformação agora usa a lógica de IPM
            distance_cm, angle_rad = self.transform_pixel_to_world(box)
            
            if distance_cm is not None:
                landmark = Landmark()
                landmark.id = self.classes[i]
                landmark.distance_m = distance_cm / 100.0  # Converte cm para metros
                landmark.angle_rad = angle_rad
                detected_landmarks.append(landmark)

        landmarks_msg.landmarks = detected_landmarks
        self.localization_publisher.publish(landmarks_msg)

    def publish_results(self):
        # ... (esta função permanece exatamente a mesma) ...
        """Publica a mensagem VisionData. Usado apenas no modo REAL."""
        objects_msg = VisionData()
        objects_msg.searching = self.searching
        objects_msg.fps = int(self.fps if hasattr(self, 'fps') else 0)

        # Listas para separar as detecções por classe
        ball_objects, robot_objects, right_goal_objects, left_goal_objects = [], [], [], []
        x_intersection_objects, l_intersection_objects, t_intersection_objects, center_objects = [], [], [], []

        if hasattr(self, 'boxes') and self.boxes:
            for i in range(len(self.boxes)):
                results = [True, int(self.boxes[i][0]), int(self.boxes[i][1]), int(self.boxes[i][2]), int(self.boxes[i][3]), self.scores[i]]
                class_id = self.classes[i]
                if class_id == 0: ball_objects.append(results)
                elif class_id == 1: robot_objects.append(results)
                elif class_id == 2: right_goal_objects.append(results)
                elif class_id == 3: left_goal_objects.append(results)
                elif class_id == 4: l_intersection_objects.append(results)
                elif class_id == 5: t_intersection_objects.append(results)
                elif class_id == 6: x_intersection_objects.append(results)
                elif class_id == 7: center_objects.append(results)

        ball_objects.sort(key=lambda obj: obj[5], reverse=True)
        robot_objects.sort(key=lambda obj: obj[5], reverse=True)
        right_goal_objects.sort(key=lambda obj: obj[1])
        left_goal_objects.sort(key=lambda obj: obj[1])

        if ball_objects: objects_msg.ball = self.setup_object(ball_objects[0])
        if robot_objects: objects_msg.robot = self.setup_object(robot_objects[0])
        if right_goal_objects: objects_msg.rightgoal = self.setup_object(right_goal_objects[-1])
        if left_goal_objects: objects_msg.leftgoal = self.setup_object(left_goal_objects[0])

        if x_intersection_objects: objects_msg.x_intersection = self.create_multi_objects(x_intersection_objects)
        if l_intersection_objects: objects_msg.l_intersection = self.create_multi_objects(l_intersection_objects)
        if t_intersection_objects: objects_msg.t_intersection = self.create_multi_objects(t_intersection_objects)
        if center_objects: objects_msg.center = self.create_multi_objects(center_objects)

        self.publisher.publish(objects_msg)
        
    def setup_object(self, obj_data):
        # ... (esta função permanece exatamente a mesma) ...
        obj = Detection()
        [obj.found, obj.x, obj.y, obj.roi_width, obj.roi_height, _] = obj_data
        return obj

    def create_multi_objects(self, detection_list):
        # ... (esta função permanece exatamente a mesma) ...
        multi_objects_msg = DetectionArray()
        multi_objects_msg.found = True
        detections = []
        for det_data in detection_list:
            det_obj = self.setup_object(det_data)
            detections.append(det_obj)
        multi_objects_msg.detections = detections
        return multi_objects_msg

    def ajuste_camera(self):
        # ... (esta função permanece exatamente a mesma) ...
        """Permite o ajuste manual de brilho da câmera no modo real."""
        print("Ajuste de Brilho: '=' para aumentar, '-' para diminuir. 'w' para continuar.")
        while rclpy.ok():
            key = cv2.waitKey(1)
            if key == ord('w'): break
            ret, frame = self.cap.read()
            if not ret: continue
            if key == ord('='):
                self.bright += 10
                self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)
            if key == ord('-'):
                self.bright -= 10
                self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)
            brilho_atual = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
            cv2.putText(frame, f'Brilho: {brilho_atual}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Ajuste de Brilho", frame)
        cv2.destroyWindow("Ajuste de Brilho")
        self.ajuste = False

# ... (função main permanece a mesma) ...
def main(args=None):
    rclpy.init(args=args)
    no_visao = Visao('Visao')
    rclpy.spin(no_visao)
    no_visao.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()