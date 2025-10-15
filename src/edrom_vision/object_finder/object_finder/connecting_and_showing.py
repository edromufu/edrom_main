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

### CORREÇÃO: A importação foi movida para o topo do arquivo ###
from ament_index_python.packages import get_package_share_directory

sys.setrecursionlimit(100000)


class Visao(Node):

    def __init__(self, nome_no):
        super().__init__(nome_no)
        self.get_logger().info('Nó de Visão Unificado Iniciado')

        # Parâmetros e variáveis comuns no topo
        self.output_img = self.declare_parameter('vision.img_output', True).get_parameter_value().bool_value
        self.use_simulation = self.declare_parameter('use_simulation', False).get_parameter_value().bool_value
        self.model = ri.set_model_input(is_simulation=self.use_simulation)
        self.searching = True
        self.current_frame = None

        # Publishers comuns criados fora do if/else
        self.localization_publisher = self.create_publisher(LandmarkArray, 'vision/landmarks', 10)
        self.publisher = self.create_publisher(VisionData, 'vision2BhvTopic', 10)
        
        # if/else focado apenas no que é diferente
        if self.use_simulation:
            self.get_logger().info('>> RODANDO EM MODO SIMULAÇÃO <<')
            ipm_matrix_path = "ipm_matrix_sim.npy"
            self.bridge = CvBridge()
            self.processed_image_publisher = self.create_publisher(ROS_Image, 'processed_image_topic', 10) 
            self.camera_subscriber = self.create_subscription(
                ROS_Image, '/camera/image', self.image_callback, 10
            )
        else:
            self.get_logger().info('>> RODANDO EM MODO REAL <<')
            ipm_matrix_path = "ipm_matrix.npy"
            self.camera_idx = self.declare_parameter('vision.camera_idx', 0).get_parameter_value().integer_value
            self.ajuste = self.declare_parameter('vision.ajuste', False).get_parameter_value().bool_value
            self.bright = self.declare_parameter('vision.brilho', 4).get_parameter_value().integer_value
    
        # Carregamento da matriz agora fica em um lugar único
        self.M_ipm = None 
        try:
            package_share_path = get_package_share_directory('object_finder')
            full_path = os.path.join(package_share_path, ipm_matrix_path)
            self.M_ipm = np.load(full_path)
            self.get_logger().info(f"Matriz de IPM '{full_path}' carregada com sucesso!")
        except (FileNotFoundError, NameError):
            self.get_logger().warn(f"ARQUIVO DE MATRIZ IPM '{ipm_matrix_path}' NÃO ENCONTRADO. Landmarks não serão publicados.")
        
        if not self.use_simulation:
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
        if self.ajuste: self.ajuste_camera()
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
        
        self.publish_localization_data()

        if self.use_simulation:
            self.publish_results() 
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

    def transform_pixel_to_world(self, bounding_box):
        # Usa a nova matriz M_ipm
        if self.M_ipm is None:
            return None, None

        # A bounding_box está no formato (center_x, center_y, width, height)
        center_x, center_y, w, h = bounding_box
        
        # O ponto de âncora correto é o centro da base da caixa
        anchor_pixel_x = center_x
        anchor_pixel_y = center_y + (h / 2)
        
        anchor_pixel = (anchor_pixel_x, anchor_pixel_y)
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

    def publish_localization_data(self):
        if self.M_ipm is None or not hasattr(self, 'boxes') or not self.boxes: return
        landmarks_msg = LandmarkArray()
        landmarks_msg.header = Header(stamp=self.get_clock().now().to_msg())
        detected_landmarks = []
        for i in range(len(self.boxes)):
            box = self.boxes[i]
            distance_cm, angle_rad = self.transform_pixel_to_world(box)
            if distance_cm is not None:
                landmark = Landmark()
                landmark.id = int(self.classes[i])
                dist_m = distance_cm / 100.0    

                landmark.distance_m = float(dist_m)
                landmark.angle_rad = float(angle_rad)
    
                detected_landmarks.append(landmark)
                if self.output_img:
                    angle_deg = np.rad2deg(angle_rad)
                    text = f"D:{dist_m:.2f}m A:{angle_deg:.1f}d"
                    box_x_center, box_y_center, box_w, box_h = box
                    text_pos = (int(box_x_center - box_w/2), int(box_y_center - box_h/2 - 10))
                    cv2.putText(self.inference_frame, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        landmarks_msg.landmarks = detected_landmarks
        self.localization_publisher.publish(landmarks_msg)

    def publish_results(self):
        objects_msg = VisionData()
        objects_msg.searching = self.searching
        objects_msg.fps = int(self.fps if hasattr(self, 'fps') else 0)
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
        obj = Detection()
        [obj.found, obj.x, obj.y, obj.roi_width, obj.roi_height, _] = obj_data
        return obj

    def create_multi_objects(self, detection_list):
        multi_objects_msg = DetectionArray()
        multi_objects_msg.found = True
        detections = []
        for det_data in detection_list:
            det_obj = self.setup_object(det_data)
            detections.append(det_obj)
        multi_objects_msg.detections = detections
        return multi_objects_msg

    def ajuste_camera(self):
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

def main(args=None):
    rclpy.init(args=args)
    ### REMOVIDO: A importação daqui foi movida para o topo ###
    no_visao = Visao('Visao')
    rclpy.spin(no_visao)
    no_visao.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()