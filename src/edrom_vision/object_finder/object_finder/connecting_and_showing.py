#!/usr/bin/env python3
# coding=utf-8

import rclpy
import os
import sys
import time
import cv2
import threading # ### OTMIZAÇÃO ###: Necessário para a trava (lock)
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import object_finder.running_inference as ri
from edrom_msgs.msg import VisionData, Detection, DetectionArray
from sensor_msgs.msg import Image as ROS_Image


class Visao(Node):

    def __init__(self, nome_no):
        super().__init__(nome_no)
        self.get_logger().info('Nó de Visão Unificado Iniciado')

        # --- PARÂMETROS GERAIS ---
        self.output_img = self.declare_parameter('vision.img_output', True).get_parameter_value().bool_value
        self.use_simulation = self.declare_parameter('use_simulation', False).get_parameter_value().bool_value
        self.use_calibration = self.declare_parameter('vision.use_calibration', True).get_parameter_value().bool_value
        self.model = ri.set_model_input(is_simulation=self.use_simulation)  
        self.searching = True
        self.fps = 0.0

        # --- OTMIZAÇÃO (Produtor-Consumidor) ---
        self.current_frame = None # Frame mais recente da câmera
        self.capture_lock = threading.Lock() # Trava para acesso seguro ao self.current_frame

        # --- PUBLICADORES ---
        self.publisher = self.create_publisher(VisionData, 'vision2BhvTopic', 100)

        # --- VARIÁVEIS DE CALIBRAÇÃO ---
        self.camera_matrix = None
        self.dist_coeffs = None
        self.map1 = None
        self.map2 = None
        self.new_camera_matrix = None
        self.roi = None # Embora não seja usado para recortar, é retornado pela função

        # --- CONFIGURAÇÃO DE AMBIENTE ---
        if self.use_simulation:
            self.get_logger().info('>> RODANDO EM MODO SIMULAÇÃO <<')
            self.bridge = CvBridge()
            self.processed_image_publisher = self.create_publisher(ROS_Image, 'vision_debug', 10)
            self.camera_subscriber = self.create_subscription(
                ROS_Image, '/camera/image', self.image_callback, 10
            )
        else:
            self.get_logger().info('>> RODANDO EM MODO REAL <<')
            self.camera_idx = self.declare_parameter('vision.camera_idx', 0).get_parameter_value().integer_value
            self.ajuste = self.declare_parameter('vision.ajuste', False).get_parameter_value().bool_value
            self.bright = self.declare_parameter('vision.brilho', 4).get_parameter_value().integer_value

            # Caminho da calibração
            if self.use_calibration:
                calib_dir_param = self.declare_parameter('vision.calibration_path', '').get_parameter_value().string_value
                if calib_dir_param:
                    calib_dir = calib_dir_param
                else:
                    package_share_path = get_package_share_directory('object_finder')
                    calib_dir = os.path.join(package_share_path, 'resource')
                self.load_calibration_files(calib_dir)
            else:
                self.get_logger().warn("⚠️ Calibração desativada por parâmetro (vision.use_calibration = False).")

            # --- INICIALIZA CÂMERA E PROCESSAMENTO (Produtor-Consumidor) ---
            self.initialize_webcam() # Dispara a thread de captura (Produtor)
            
            # ### OTMIZAÇÃO ###: Cria um timer para o loop de processamento (Consumidor)
            # Ajuste o FPS de processamento desejado (ex: 30 FPS = 1.0/30.0)
            # O processamento rodará o mais rápido que puder, mas o timer garante
            # que ele não "roube" todo o tempo de execução do ROS.
            # Um valor baixo (ex: 1.0/100.0) tentará rodar o mais rápido possível.
            processing_fps = 60.0 
            self.processing_timer = self.create_timer(1.0 / processing_fps, self.processing_loop)

    # =============================================================
    # ==================== MÉTODOS DE CALIBRAÇÃO ==================
    # =============================================================
    def load_calibration_files(self, calib_dir):
        try:
            cam_matrix_full_path = os.path.join(calib_dir, 'camera_matrix.npy')
            self.camera_matrix = np.load(cam_matrix_full_path)
            self.get_logger().info(f"Matriz da câmera carregada de '{cam_matrix_full_path}'")
        except FileNotFoundError:
            self.get_logger().warn("ARQUIVO 'camera_matrix.npy' NÃO ENCONTRADO. A correção de distorção não será aplicada.")

        try:
            dist_coeffs_full_path = os.path.join(calib_dir, 'dist_coeffs.npy')
            self.dist_coeffs = np.load(dist_coeffs_full_path)
            self.get_logger().info(f"Coeficientes de distorção carregados de '{dist_coeffs_full_path}'")
        except FileNotFoundError:
            self.get_logger().warn("ARQUIVO 'dist_coeffs.npy' NÃO ENCONTRADO. A correção de distorção não será aplicada.")

    def precompute_undistortion_maps(self, frame_shape):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return
        h, w = frame_shape[:2]
        
        # ### OTMIZAÇÃO ###: alpha=0 para "dar zoom" e remover bordas pretas
        # Isso elimina a necessidade de recortar (crop) a imagem depois.
        self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 0, (w, h) # '1' mudou para '0'
        )
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix, (w, h), cv2.CV_16SC2
        )
        self.get_logger().info("Mapas de correção (alpha=0) pré-calculados com sucesso!")

    # =============================================================
    # ====================== CAPTURA DE IMAGEM ====================
    # =============================================================
    
    def image_callback(self, ros_image_msg):
        """ Callback da Simulação: já funciona como um loop de processamento """
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding="bgr8")
            self.process_frame(frame) # Processa o frame recebido
        except Exception as e:
            self.get_logger().error(f'Falha na conversão da imagem: {e}')

    def initialize_webcam(self):
        """ (Modo Real) Inicializa a câmera e dispara a thread de captura """
        self.cap = cv2.VideoCapture(self.camera_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir a câmera no índice {self.camera_idx}")
            rclpy.shutdown()
            return

        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.bright)

        # Lemos um frame apenas para obter as dimensões da imagem
        ret, frame_teste = self.cap.read()
        if not ret:
            self.get_logger().error("Não foi possível ler o primeiro frame da câmera para calibração.")
            rclpy.shutdown()
            return
            
        # Pré-calcula os mapas agora, se a calibração estiver ativa
        if self.use_calibration:
            self.precompute_undistortion_maps(frame_teste.shape)

        # Agora chamamos o ajuste (que usará os mapas pré-calculados)
        if self.ajuste:
            self.ajuste_camera()

        # Inicia thread da câmera (Produtor)
        thread = threading.Thread(target=self._capture_loop, daemon=True)
        thread.start()
        self.get_logger().info("Thread de captura da câmera (Produtor) iniciada.")

    def _capture_loop(self):
        """ (Modo Real - Thread Produtor) Loop de captura da câmera """
        prev_time = time.time()
        
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Falha ao capturar frame da webcam.")
                continue

            # ### OTMIZAÇÃO ###: Armazena o frame mais recente de forma segura
            with self.capture_lock:
                self.current_frame = frame

            # Calcula FPS da CÂMERA (não do processamento)
            current_time = time.time()
            self.fps = 1.0 / (current_time - prev_time)
            prev_time = current_time

            # self.process_frame(frame) # <-- REMOVIDO DAQUI
            # O cv2.waitKey() e 'q' são removidos daqui

    # =============================================================
    # ===================== PROCESSAMENTO =========================
    # =============================================================

    def processing_loop(self):
        """ (Modo Real - Loop Consumidor) Chamado pelo timer do ROS """
        frame_to_process = None
        
        # ### OTMIZAÇÃO ###: Pega o frame mais recente de forma segura
        with self.capture_lock:
            if self.current_frame is None:
                return  # Ainda não recebemos um frame
            # Copia o frame para processar, liberando a trava o mais rápido possível
            frame_to_process = self.current_frame.copy()

        # O process_frame agora é chamado aqui, fora da thread da câmera
        if frame_to_process is not None:
            self.process_frame(frame_to_process)

    def process_frame(self, frame):
        """ (Ambos os Modos) Processa um único frame """
        
        # Corrige distorção da lente
        if not self.use_simulation and self.use_calibration and self.map1 is not None:
            frame = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
            
            # ### OTMIZAÇÃO ###: Recorte (ROI) removido, pois usamos alpha=0

        # A inferência é a parte pesada
        self.classes, self.scores, self.boxes, self.inference_frame = ri.detect_model(
            self.model, frame # Processa o frame que foi passado como argumento
        )

        self.publish_results() # Publica os resultados baseados em self.classes, etc.

        if self.use_simulation:
            try:
                processed_image_msg = self.bridge.cv2_to_imgmsg(self.inference_frame, "bgr8")
                self.processed_image_publisher.publish(processed_image_msg)
            except Exception as e:
                self.get_logger().error(f'Falha ao publicar imagem processada: {e}')

        if self.output_img:
            cv2.imshow("Visao EDROM", self.inference_frame)
            # ### OTMIZAÇÃO ###: waitKey(1) é essencial para o imshow funcionar
            # Ele é chamado no thread principal (pelo timer ou callback),
            # então ele não bloqueia a captura.
            cv2.waitKey(1)


    # =============================================================
    # ===================== PUBLICAÇÃO FINAL =====================
    # =============================================================
    def publish_results(self):
        objects_msg = VisionData()
        objects_msg.searching = self.searching
        objects_msg.fps = int(self.fps) # Note: Este é o FPS da CÂMERA

        ball_objects, robot_objects = [], []
        right_goal_objects, left_goal_objects = [], []
        x_intersection_objects, l_intersection_objects, t_intersection_objects, center_objects = [], [], [], []

        if getattr(self, 'boxes', []):
            for i in range(len(self.boxes)):
                box = self.boxes[i]
                results = [True, int(box[0]), int(box[1]), int(box[2]), int(box[3]), self.scores[i]]
                class_id = self.classes[i]
                if class_id == 0: ball_objects.append(results)
                elif class_id == 1: robot_objects.append(results)
                elif class_id == 2: right_goal_objects.append(results)
                elif class_id == 3: left_goal_objects.append(results)
                elif class_id == 4: l_intersection_objects.append(results)
                elif class_id == 5: t_intersection_objects.append(results)
                elif class_id == 6: x_intersection_objects.append(results)
                elif class_id == 7: center_objects.append(results)

        # Ordenação e atribuição
        ball_objects.sort(key=lambda o: o[5], reverse=True)
        robot_objects.sort(key=lambda o: o[5], reverse=True)
        right_goal_objects.sort(key=lambda o: o[1])
        left_goal_objects.sort(key=lambda o: o[1])

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
        msg = DetectionArray()
        msg.found = True
        msg.detections = [self.setup_object(d) for d in detection_list]
        return msg

    # =============================================================
    # ====================== AJUSTE DE CÂMERA =====================
    # =============================================================
    def ajuste_camera(self):
        print("Ajuste de Brilho: '=' para aumentar, '-' para diminuir. 'w' para continuar.")
        while rclpy.ok():
            key = cv2.waitKey(1)
            if key == ord('w'):
                break
            
            # Pega o frame mais recente da thread de captura
            frame = None
            with self.capture_lock:
                if self.current_frame is not None:
                    frame = self.current_frame.copy()
            
            if frame is None:
                continue

            # ### OTMIZAÇÃO ###: Aplicar calibração (sem recorte)
            if self.use_calibration and self.map1 is not None:
                frame = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
                # Recorte (ROI) removido

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
    no_visao = Visao('Visao')
    try:
        rclpy.spin(no_visao)
    except KeyboardInterrupt:
        no_visao.get_logger().info('Nó encerrado via (Ctrl+C)')
    finally:
        no_visao.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()