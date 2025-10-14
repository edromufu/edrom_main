# coding=utf-8
import rclpy
from rclpy.node import Node
import numpy as np
import cv2 as cv
from numpy.random import randn
from rclpy.time import Time
from collections import deque

# --- Mensagens ROS ---
from std_msgs.msg import Float32MultiArray, Float32, String
from edrom_msgs.msg import LandmarkArray

# --- Código Local ---
from .ParticleFilter import ParticleFilter as pf
from .FieldGenerator import FieldGenerator as fg

class Localization(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info('Nó de Localização (com Sincronização por Timestamp) Iniciado')

        # --- Parâmetros ---
        self.N = self.declare_parameter('loc.number', 250).get_parameter_value().integer_value
        self.showField = self.declare_parameter('loc.showfield', True).get_parameter_value().bool_value
        
        # --- Parâmetros de Ruído (para serem calibrados) ---
        self.desvioPos = 1.5      # Incerteza da odometria em cm
        self.desvioAngle = 1.0    # Incerteza da odometria em graus
        self.sensor_noise_dist = 60.0 # Incerteza do sensor de visão em cm
        self.sensor_noise_angle = 20.0 # Incerteza do sensor de visão em graus
        
        # --- Lógica de Estado ---
        self.localization_state = 'INICIALIZANDO'
        self.head_scan_complete = False
        self.allow_resampling = False
        
        # --- Dados Armazenados (Estado Contínuo) ---
        self.imu_yaw_deg = 0.0
        self.head_angles_deg = [0.0, 0.0]  # [pan_deg, tilt_deg]
        self.ground_truth_pose = None
        
        # --- Mapeamento de IDs (CRÍTICO!) ---
        self.vision_to_filter_id_map = { 4: 2, 5: 3, 3: 4, 2: 5, 7: 0 }

        # --- Configurações do Filtro e Campo ---
        self.fov = np.deg2rad(80)
        self.camRange = 300
        self.minRange = 30
        self.limit = np.array([[0,0],[fg.padding*2 + fg.fieldLenght, fg.padding*2 + fg.fieldWidth]])
        self.particleFilter = pf(self.N, self.fov, self.minRange, fg.fieldIntersections)

        # --- Buffers para Sincronização de Mensagens ---
        self.odom_buffer = deque()
        self.landmark_buffer = deque()
        
        # --- Publishers ---
        self.loc_command_publisher = self.create_publisher(String, 'localization/command', 10)
        
        # --- Subscribers ---
        self.create_subscription(String, 'localization/status', self.loc_status_callback, 10)
        self.create_subscription(LandmarkArray, 'vision/landmarks', self.landmarks_callback, 10)
        self.create_subscription(Float32MultiArray, 'robot/odometry', self.odometry_callback, 10)
        self.create_subscription(Float32, 'imu/topic', self.imu_callback, 10)
        self.create_subscription(Float32MultiArray, 'head/joint_states', self.head_callback, 10)
        self.create_subscription(Float32MultiArray, '/simulation/ground_truth_pose', self.ground_truth_callback, 10)

        # --- Timers ---
        # Timer principal para processar os dados dos buffers em ordem cronológica
        processing_rate = 50.0  # Processa os dados a 50Hz
        self.processing_timer = self.create_timer(1.0/processing_rate, self.processing_loop)
        
        if self.showField:
            self.visualization_timer = self.create_timer(0.1, self.update_visualization)

        self.initialize_particles_startup()

    def initialize_particles_startup(self):
        # ... (código de inicialização das partículas permanece o mesmo) ...
        self.get_logger().info("Dispersando partículas nas 4 posições de início.")
        num_positions = len(fg.allStartPos)
        if num_positions == 0:
            self.get_logger().error("Nenhuma posição inicial definida!")
            return
        particles_per_pos = self.N // num_positions
        all_particles = [
            mean_pos + (randn(particles_per_pos, 3) * [30, 30, 20])
            for mean_pos in fg.allStartPos
        ]
        self.particleFilter.particles = np.concatenate(all_particles)
        self.particleFilter.particles[:, 2] %= 360
        new_N = len(self.particleFilter.particles)
        self.particleFilter.N = new_N
        self.N = new_N
        self.particleFilter.weights = np.ones(new_N) / new_N
        self.get_logger().info(f"Filtro inicializado com {new_N} partículas.")
        
        # Restaura a lógica de estado original
        self.localization_state = 'INICIALIZANDO'
        self.allow_resampling = False
        self.head_scan_complete = False
        self.get_logger().info("Estado -> INICIALIZANDO. Reamostragem desligada.")
        self.send_localization_command('START_SCAN')

    def send_localization_command(self, command):
        self.get_logger().info(f"Enviando comando para o Behavior: '{command}'")
        self.loc_command_publisher.publish(String(data=command))

    def loc_status_callback(self, msg):
        self.get_logger().info(f"Recebido status: '{msg.data}'")
        if msg.data == 'SCAN_COMPLETE':
            self.head_scan_complete = True

    # --- NOVOS CALLBACKS "COLETORES" ---
    def odometry_callback(self, msg):
        # A mensagem Float32MultiArray não tem header. Nós a "carimbamos" com o tempo de chegada.
        # O ideal é que o nó da odometria publique uma mensagem com header.
        timestamp = self.get_clock().now().to_msg()
        self.odom_buffer.append((timestamp, msg.data))

    def landmarks_callback(self, msg):
        # A mensagem LandmarkArray já tem header, o que é perfeito.
        if msg.landmarks:
            self.landmark_buffer.append(msg)

    # --- CALLBACKS DE ESTADO (sem mudanças) ---
    def head_callback(self, msg):
        self.head_angles_deg[0] = np.rad2deg(msg.data[0])
        self.head_angles_deg[1] = np.rad2deg(msg.data[1])

    def imu_callback(self, msg):
        self.imu_yaw_deg = np.rad2deg(msg.data) % 360

    def ground_truth_callback(self, msg):
        self.ground_truth_pose = np.array(msg.data)

    # --- NOVO LOOP DE PROCESSAMENTO CENTRAL ---
    def processing_loop(self):
        # Só processa se tivermos uma medição de visão para usar como referência de tempo
        if not self.landmark_buffer:
            return

        # Pega a medição de visão mais antiga da fila
        vision_msg = self.landmark_buffer.popleft()
        vision_time = Time.from_msg(vision_msg.header.stamp)

        # Processa todas as medições de odometria que aconteceram ANTES desta visão
        while self.odom_buffer:
            odom_time = Time.from_msg(self.odom_buffer[0][0])
            if odom_time < vision_time:
                _ , odom_data = self.odom_buffer.popleft()
                if self.localization_state == 'RASTREAMENTO':
                    erro_pos, erro_ang = self.desvioPos, self.desvioAngle
                    self.particleFilter.predict(odom_data, (erro_pos, erro_pos, erro_ang), self.limit)
            else:
                break # A próxima odometria é mais nova que a visão, para por aqui.
        
        # Agora, com a nuvem no estado correto, aplica a correção da visão
        observed_landmarks = [
            (lm.distance_m * 100.0, self.vision_to_filter_id_map.get(lm.id), np.rad2deg(lm.angle_rad))
            for lm in vision_msg.landmarks if self.vision_to_filter_id_map.get(lm.id) is not None
        ]
        
        if observed_landmarks:
            self.particleFilter.update(observed_landmarks, self.sensor_noise_dist, self.sensor_noise_angle, 
                                       self.head_angles_deg[0], self.camRange, self.imu_yaw_deg)
        
        # Lógica de reamostragem e transição de estado
        if self.localization_state == 'INICIALIZANDO' and self.head_scan_complete:
            self.get_logger().info("Scan completo. Realizando 1ª reamostragem e mudando para RASTREAMENTO.")
            self.particleFilter.resample_from_index()
            self.localization_state = 'RASTREAMENTO'
            self.allow_resampling = True
        
        if self.allow_resampling and self.particleFilter.neff() < (self.N / 2):
            self.get_logger().info(f"Neff baixo ({self.particleFilter.neff():.2f}), reamostrando.")
            self.particleFilter.resample_from_index()
        
        self.particleFilter.estimate()

    # --- FUNÇÃO DE VISUALIZAÇÃO (sem mudanças) ---
    def update_visualization(self):
        field = fg.generate()
        field = fg.drawInField(field)
        
        neck_angle = self.head_angles_deg[0]
        field = fg.drawParticles(field, self.particleFilter.particles, neckAngle=neck_angle, fov=self.fov)
        
        estimated_pose = np.append(self.particleFilter.mean, neck_angle)
        fg.drawParticle(field, estimated_pose, self.fov, self.minRange, self.camRange, 
                        drawFov=True, color=[200, 10, 250], robo=True)
        
        if self.ground_truth_pose is not None:
            gt_pose_with_neck = np.append(self.ground_truth_pose, neck_angle)
            fg.drawParticle(field, gt_pose_with_neck, self.fov, self.minRange, self.camRange, 
                            drawFov=False, color=[0, 255, 0], robo=True)

        cv.imshow("2D Particle Filter", cv.flip(field, 0))
        key = cv.waitKey(1)
        if key == 27:
            self.get_logger().info("Tecla ESC pressionada, encerrando.")
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node_localization = Localization('Localizacao')
    rclpy.spin(node_localization)
    node_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()