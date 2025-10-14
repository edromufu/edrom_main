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
# NOTA: Certifique-se de que ParticleFilter.py e FieldGenerator.py estão acessíveis
from .ParticleFilter import ParticleFilter as pf
from .FieldGenerator import FieldGenerator as fg

class Localization(Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info('Nó de Localização (com Decisão de Lado Inicial) Iniciado')

        # --- Parâmetros ---
        self.N = self.declare_parameter('loc.number', 250).get_parameter_value().integer_value
        self.showField = self.declare_parameter('loc.showfield', True).get_parameter_value().bool_value
        
        # --- Parâmetro de Lado Inicial (CORRIGIDO) ---
        # Lado de início: 'LEFT', 'RIGHT', ou 'BOTH'
        self.start_side = self.declare_parameter('loc.start_side', 'LEFT').get_parameter_value().string_value
        self.get_logger().info(f"Inicialização do lado: {self.start_side}")
        
        # --- Parâmetros de Ruído (Otimização Sugerida) ---
        self.desvioPos = 2.0      # Incerteza da odometria em cm
        self.desvioAngle = 1.5    # Incerteza da odometria em graus
        self.sensor_noise_dist = 15.0 # Incerteza do sensor de visão em cm (Reduzido para maior precisão)
        self.sensor_noise_angle = 5.0 # Incerteza do sensor de visão em graus (Reduzido para maior precisão)
        
        ### Limiar de variância para considerar a localização confiável (em cm^2)
        self.confidence_variance_threshold = 100.0 # (Desvio padrão de 10 cm)
        
        ### Máquina de Estados
        self.localization_state = 'INICIALIZANDO' 
        self.head_scan_complete = False
        self.allow_resampling = False
        
        ### Metas de Ataque e Defesa
        self.goal_to_attack_coords = None
        self.goal_to_defend_coords = None
        # Coordenadas do centro dos gols
        self.GOAL_ESQUERDO_COORDS = np.array([fg.padding, fg.padding + fg.fieldWidth / 2])
        self.GOAL_DIREITO_COORDS = np.array([fg.padding + fg.fieldLenght, fg.padding + fg.fieldWidth / 2])
        
        # --- Dados Armazenados, Mapeamento de IDs, etc. ---
        self.imu_yaw_deg = 0.0
        self.head_angles_deg = [0.0, 0.0]
        self.ground_truth_pose = None
        # Mapeamento do ID do Landmark (Visão) -> ID do Landmark (Filtro)
        self.vision_to_filter_id_map = { 4: 2, 5: 3, 3: 4, 2: 5, 7: 0 } 
        self.fov = np.deg2rad(80)
        self.camRange = 300
        self.minRange = 30
        self.limit = np.array([[0,0],[fg.padding*2 + fg.fieldLenght, fg.padding*2 + fg.fieldWidth]])
        self.particleFilter = pf(self.N, self.fov, self.minRange, fg.fieldIntersections)
        self.odom_buffer = deque()
        self.landmark_buffer = deque()
        
        self.loc_command_publisher = self.create_publisher(String, 'localization/command', 10)
        self.attack_goal_publisher = self.create_publisher(Float32MultiArray, 'game/goal_to_attack', 10)
        
        # --- Subscribers ---
        self.create_subscription(String, 'localization/status', self.loc_status_callback, 10)
        self.create_subscription(LandmarkArray, 'vision/landmarks', self.landmarks_callback, 10)
        self.create_subscription(Float32MultiArray, 'robot/odometry', self.odometry_callback, 10)
        self.create_subscription(Float32, 'imu/topic', self.imu_callback, 10)
        self.create_subscription(Float32MultiArray, 'head/joint_states', self.head_callback, 10)
        self.create_subscription(Float32MultiArray, '/simulation/ground_truth_pose', self.ground_truth_callback, 10)

        # --- Timers ---
        processing_rate = 50.0
        self.processing_timer = self.create_timer(1.0/processing_rate, self.processing_loop)
        if self.showField:
            self.visualization_timer = self.create_timer(0.1, self.update_visualization)

        self.initialize_particles_startup()

    # --- Método de Inicialização das Partículas (CORRIGIDO) ---
    def initialize_particles_startup(self):
        self.get_logger().info(f"Dispersando partículas nas posições de início para o lado: {self.start_side}")
        
        # 1. Selecionar as posições de início baseadas na flag
        if self.start_side == 'LEFT':
            start_positions = fg.allStartPos[:4] # startPosL1 a L4
        elif self.start_side == 'RIGHT':
            start_positions = fg.allStartPos[4:] # startPosR1 a R4
        else: # 'BOTH' ou inválido
            start_positions = fg.allStartPos
            
        num_positions = len(start_positions)
        if num_positions == 0:
            self.get_logger().error("Nenhuma posição inicial definida!")
            return
            
        # 2. Configuração de N
        # ATENÇÃO: self.N já foi lido e declarado no __init__. Evita o erro "ParameterAlreadyDeclaredException".
        if self.N >= num_positions:
            self.N = (self.N // num_positions) * num_positions 
        else:
            self.N = num_positions
            
        particles_per_pos = self.N // num_positions
        
        # 3. Geração das Partículas
        all_particles = [
            mean_pos[:3] + (randn(particles_per_pos, 3) * [30, 30, 20]) 
            for mean_pos in start_positions
        ]
        
        self.particleFilter.particles = np.concatenate(all_particles)
        self.particleFilter.particles[:, 2] %= 360
        self.particleFilter.N = self.N 
        self.particleFilter.weights = np.ones(self.N) / self.N
        self.get_logger().info(f"Filtro inicializado com {self.N} partículas em {num_positions} aglomerados.")
        
        # 4. Iniciar a Máquina de Estados
        self.localization_state = 'INICIALIZANDO'
        self.allow_resampling = False
        self.head_scan_complete = False
        self.goal_to_attack_coords = None
        self.get_logger().info("Estado -> INICIALIZANDO. Reamostragem desligada. Iniciando Scan.")
        
        self.send_localization_command('START_SCAN') # Sempre inicia o scan

    def send_localization_command(self, command):
        self.get_logger().info(f"Enviando comando para o Behavior: '{command}'")
        self.loc_command_publisher.publish(String(data=command))

    # --- Callbacks (Inalterados) ---
    def loc_status_callback(self, msg):
        self.get_logger().info(f"Recebido status: '{msg.data}'")
        if msg.data == 'SCAN_COMPLETE':
            self.head_scan_complete = True

    def odometry_callback(self, msg):
        timestamp = self.get_clock().now().to_msg()
        self.odom_buffer.append((timestamp, msg.data))

    def landmarks_callback(self, msg):
        if msg.landmarks:
            self.landmark_buffer.append(msg)

    def head_callback(self, msg):
        self.head_angles_deg[0] = np.rad2deg(msg.data[0])
        self.head_angles_deg[1] = np.rad2deg(msg.data[1])

    def imu_callback(self, msg):
        self.imu_yaw_deg = np.rad2deg(msg.data) % 360

    def ground_truth_callback(self, msg):
        self.ground_truth_pose = np.array(msg.data)

    # --- Loop de Processamento (Lógica do Gol CORRIGIDA) ---
    def processing_loop(self):
        if not self.landmark_buffer:
            return

        vision_msg = self.landmark_buffer.popleft()
        vision_time = Time.from_msg(vision_msg.header.stamp)

        while self.odom_buffer:
            odom_time = Time.from_msg(self.odom_buffer[0][0])
            if odom_time < vision_time:
                _ , odom_data = self.odom_buffer.popleft()
                if self.localization_state != 'INICIALIZANDO':
                    erro_pos, erro_ang = self.desvioPos, self.desvioAngle
                    self.particleFilter.predict(odom_data, (erro_pos, erro_pos, erro_ang), self.limit)
            else:
                break
        
        observed_landmarks = [
            (lm.distance_m * 100.0, self.vision_to_filter_id_map.get(lm.id), np.rad2deg(lm.angle_rad))
            for lm in vision_msg.landmarks if self.vision_to_filter_id_map.get(lm.id) is not None
        ]
        
        if observed_landmarks:
            self.particleFilter.update(observed_landmarks, self.sensor_noise_dist, self.sensor_noise_angle, 
                                       self.head_angles_deg[0], self.camRange, self.imu_yaw_deg)
        
        # --- MÁQUINA DE ESTADOS LÓGICOS ---

        if self.localization_state == 'INICIALIZANDO' and self.head_scan_complete:
            self.get_logger().info("Scan inicial completo. Realizando 1ª reamostragem.")
            
            self.particleFilter.resample_from_index()
            
            self.get_logger().info("Mudando para o estado -> DECIDINDO_LADO.")
            self.localization_state = 'DECIDINDO_LADO'

        elif self.localization_state == 'DECIDINDO_LADO':
            self.particleFilter.estimate()
            is_confident = (self.particleFilter.var[0] < self.confidence_variance_threshold and
                           self.particleFilter.var[1] < self.confidence_variance_threshold)
            
            if is_confident:
                self.get_logger().info(f"Localização CONFIÁVEL (var_x={self.particleFilter.var[0]:.1f}). Tomando decisão do lado.")
                pose_atual_x = self.particleFilter.mean[0]
                meio_do_campo_x = fg.padding + fg.fieldLenght / 2

                # LÓGICA CORRIGIDA: Atacar o gol OPOSTO ao lado detectado
                if pose_atual_x < meio_do_campo_x:
                    # Robô no lado ESQUERDO (x < meio). Ataca o gol DIREITO.
                    self.get_logger().info("DECIDIDO: Lado ESQUERDO. Atacar o Gol DIREITO.")
                    self.goal_to_attack_coords = self.GOAL_DIREITO_COORDS
                    self.goal_to_defend_coords = self.GOAL_ESQUERDO_COORDS
                else:
                    # Robô no lado DIREITO (x > meio). Ataca o gol ESQUERDO.
                    self.get_logger().info("DECIDIDO: Lado DIREITO. Atacar o Gol ESQUERDO.")
                    self.goal_to_attack_coords = self.GOAL_ESQUERDO_COORDS
                    self.goal_to_defend_coords = self.GOAL_DIREITO_COORDS
                
                goal_msg = Float32MultiArray(data=self.goal_to_attack_coords.tolist())
                self.attack_goal_publisher.publish(goal_msg)

                self.get_logger().info("Mudando para o estado -> RASTREAMENTO. Reamostragem ATIVADA.")
                self.localization_state = 'RASTREAMENTO'
                self.allow_resampling = True
            else:
                self.get_logger().info(f"Aguardando convergência (var_x={self.particleFilter.var[0]:.1f})", throttle_duration_sec=1.0)
                if self.particleFilter.neff() < (self.N / 2):
                    self.particleFilter.resample_from_index()

        elif self.localization_state == 'RASTREAMENTO':
            if self.allow_resampling and self.particleFilter.neff() < (self.N / 2):
                self.get_logger().info(f"Neff baixo ({self.particleFilter.neff():.2f}), reamostrando.")
                self.particleFilter.resample_from_index()
        
        self.particleFilter.estimate()

    # --- Visualização (Adição do Gol de Ataque) ---
    def update_visualization(self):
        field = fg.generate()
        field = fg.drawInField(field)
        
        neck_angle = self.head_angles_deg[0]
        field = fg.drawParticles(field, self.particleFilter.particles, neckAngle=neck_angle, fov=self.fov)
        
        estimated_pose = np.append(self.particleFilter.mean, neck_angle)
        fg.drawParticle(field, estimated_pose, self.fov, self.minRange, self.camRange, 
                        drawFov=True, color=[200, 10, 250], robo=True) # Cor roxa/rosa para estimativa
        
        if self.ground_truth_pose is not None:
            gt_pose_with_neck = np.append(self.ground_truth_pose, neck_angle)
            fg.drawParticle(field, gt_pose_with_neck, self.fov, self.minRange, self.camRange, 
                            drawFov=False, color=[0, 255, 0], robo=True) # Cor verde para Ground Truth

        if self.goal_to_attack_coords is not None:
            center_x = int(self.goal_to_attack_coords[0])
            center_y = int(self.goal_to_attack_coords[1])
            # Desenha um grande círculo amarelo para indicar o gol de ataque (Amarelo = Goal)
            cv.circle(field, (center_x, center_y), 15, [0, 255, 255], -1) 

        cv.imshow("2D Particle Filter", cv.flip(field, 0))
        key = cv.waitKey(1)
        if key == 27: # Tecla ESC
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