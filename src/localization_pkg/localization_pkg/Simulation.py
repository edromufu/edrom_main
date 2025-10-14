# coding=utf-8
import rclpy
from rclpy.node import Node
import numpy as np
import random

# --- Mensagens ROS ---
from std_msgs.msg import Float32MultiArray, Float32, String
from edrom_msgs.msg import Landmark, LandmarkArray

# --- Código local ---
from .FieldGenerator import FieldGenerator as fg
from .ParticleFilter import ParticleFilter as pf

class Simulation(Node):

    def __init__(self):
        super().__init__('robot_simulation')
        self.get_logger().info('Nó de Simulação do Robô (MODO REALISTA) Iniciado')

        # --- Publishers ---
        self.odometry_publisher = self.create_publisher(Float32MultiArray, 'robot/odometry', 10)
        self.head_publisher = self.create_publisher(Float32MultiArray, 'head/joint_states', 10)
        self.landmarks_publisher = self.create_publisher(LandmarkArray, 'vision/landmarks', 10)
        self.imu_publisher = self.create_publisher(Float32, 'imu/topic', 10)
        self.loc_status_publisher = self.create_publisher(String, 'localization/status', 10)
        self.ground_truth_publisher = self.create_publisher(Float32MultiArray, 'simulation/ground_truth_pose', 10)


        # --- Subscriber para Comandos de Movimento ---
        self.create_subscription(Float32MultiArray, 'robot/command', self.command_callback, 10)
        
        # --- Estado do Robô (Ground Truth) ---
        self.sim_robot_pose = np.array([
            fg.padding + fg.fieldLenght / 2, 
            fg.padding + fg.fieldWidth / 2, 
            0.0, 0.0, 0.0 # x, y, body_theta_deg, head_pan_deg, head_tilt_deg
        ], dtype=float)

        self.odometry_command = np.zeros(3)

        # --- Mapeamento de IDs e Utilitários ---
        self.filter_to_vision_id_map = { 2: 4, 3: 5, 4: 3, 5: 2, 0: 7 }
        self.helper_filter = pf(N=1, fov=np.deg2rad(80), minRange=30, intersections=fg.fieldIntersections)
        self.camRange_cm = 400

        # =============================================================================
        # NOVOS PARÂMETROS PARA SIMULAÇÃO REALISTA
        # =============================================================================
        
        # --- Parâmetros de Realismo da VISÃO ---
        self.vision_detection_prob = 0.95  # 95% de chance de detectar um landmark visível
        self.vision_false_positive_rate = 0.03 # 3% de chance de gerar um landmark "fantasma" por ciclo
        
        # Ruído da visão agora é dependente da distância
        self.vision_noise_base_dist_cm = 3.0   # Ruído base em cm
        self.vision_noise_dist_factor = 0.05   # Aumenta o ruído em 5% da distância
        self.vision_noise_base_angle_deg = 1.0 # Ruído angular em graus

        # --- Parâmetros de Realismo do MOVIMENTO ---
        # Fator de eficiência: < 1.0 significa que o robô se move menos do que o comandado
        self.odom_efficiency = np.array([0.9, 0.9, 0.95]) # [dx_eff, dy_eff, d_theta_eff]

        # Ruído adicionado ao movimento REAL do robô (afeta o ground truth)
        self.odom_ground_truth_noise_std = np.array([0.5, 0.5, 0.8]) # [cm, cm, deg]
        
        # Ruído adicionado à odometria PUBLICADA (o que o filtro vai receber)
        # Pode ser diferente do ruído real para simular erros de calibração
        self.odom_published_noise_std = np.array([1.0, 1.0, 1.5]) # [cm, cm, deg]
        
        # =============================================================================

        # --- Timers ---
        self.timer = self.create_timer(0.1, self.simulation_loop)
        self.scan_complete_timer = self.create_timer(5.0, self.publish_scan_complete)


    def command_callback(self, msg):
        """Recebe o comando de movimento do nó teleop e o armazena."""
        if len(msg.data) == 3:
            self.odometry_command = np.array(msg.data)

    def publish_scan_complete(self):
        msg = String(data='SCAN_COMPLETE')
        self.loc_status_publisher.publish(msg)
        self.scan_complete_timer.cancel()

    def simulation_loop(self):
        # --- 1. Simulação de Movimento Realista ---
        
        # Aplica ineficiência ao comando recebido
        inefficient_command = self.odometry_command * self.odom_efficiency
        
        # Adiciona ruído para obter o movimento REAL (ground truth)
        actual_movement = inefficient_command + (np.random.randn(3) * self.odom_ground_truth_noise_std)
        
        # Atualiza a pose REAL do robô (ground truth)
        dx_real, dy_real, d_theta_deg_real = actual_movement
        current_body_theta_rad = np.deg2rad(self.sim_robot_pose[2])
        self.sim_robot_pose[0] += dx_real * np.cos(current_body_theta_rad) - dy_real * np.sin(current_body_theta_rad)
        self.sim_robot_pose[1] += dx_real * np.sin(current_body_theta_rad) + dy_real * np.cos(current_body_theta_rad)
        self.sim_robot_pose[2] = (self.sim_robot_pose[2] + d_theta_deg_real) % 360
        
        # Gera a odometria a ser PUBLICADA (o que o filtro vai ver)
        # É o comando original + um ruído diferente
        odometry_for_filter = self.odometry_command + (np.random.randn(3) * self.odom_published_noise_std)
        self.odometry_publisher.publish(Float32MultiArray(data=odometry_for_filter.tolist()))
        
        self.odometry_command.fill(0) # Reseta o comando para o robô parar
        
        # Publica outros dados (IMU e cabeça não precisam de tanto realismo por enquanto)
        head_angles_rad = np.deg2rad(self.sim_robot_pose[3:5])
        self.head_publisher.publish(Float32MultiArray(data=head_angles_rad.tolist()))
        self.imu_publisher.publish(Float32(data=np.deg2rad(self.sim_robot_pose[2])))

        # --- 2. Simulação de Visão Realista ---
        visible_landmarks = self.helper_filter.checkFOV(self.sim_robot_pose, self.camRange_cm, self.sim_robot_pose[3])
        
        landmarks_msg = LandmarkArray()
        landmarks_msg.header.stamp = self.get_clock().now().to_msg()
        detected_landmarks = []
        
        for info, dist, angle in visible_landmarks:
            # Falso Negativo: O robô pode não detectar o landmark
            if np.random.rand() > self.vision_detection_prob:
                continue # Landmark "perdido", pula para o próximo
            
            vision_id = self.filter_to_vision_id_map.get(info[2])
            if vision_id is not None:
                # Ruído Dependente da Distância
                dist_noise_std = self.vision_noise_base_dist_cm + (dist * self.vision_noise_dist_factor)
                angle_noise_std = self.vision_noise_base_angle_deg

                noisy_dist = dist + (np.random.randn() * dist_noise_std)
                noisy_angle = angle + (np.random.randn() * angle_noise_std)

                landmark = Landmark(id=vision_id, distance_m=noisy_dist/100.0, angle_rad=np.deg2rad(noisy_angle))
                detected_landmarks.append(landmark)

        self.get_logger().info(f"Pose Real: ({self.sim_robot_pose[0]:.1f}, {self.sim_robot_pose[1]:.1f}, {self.sim_robot_pose[2]:.1f}°), LMs Detectados: {len(detected_landmarks)}", throttle_duration_sec=1.0)

        
        # Falso Positivo: Chance de adicionar um landmark "fantasma"
        if np.random.rand() < self.vision_false_positive_rate:
            self.get_logger().warn('Gerando um landmark FANTASMA!', throttle_duration_sec=5.0)
            ghost_dist = random.uniform(self.helper_filter.minRange, self.camRange_cm)
            ghost_angle = random.uniform(-40, 40) # Dentro do FOV de 80 graus
            ghost_id = random.choice(list(self.filter_to_vision_id_map.values()))
            
            ghost_landmark = Landmark(id=ghost_id, distance_m=ghost_dist/100.0, angle_rad=np.deg2rad(ghost_angle))
            detected_landmarks.append(ghost_landmark)

        landmarks_msg.landmarks = detected_landmarks
        self.landmarks_publisher.publish(landmarks_msg)
        
        self.get_logger().info(f"Pose Real: ({self.sim_robot_pose[0]:.1f}, {self.sim_robot_pose[1]:.1f}, {self.sim_robot_pose[2]:.1f}°), LMs Detectados: {len(detected_landmarks)}", throttle_duration_sec=1.0)

        gt_msg = Float32MultiArray(data=self.sim_robot_pose[0:3].tolist())
        self.ground_truth_publisher.publish(gt_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = Simulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()