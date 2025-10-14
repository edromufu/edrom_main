# coding=utf-8
import numpy as np
from filterpy.monte_carlo import systematic_resample
from numpy.random import randn, uniform

class ParticleFilter():

    # Contructor
    def __init__(self, N, fov, minRange, intersections):
        self.N = N
        self.fov = fov
        self.minRange = minRange
        self.intersections = intersections
        
        # As variáveis são preparadas, mas os arrays são criados vazios
        self.particles = np.empty((self.N, 3))
        self.weights = np.empty(self.N)
        
        self.mean = np.zeros(3)
        self.var = np.zeros(3)
        self.desvioAngle = 15.0

    # Gera partículas distribuídas uniformemente dentro dos intervalos fornecidos (xRange, yRange, headingRange).
    def create_uniform_particles(self, xRange, yRange, headingRange):
        particles = np.empty((self.N, 3))   #Cria um array vazio para armazenar as partículas, N x 3.

        #Gera posições x, y e angulos uniformemente distribuídas no intervalo xRange, yRange e headingRange.
        particles[:, 0] = uniform(xRange[0], xRange[1], size=self.N)    
        particles[:, 1] = uniform(yRange[0], yRange[1], size=self.N) 
        particles[:, 2] = uniform(headingRange[0], headingRange[1], size=self.N)    
        
        particles[:, 2] %= 360  #Normaliza os ângulos para o intervalo [0, 360) graus.

        return particles.astype(float) # Retorna as partículas como float para precisão

    # Gera partículas distribuídas segundo uma distribuição Gaussiana com a média (mean) e desvio padrão (standardDeviation) fornecidos.
    def create_gaussian_particles(self, mean, standardDeviation):
        particles = np.empty((self.N, 3))   #Cria um array vazio para armazenar as partículas.

        #Gera posições x,y e ângulos com distribuição Gaussiana com média mean[0],mean[1],mean[2] e desvio padrão standardDeviation[0],standardDeviation[1],standardDeviation[2] respectivamente.
        particles[:, 0] = mean[0] + (randn(self.N) * standardDeviation[0])
        particles[:, 1] = mean[1] + (randn(self.N) * standardDeviation[1])
        particles[:, 2] = mean[2] + (randn(self.N) * standardDeviation[2])

        particles[:, 2] %= 360      #Normaliza os ângulos para o intervalo [0, 360) graus.
        return particles.astype(float)   #Retorna as partículas como float para precisão

    # Em ParticleFilter.py

    def predict(self, passo, erro, limit=(0, 0)):
        """
        Prevê o próximo estado das partículas com base no comando de odometria.
        A ordem das operações é crucial para a precisão física.
        """
        # passo e erro = [dx, dy, dtheta] em cm e graus
        
        # 1. Salva o ângulo original em radianos para o cálculo da translação.
        #    Este é o estado da partícula no início do passo de tempo.
        theta_rad = np.deg2rad(self.particles[:, 2])

        # 2. Gera o deslocamento com ruído, que está no referencial do robô.
        dist_x = passo[0] + (randn(self.N) * erro[0])
        dist_y = passo[1] + (randn(self.N) * erro[1])

        # 3. ATUALIZA A POSIÇÃO (X, Y) USANDO O ÂNGULO ANTIGO.
        #    Rotacionamos o deslocamento local (dist_x, dist_y) para o referencial
        #    global do campo usando o ângulo que a partícula TINHA.
        self.particles[:, 0] += (dist_x * np.cos(theta_rad) - dist_y * np.sin(theta_rad))
        self.particles[:, 1] += (dist_x * np.sin(theta_rad) + dist_y * np.cos(theta_rad))
        
        # 4. AGORA, E SOMENTE AGORA, ATUALIZA A ORIENTAÇÃO da partícula.
        self.particles[:, 2] += passo[2] + (randn(self.N) * erro[2])
        self.particles[:, 2] %= 360  # Normaliza os ângulos para o intervalo [0, 360)

        # 5. Opcional: Lógica para manter as partículas dentro dos limites do campo.
        #    (O código de reflexão ou contenção pode ser adicionado aqui, se necessário)
        #    np.clip(self.particles[:, 0], limit[0][0], limit[1][0], out=self.particles[:, 0])
        #    np.clip(self.particles[:, 1], limit[0][1], limit[1][1], out=self.particles[:, 1])
        
    # --- FUNÇÃO CORRIGIDA E OTIMIZADA ---
    # Verifica quais interseções estão dentro do campo de visão (FOV) de uma partícula.
    def checkFOV(self, particle, maxRange, neckAngle):
        seen = []
        # Converte a orientação da partícula e o fov para radianos uma única vez
        particle_angle_rad = np.deg2rad(particle[2])
        fov_rad = self.fov

        for intersection in self.intersections:
            if len(intersection) < 3: continue

            # Calcula o vetor da partícula PARA a interseção
            distX = intersection[0][0] - particle[0]
            distY = intersection[0][1] - particle[1]
            distance = np.sqrt(distX**2 + distY**2)

            if self.minRange <= distance <= maxRange:
                # Ângulo do landmark no referencial global (do mundo)
                # Resultado em radianos, no intervalo [-pi, pi]
                angle_to_landmark_global_rad = np.arctan2(distY, distX)
                
                # Orientação total da câmera (corpo + pescoço) em radianos
                camera_angle_rad = particle_angle_rad + np.deg2rad(neckAngle)

                # Diferença angular entre a direção da câmera e o landmark
                angle_diff_rad = angle_to_landmark_global_rad - camera_angle_rad
                
                # Normaliza a diferença para o intervalo [-pi, pi] para a checagem
                # Isso é crucial para evitar problemas quando os ângulos cruzam 360/0 graus
                angle_diff_norm_rad = (angle_diff_rad + np.pi) % (2 * np.pi) - np.pi

                # Verifica se o landmark está dentro do cone de visão (FOV)
                if abs(angle_diff_norm_rad) <= fov_rad / 2.0:
                    # O ângulo a ser retornado é o ângulo relativo à câmera, que já calculamos
                    angle_seen_deg = np.rad2deg(angle_diff_norm_rad)
                    seen.append((intersection, distance, angle_seen_deg))
        return seen
    
    # Calcula o número efetivo de partículas para verificar a necessidade de reamostragem (resample).
    def neff(self):
        return 1. / np.sum(np.square(self.weights))
    
    # Reamostra as partículas com base nos pesos, utilizando o método de reamostragem sistemática.
    def resample_from_index(self):
        indexes = systematic_resample(self.weights)
        # Reamostra as partículas e reseta os pesos
        self.particles[:] = self.particles[indexes]
        self.weights.fill(1.0 / self.N)

    # Estima a posição do robô calculando a média e a variância ponderada das partículas.
    def estimate(self):
        pos = self.particles
        
        # Média circular para o ângulo para evitar problemas com a descontinuidade em 0/360 graus
        mean_sin = np.average(np.sin(np.deg2rad(pos[:, 2])), weights=self.weights, axis=0)
        mean_cos = np.average(np.cos(np.deg2rad(pos[:, 2])), weights=self.weights, axis=0)
        mean_angle = np.rad2deg(np.arctan2(mean_sin, mean_cos))
        
        self.mean[0:2] = np.average(pos[:, 0:2], weights=self.weights, axis=0)
        self.mean[2] = (mean_angle + 360) % 360 # Garante resultado positivo

        self.var = np.average((pos - self.mean)**2, weights=self.weights, axis=0)
        # A variância do ângulo precisa de tratamento especial, mas por enquanto isso é suficiente
        
    # Lógica principal de atualização de pesos
    # NO ARQUIVO: ParticleFilter.py

# --- FUNÇÃO ATUALIZADA ---
    # ParticleFilter.py (Substituir o método update completo)

    def update(self, observed_landmarks, sensor_noise_dist, sensor_noise_angle, neck_angle_deg, max_range_cm,imu_yaw_deg):
        # observed_landmarks é uma lista de tuplas (dist_cm, id, ang_deg)
        
        # Parâmetro para penalizar Falsos Positivos e Misses (Ajustável!)
        PROB_MISS_PENALTY = 0.5 
        
        for i, particle in enumerate(self.particles):
            expected_landmarks = self.checkFOV(particle, max_range_cm, neck_angle_deg)
            
            total_prob = 1.0
            
            # 1. Penalidade por Ver Nada / Prever Algo
            
            # Penalidade 1: O robô vê landmarks (visão) mas a partícula não (está em um local "cego")
            if observed_landmarks and not expected_landmarks:
                total_prob *= 0.001 # Penalidade alta por estar em um local cego mas ver algo
                
            # Penalidade 2: O robô não vê nada (visão) mas a partícula prevê vários landmarks
            if not observed_landmarks and expected_landmarks:
                # Penaliza suavemente pela possível perda de detecção (Falso Negativo da Visão)
                total_prob *= 0.1 

            # 2. Casamento de Observações
            for obs_dist, obs_id, obs_ang in observed_landmarks:
                
                best_match_found = False
                best_prob_for_obs = 1e-300
                
                for exp_data in expected_landmarks:
                    exp_landmark_data, exp_dist, exp_ang = exp_data
                    exp_id = exp_landmark_data[2]
                    
                    if obs_id == exp_id:
                        dist_diff = obs_dist - exp_dist
                        ang_diff = obs_ang - exp_ang
                        
                        # Função de densidade de probabilidade Gaussiana (PDF)
                        prob_dist = np.exp(-(dist_diff**2) / (2 * sensor_noise_dist**2))
                        prob_ang = np.exp(-(ang_diff**2) / (2 * sensor_noise_angle**2))
                        
                        prob = prob_dist * prob_ang
                        
                        # Encontra o melhor match para esta observação
                        if prob > best_prob_for_obs:
                            best_prob_for_obs = prob
                            best_match_found = True

                # Multiplica pela probabilidade do melhor match
                total_prob *= best_prob_for_obs
                
                # Penalidade para Falso Positivo (Ghost Landmark):
                # Se não houver um bom match (prob muito baixa), penalizar a partícula suavemente.
                # O limiar 0.001 é arbitrário, mas funciona bem na prática
                if best_prob_for_obs < 0.001 and best_prob_for_obs > 1e-300: 
                    total_prob *= PROB_MISS_PENALTY # Penaliza em 50%

            # 3. Integração do IMU
            imu_error = particle[2] - imu_yaw_deg
            imu_error = (imu_error + 180) % 360 - 180
            
            # Usando um desvio padrão de 20 graus (20.0) para ser mais restritivo que o 30.0 anterior
            prob_imu = np.exp(-(imu_error ** 2) / (2 * (20.0) ** 2)) 
            
            self.weights[i] = total_prob * prob_imu


        # A normalização dos pesos permanece a mesma
        self.weights += 1e-300
        sum_weights = np.sum(self.weights)
        if sum_weights > 0:
            self.weights /= sum_weights
        else:
            self.weights.fill(1.0 / self.N)