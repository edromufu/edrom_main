# Motion — Geração de Movimento

O sistema de motion do Aurea é composto por quatro módulos principais, todos dentro de `src/edrom_motion/`.

| Módulo | Tecnologia | Função |
|--------|-----------|--------|
| `aurea_walk` | C++ + Eigen3 | Walking engine + cinemática inversa |
| `aurea_kick` | C++ | Engine de chute (ROS2 Action) |
| `edrom_imu_driver` | Python | Leitura do IMU via serial |
| `vision_controller` | Python | Controle de cabeça (busca + rastreamento) |

---

## Walking Engine

**Pacote:** `aurea_walk`
**Arquivo:** `src/edrom_motion/aurea_walk/src/walking_engine_node.cpp`

### Launch

```bash
ros2 launch aurea_walk walking.launch.py

# Com teleop habilitado
ros2 launch aurea_walk walking.launch.py teleop:=True
```

### Estados de Caminhada

| Estado | Descrição |
|--------|-----------|
| `IDLE` | Parado, sem movimento |
| `WALKING` | Caminhando |
| `IDLE_MARCH` | Marchando no lugar |
| `STOPPING` | Desacelerando |
| `HOMING` | Retornando à pose inicial |

### Interface ROS2

| Direção | Tópico/Serviço | Tipo |
|---------|----------------|------|
| Subscreve | `/cmd_vel` | `geometry_msgs/Twist` |
| Subscreve | `/stop_walking` | `std_msgs/Empty` |
| Publica | `/goal_joint_states` | `sensor_msgs/JointState` |
| Serviço cliente | `/solve_ik` | `aurea_walk/srv/SolveIK` |

### Parâmetros Principais

```yaml
# Cinemática
step_period: 0.6            # Tempo de um passo completo (s)
com_height: 0.22            # Altura do centro de massa (m)
step_height: 0.026          # Altura máxima do pé em swing (m)
double_support_ratio: 0.15  # Proporção de suporte duplo
feet_separation: 0.0425     # Distância lateral entre pés (m)

# Controle
update_frequency: 100.0     # Frequência da malha de controle (Hz)
backlash_offset_hp: -0.1    # Compensação de folga no hip_pitch

# Ganhos de compensação
kp_gain_hip_roll: 5.0
kp_gain_hip_pitch: -5.0     # Negativo: direção inversa
kp_gain_knee: 5.0

# Braços
arm_swing_amplitude: 0.4    # Amplitude de balanço dos braços (rad)
idle_shoulder_pitch: 0.7    # Pose idle: ombro pitch (rad)
idle_shoulder_roll: -1.4    # Pose idle: ombro roll (rad)
idle_elbow: -1.6            # Pose idle: cotovelo (rad)

# Retorno à pose
homing_duration: 2.0        # Duração do homing (s)
```

**Config:** `src/edrom_motion/aurea_walk/configs/walking_params.yaml`

### Fluxo de Controle (100 Hz)

```
1. cmd_vel_callback() — recebe velocidade desejada (vx, vy, vtheta)
2. main_loop():
   a. Atualiza posição dos pés (ZMP + marcha oscilante)
   b. Calcula posição alvo do torso
   c. Requisita IK para perna direita e esquerda (/solve_ik)
   d. Aguarda respostas do IK
   e. Aplica compensação de gravidade (ganhos PID)
   f. Adiciona balanço de braços sincronizado
   g. Publica em /goal_joint_states
```

### Comandos de Teste

```bash
# Enviar velocidade de caminhada
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.3}}"

# Parar imediatamente
ros2 topic pub /stop_walking std_msgs/msg/Empty "{}"
```

---

## Cinemática Inversa (IK)

**Arquivo:** `src/edrom_motion/aurea_walk/src/ik_node.cpp`

Resolve 6 DOF de cada perna analiticamente usando a biblioteca Eigen3.

### Serviço

```
Serviço: /solve_ik
Tipo:    aurea_walk/srv/SolveIK

Requisição:
  geometry_msgs/Pose target_pose   # Posição/orientação alvo do pé
  string leg_id                    # "direita" ou "esquerda"

Resposta:
  bool success
  sensor_msgs/JointState result_joint_state
```

### Exemplo de chamada

```bash
ros2 service call /solve_ik aurea_walk/srv/SolveIK \
  "{leg_id: 'direita', target_pose: {position: {x: 0.0, y: -0.04, z: -0.22}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

---

## Kick Engine

**Pacote:** `aurea_kick`
**Arquivo:** `src/edrom_motion/aurea_kick/src/kick_node.cpp`

Implementado como **ROS2 Action** (`aurea_kick::action::Kick`), permitindo feedback de progresso durante a execução.

### Fases do Chute

| Fase | Duração | Descrição |
|------|---------|-----------|
| A | 0.8 s | Deslocar ZMP para o pé de suporte |
| B | 0.4 s | Recuar o pé de chute |
| C | 0.2 s | **Chutar** (movimento rápido) |
| D | 0.4 s | Voltar pé de chute |
| E | 0.8 s | Centralizar ZMP — retorno à postura neutra |

**Total:** ~2.6 segundos por chute

**Parâmetros:**

```yaml
x_amplitude: 0.08     # Distância de recuo/avanço do pé (m)
z_height: 0.04        # Altura do pé durante o chute (m)

kp_gain_hip_roll: 1.0
kp_gain_hip_pitch: -10.0
kp_gain_knee: 10.0
backlash_hip_offset: -0.2
```

---

## IMU Driver

**Pacote:** `edrom_imu_driver` (Python)

Lê dados do IMU via comunicação serial com um Arduino.

**Publicações:**

- Aceleração linear (m/s²)
- Velocidade angular (rad/s)
- Orientação (quaternion)

Usado pelo `fall_interpreter.py` no módulo de comportamento para detectar quedas.

---

## Vision Controller (Controle de Cabeça)

**Arquivo:** `src/edrom_motion/vision_controller/vision_controller/search_and_track_node.py`

### Estados

| Estado | Descrição |
|--------|-----------|
| `IDLE` | Cabeça parada |
| `SEARCHING` | Varre o campo procurando a bola |
| `TRACKING` | Rastreia a bola detectada |

### Parâmetros

```yaml
image_width: 640
image_height: 480
kp_head_pan: 0.0007       # Ganho proporcional pan
kp_head_tilt: 0.0007      # Ganho proporcional tilt
pan_speed_rad_s: 0.8      # Velocidade de varredura
pan_min_limit_rad: -1.3
pan_max_limit_rad: 1.3
tilt_initial_rad: 0.0
tilt_min_limit_rad: -0.52
tilt_max_limit_rad: 0.52
```

### Algoritmo

```
Modo SEARCHING:
  Varre head_pan de -1.3 até +1.3 rad
  A cada extremo, incrementa head_tilt em 0.26 rad

Modo TRACKING:
  Erro pan  = center_x − bola_x
  Erro tilt = center_y − bola_y
  goal_pan  = atual_pan  + Kp × erro_pan
  goal_tilt = atual_tilt + Kp × erro_tilt
```

### Interface ROS2

| Direção | Tópico | Tipo |
|---------|--------|------|
| Subscreve | `vision2BhvTopic` | `VisionData` (edrom_msgs) |
| Publica | `/goal_joint_states` | `sensor_msgs/JointState` |

---

## Gerador de Trajetórias

**Arquivo:** `src/edrom_motion/aurea_walk/src/trajectory_generator.cpp`

Gera trajetórias suaves entre poses usando:

- Splines cúbicas
- Interpolação de Bezier
- Sincronização temporal entre segmentos
