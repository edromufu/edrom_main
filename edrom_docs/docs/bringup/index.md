# Bring UP — Inicialização do Robô

O pacote `edrom_bringup` é o orquestrador central que inicializa todos os subsistemas do Aurea em sequência.

## Comando Principal

```bash
ros2 launch edrom_bringup robot.launch.py
```

**Arquivo:** `src/edrom_bringup/launch/robot.launch.py`

## Argumentos de Launch

| Argumento | Padrão | Descrição |
|-----------|--------|-----------|
| `camera_idx` | `'2'` | Índice da câmera USB |
| `img_output` | `'False'` | Exibir saída de imagem (debug) |
| `imu_connected` | `'false'` | Habilita driver do IMU |

Exemplo com argumentos:

```bash
ros2 launch edrom_bringup robot.launch.py camera_idx:=0 img_output:=True imu_connected:=true
```

## Sequência de Inicialização

O launch file sobe os seguintes componentes em ordem:

### 1. Walking Engine & Motion (`aurea_walk`)

Launch: `walking.launch.py`

| Nó | Função |
|----|--------|
| `walking_engine_node` | Engine principal de caminhada (C++) |
| `ik_node` | Serviço de cinemática inversa (C++) |
| `kick_node` | Engine de chute (C++) |
| `teleop_kick_bridge` | Bridge para teleop (condicional) |

### 2. Low-Level Control (`edrom_lowlevel`)

Launch: `direct_control.launch.py`

| Nó | Função |
|----|--------|
| `direct_controller` | Comunica com motores Dynamixel via USB |
| `robot_initializer` | Coloca robô em pose inicial via IK |

### 3. Comportamento (`transitions_and_states`)

Launch: `behaviour.launch.py`

| Nó | Função |
|----|--------|
| `behaviour_node` | Máquina de estados principal |
| `ros_packer` | Interpretador de sensores |
| `imu_ros_arduino` | Driver do IMU Arduino (condicional) |

### 4. Visão (`object_finder`)

Launch: `vision.launch.py`

| Nó | Função |
|----|--------|
| `object_finder` | Detecção de objetos com YOLOv8 |

### 5. Controle de Cabeça (`vision_controller`)

| Nó | Função |
|----|--------|
| `search_and_track_node` | Busca e rastreamento visual |

## Inicialização em Background

O arquivo `init_robot.py` na raiz do projeto inicia o robô em background e salva o PGID em `robot_pgid.txt` para posterior encerramento.

```bash
python3 init_robot.py
```

## Via Docker

```bash
# Subir container
docker compose up -d

# Entrar no container
docker compose exec ros2_dev /bin/bash

# Iniciar robô dentro do container
ros2 launch edrom_bringup robot.launch.py
```

## Monitoramento

```bash
# Ver todos os tópicos ativos
ros2 topic list

# Verificar frequência de publicação dos motores
ros2 topic hz /goal_joint_states

# Ver estado da visão
ros2 topic echo /vision2BhvTopic
```
