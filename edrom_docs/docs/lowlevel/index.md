# Low Level — Controle de Hardware

O pacote `edrom_lowlevel` é responsável pela comunicação direta com os motores Dynamixel via USB.
É implementado em C++ para garantir baixa latência na malha de controle.

**Pacote:** `src/edrom_lowlevel/`

## Executáveis

| Executável | Função |
|-----------|--------|
| `direct_controller` | Controla todos os motores via GroupSyncWrite |
| `robot_initializer` | Calcula e aplica pose inicial via IK |
| `head_reader` | Lê posição atual da cabeça |

## Launch

```bash
ros2 launch edrom_lowlevel direct_control.launch.py
```

---

## Direct Controller

**Arquivo:** `src/edrom_lowlevel/src/direct_controller.cpp`

**Config:** `src/edrom_lowlevel/config/motors_direct.yaml`

### Interface ROS2

| Direção | Tópico | Tipo |
|---------|--------|------|
| Subscreve | `/goal_joint_states` | `sensor_msgs/JointState` |

### Comunicação Serial

```yaml
usb_port: "/dev/ttyUSB0"
baud_rate: 1000000  # 1 Mbps
```

### Fluxo de Dados

```
/goal_joint_states
       │
  direct_controller
       │
  GroupSyncWrite ──► Protocolo 1.0 (AX-12A) ──► Cabeça (IDs 1, 2)
       │
       └────────────► Protocolo 2.0 (MX-106) ──► Corpo (IDs 3-20)
```

### Operação

A cada mensagem recebida em `/goal_joint_states`:

1. Limpa os buffers dos dois grupos sync
2. Converte radianos para valor de posição do motor
3. Adiciona offset de calibração
4. Aplica inversão de sinal se necessário
5. Executa `txPacket()` em ambos os grupos simultaneamente

---

## Robot Initializer

**Arquivo:** `src/edrom_lowlevel/src/robot_initializer.cpp`

Coloca o robô em uma posição inicial segura ao inicializar o sistema.

### Sequência de Inicialização

1. Aguarda o serviço `/solve_ik` ficar disponível
2. Requisita IK para perna direita: posição `(0, -feet_sep, 0)`
3. Requisita IK para perna esquerda: posição `(0, +feet_sep, 0)`
4. Aplica offset de -0.2 rad no `hip_pitch`
5. Publica resultado em `/goal_joint_states`

### Parâmetros

```yaml
initial_com_height: 0.22      # Altura do centro de massa em metros
initial_feet_separation: 0.0465  # Separação lateral dos pés em metros
```

---

## Hardware Interface (Opcional)

**Arquivo:** `src/edrom_lowlevel/include/edrom_lowlevel/aurea_hardware_interface.hpp`

Implementa `SystemInterface` do `ros2_control`. Disponível para integração futura com controllers do ROS2 Control.

---

## Calibração dos Motores

Para calibrar um motor específico, edite `motors_direct.yaml`:

```yaml
joints:
  l_knee:
    id: 15
    protocol: 2.0
    inverted: false
    calibration_offset: -0.15  # Ajuste em radianos
```

**Processo:**

1. Colocar o robô em pose plana (posição de referência)
2. Ajustar `calibration_offset` para cada junta que precisar
3. Salvar o arquivo e reiniciar o `direct_controller`
4. Verificar visualmente a pose resultante
