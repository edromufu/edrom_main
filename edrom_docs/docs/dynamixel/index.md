# Dynamixel — Servos e Comunicação

O Aurea utiliza servos Dynamixel com dois protocolos distintos:

- **Protocolo 1.0** — motores AX-12A (cabeça)
- **Protocolo 2.0** — motores MX-106 (corpo inteiro)

**SDK:** `src/DynamixelSDK/` (SDK oficial Robotis para Linux)

---

## Comunicação Serial

```yaml
Porta:    /dev/ttyUSB0  (conversor USB-Serial FTDI)
Baudrate: 1000000 bps   (1 Mbps)
```

---

## Protocolo 1.0 — AX-12A (Cabeça)

| Propriedade | Valor |
|-------------|-------|
| Range de posição | 0 – 1023 |
| Cobertura angular | 300° |
| Tamanho do dado | 2 bytes |
| `ADDR_TORQUE_ENABLE` | 24 |
| `ADDR_GOAL_POSITION` | 30 |
| `ADDR_PRESENT_POSITION` | 36 |

**Conversão:**

```
rad → valor:  value = (rad + 150°) × (1023 / 300°)
valor → rad:  rad   = (value × 300° / 1023) − 150°
```

---

## Protocolo 2.0 — MX-106 (Corpo)

| Propriedade | Valor |
|-------------|-------|
| Range de posição | 0 – 4095 |
| Cobertura angular | 360° |
| Tamanho do dado | 4 bytes |
| `ADDR_TORQUE_ENABLE` | 64 |
| `ADDR_GOAL_POSITION` | 116 |
| `ADDR_PRESENT_POSITION` | 132 |

**Conversão:**

```
rad → valor:  value = (rad + π) × (4095 / 2π)
valor → rad:  rad   = (value × 2π / 4095) − π
```

---

## Mapeamento Completo de Motores

| Junta | ID | Protocolo | Invertido | Offset Calib. |
|-------|----|-----------|-----------|---------------|
| `head_pan` | 1 | 1.0 | Não | 0.0 |
| `head_tilt` | 2 | 1.0 | Não | 0.0 |
| `l_sho_pitch` | 7 | 2.0 | Não | 0.0 |
| `l_sho_roll` | 5 | 2.0 | Sim | 0.0 |
| `l_el` | 3 | 2.0 | Não | 0.0 |
| `r_sho_pitch` | 8 | 2.0 | Sim | 0.0 |
| `r_sho_roll` | 6 | 2.0 | Sim | 0.0 |
| `r_el` | 4 | 2.0 | Não | 0.0 |
| `r_hip_yaw` | 10 | 2.0 | Não | 0.0 |
| `r_hip_roll` | 12 | 2.0 | Sim | **+0.07** |
| `r_hip_pitch` | 14 | 2.0 | Não | 0.0 |
| `r_knee` | 16 | 2.0 | Sim | 0.0 |
| `r_ank_pitch` | 18 | 2.0 | Sim | 0.0 |
| `r_ank_roll` | 20 | 2.0 | Não | 0.0 |
| `l_hip_yaw` | 9 | 2.0 | Não | 0.0 |
| `l_hip_roll` | 11 | 2.0 | Sim | **-0.075** |
| `l_hip_pitch` | 13 | 2.0 | Sim | **+0.04** |
| `l_knee` | 15 | 2.0 | Não | **-0.15** |
| `l_ank_pitch` | 17 | 2.0 | Sim | 0.0 |
| `l_ank_roll` | 19 | 2.0 | Não | 0.0 |

**Config:** `src/edrom_lowlevel/config/motors_direct.yaml`

---

## Sequência de Inicialização

```
1. Abre porta serial (/dev/ttyUSB0)
2. Define baudrate (1 Mbps)
3. Para cada motor: habilita torque (escreve 1 em ADDR_TORQUE_ENABLE)
4. Cria dois GroupSyncWrite (um para cada protocolo)
```

## Operação Contínua (por ciclo)

```
1. Limpa parâmetros dos dois grupos sync
2. Para cada junta:
   a. Converte radianos → valor do motor
   b. Aplica calibration_offset
   c. Aplica inversão de sinal (se inverted: true)
   d. Adiciona ao grupo sync correspondente
3. Executa txPacket() nos dois grupos
```

---

## Calibração

Para ajustar offsets de calibração, edite `motors_direct.yaml`:

```yaml
joints:
  l_knee:
    id: 15
    protocol: 2.0
    inverted: false
    calibration_offset: -0.15   # valor em radianos
```

Reinicie o `direct_controller` após salvar as alterações.

---

## Resolução de Problemas

**Motor não responde:**
```bash
# Verificar se a porta USB está disponível
ls /dev/ttyUSB*

# Permissão de acesso
sudo chmod 666 /dev/ttyUSB0
# ou adicione seu usuário ao grupo dialout:
sudo usermod -aG dialout $USER
```

**Verificar comunicação:**
```bash
# Ver se goal_joint_states está chegando
ros2 topic echo /goal_joint_states
```
