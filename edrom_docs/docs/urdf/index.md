# URDF — Modelo do Robô Aurea

O modelo cinemático e visual do Aurea é definido em URDF (Unified Robot Description Format),
exportado do SolidWorks e utilizado tanto para visualização no RViz2 quanto para a simulação no Webots.

## Arquivos

| Arquivo | Descrição |
|---------|-----------|
| `src/aurea_description/urdf/aurea_urdf_pkg.urdf` | URDF principal do Aurea (exportado SolidWorks, 1300+ linhas) |
| `src/aurea_description/config/joint_names_aurea_urdf_pkg.yaml` | Ordem das juntas para o controlador |
| `src/op3_description/urdf/robotis_op3.urdf.xacro` | URDF do OP3 (referência modular com xacro) |

## Visualização no RViz2

```bash
ros2 launch aurea_description visualizar.launch.py
```

**Arquivo:** `src/aurea_description/launch/visualizar.launch.py`

---

## Estrutura Cinemática (20 DOF)

### Cabeça (2 DOF)

```
base_link
  └─ head_pan_link   (ID 1) ── rotação horizontal
       └─ head_tilt_link  (ID 2) ── rotação vertical
```

### Braços (3 DOF cada)

```
base_link
  ├─ l_sho_pitch_link  (ID 7) ── pitch do ombro esquerdo
  │    └─ l_sho_roll_link  (ID 5) ── roll do ombro esquerdo
  │         └─ l_el_link  (ID 3) ── cotovelo esquerdo
  │
  └─ r_sho_pitch_link  (ID 8) ── pitch do ombro direito
       └─ r_sho_roll_link  (ID 6) ── roll do ombro direito
            └─ r_el_link  (ID 4) ── cotovelo direito
```

### Pernas (6 DOF cada)

```
base_link
  ├─ l_hip_yaw_link   (ID  9)
  │    └─ l_hip_roll_link   (ID 11)
  │         └─ l_hip_pitch_link  (ID 13)
  │              └─ l_knee_link  (ID 15)
  │                   └─ l_ank_pitch_link  (ID 17)
  │                        └─ l_ank_roll_link  (ID 19)
  │
  └─ r_hip_yaw_link   (ID 10)
       └─ r_hip_roll_link   (ID 12)
            └─ r_hip_pitch_link  (ID 14)
                 └─ r_knee_link  (ID 16)
                      └─ r_ank_pitch_link  (ID 18)
                           └─ r_ank_roll_link  (ID 20)
```

---

## Mapeamento Completo de Juntas

| # | Nome da Junta | ID | Protocolo |
|---|---------------|----|-----------|
| 1 | `head_pan` | 1 | 1.0 (AX) |
| 2 | `head_tilt` | 2 | 1.0 (AX) |
| 3 | `l_sho_pitch` | 7 | 2.0 (MX) |
| 4 | `l_sho_roll` | 5 | 2.0 (MX) |
| 5 | `l_el` | 3 | 2.0 (MX) |
| 6 | `r_sho_pitch` | 8 | 2.0 (MX) |
| 7 | `r_sho_roll` | 6 | 2.0 (MX) |
| 8 | `r_el` | 4 | 2.0 (MX) |
| 9 | `r_hip_yaw` | 10 | 2.0 (MX) |
| 10 | `r_hip_roll` | 12 | 2.0 (MX) |
| 11 | `r_hip_pitch` | 14 | 2.0 (MX) |
| 12 | `r_knee` | 16 | 2.0 (MX) |
| 13 | `r_ank_pitch` | 18 | 2.0 (MX) |
| 14 | `r_ank_roll` | 20 | 2.0 (MX) |
| 15 | `l_hip_yaw` | 9 | 2.0 (MX) |
| 16 | `l_hip_roll` | 11 | 2.0 (MX) |
| 17 | `l_hip_pitch` | 13 | 2.0 (MX) |
| 18 | `l_knee` | 15 | 2.0 (MX) |
| 19 | `l_ank_pitch` | 17 | 2.0 (MX) |
| 20 | `l_ank_roll` | 19 | 2.0 (MX) |

## Propriedades Físicas

- **Base (tronco):** massa 1.4383 kg
- **head_tilt:** 0.06657 kg
- Todos os links incluem malhas **STL** para geometria visual e de colisão

## Configuração do Controlador

```yaml
# src/aurea_description/config/joint_names_aurea_urdf_pkg.yaml
controller_joint_names: [
  '', 'head_pan', 'head_tilt',
  'l_sho_pitch', 'l_sho_roll', 'l_el',
  'r_sho_pitch', 'r_sho_roll', 'r_el',
  'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll',
  'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll'
]
```
