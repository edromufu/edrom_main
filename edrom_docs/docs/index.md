# EDROM Documentation

**EDROM** é um robô humanóide desenvolvido para a **RoboCup Humanoid Soccer League**.
O robô se chama **Aurea** e utiliza ROS2 Humble como middleware de robótica.

---

## Visão Geral do Sistema

| Componente | Tecnologia | Linguagem |
|------------|-----------|-----------|
| Walking Engine + IK | ROS2 (aurea_walk) | C++ |
| Kick Engine | ROS2 (aurea_kick) | C++ |
| Low Level / Hardware | ROS2 (edrom_lowlevel) | C++ |
| Máquina de Estados | ROS2 (transitions_and_states) | Python |
| Visão Computacional | YOLOv8 (object_finder) | Python |
| Simulação | Webots R2025a | Python |

## Fluxo de Dados Principal

```
Câmera
  └─► object_finder (YOLOv8) ──► VisionData
                                      │
                               behaviour_node
                                      │
                                state_machine ──► cmd_vel / rotinas
                                      │
                              walking_engine ──► /solve_ik (serviço)
                                      │
                              goal_joint_states
                                      │
                             direct_controller
                                      │
                               Dynamixel USB
                                      │
                                  Motores
```

## Build e Setup

```bash
cd edrom_main
colcon build --symlink-install
source install/setup.bash
```

## Inicialização Rápida

```bash
# Sistema completo
ros2 launch edrom_bringup robot.launch.py

# Apenas simulação
ros2 launch bhv_simulator behaviour_simulator.launch.py
```

---

*Esta documentação está em constante atualização.*
