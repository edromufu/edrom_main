# Simulation — Webots

O ambiente de simulação utiliza o **Webots R2025a** integrado ao ROS2 via `webots_ros2_driver`.
Permite testar comportamentos, movimentos e a máquina de estados sem o hardware físico.

## Pacotes

| Pacote | Descrição |
|--------|-----------|
| `src/edrom_simulation/aurea_webots/` | Integração ROS2-Webots (Python) |
| `src/behaviour/bhv_simulator/` | Simulador de comportamento completo |

---

## Instalação do Webots

O Webots é instalado via `Makefile` na raiz do projeto:

```bash
# Instalação completa (download + extração + configuração de PATH)
make install

# Apenas download
make download

# Apenas extração (específica de versão)
make extract WEBOTS_VERSION=R2025a

# Apenas configurar PATH no .bashrc
make configure
```

Após a instalação, reinicie o terminal:

```bash
source $HOME/.bashrc
webots  # verificar instalação
```

**Versão padrão:** R2025a
**Diretório de instalação:** `/usr/local/webots`

---

## Executar a Simulação

```bash
ros2 launch bhv_simulator behaviour_simulator.launch.py
```

**Arquivo:** `src/behaviour/bhv_simulator/launch/behaviour_simulator.launch.py`

**O que acontece ao executar:**

1. `SetEnvironmentVariable` — configura PYTHONPATH para os controllers Webots
2. `ExecuteProcess` — abre o Webots com o arquivo de mundo `.wbt`

---

## Mundo Webots

**Arquivo:** `src/behaviour/bhv_simulator/worlds/bhv_sim_world.wbt`

**Componentes do mundo:**

- Campo de futebol com marcações
- Robô Aurea (carregado via URDF)
- Bola
- Gols (esquerdo e direito)

---

## Descrição do Aurea para Webots

| Arquivo | Descrição |
|---------|-----------|
| `src/edrom_simulation/aurea_webots/resource/aurea_urdf_pkg.urdf` | URDF do Aurea para simulação |
| `src/edrom_simulation/aurea_webots/protos/Aurea.proto` | Proto Webots do robô |

As malhas STL são as mesmas do URDF físico.

---

## Controllers Webots

**Diretório:** `src/behaviour/bhv_simulator/controllers/`

Scripts Python que implementam a lógica de controle do robô diretamente dentro do ambiente Webots, permitindo simular sensores e atuadores.

---

## Tópicos ROS2 na Simulação

Os mesmos tópicos do hardware real são utilizados na simulação, permitindo usar o mesmo código de comportamento:

```bash
# Enviar velocidade para o robô simulado
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Parar caminhada
ros2 topic pub /stop_walking std_msgs/msg/Empty "{}"

# Ver joint states
ros2 topic echo /goal_joint_states
```

---

## Para mais informações

Documentação oficial do Webots: [cyberbotics.com/doc/guide/index](https://cyberbotics.com/doc/guide/index)
