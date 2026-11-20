# bhv_simulator — teste cinemático de trajetórias

O Supervisor lê bola, corpo e obstáculos no Webots. Um nó ROS chama o planejador
copiado de ThethaStar e segue o caminho com `/cmd_vel`. Não depende do filtro de
partículas nem de percepção por câmera.

## Executar

Na raiz de `edrom_main`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select bhv_simulator
source install/setup.bash
ros2 launch bhv_simulator behaviour_simulator.launch.py
```

Não é mais necessário compilar/instalar `thetastar` separadamente. Use a instalação
normal: `--symlink-install` é incompatível com o setuptools atual deste ambiente.
O Webots deve estar no PATH; `WEBOTS_HOME` usa `/usr/local/webots` por padrão.

A câmera e os sensores antigos ficam desligados no teste de trajetórias, evitando
renderizar/copiar imagens de 416×416 a cada passo. Para ativá-los:

```bash
ros2 launch bhv_simulator behaviour_simulator.launch.py sensors:=true
```

Para comandar manualmente ou iniciar outro planejador, use `planning:=false`.
Não rode simultaneamente `thetastar main_ros`: haveria dois publicadores de comandos.
O launch usa modo `realtime` e o seguimento usa o relógio da simulação.

## Código e parâmetros

- `planning/know.py`, `planning/campoVetorial.py` e
  `planning/thtastart_v2_p1inguin.py`: cópias **sem alterações** dos arquivos de
  `/home/vtr_caixeta/ThethaStar` disponíveis na atualização desta integração.
- `trajectory_planner.py`: lê ROS, chama o adaptador e publica caminho/velocidades.
- `trajectory_navigation.py`: fornece dados ao `HybridPlanner.gerar_caminho_theta()`
  e segue os waypoints. Não implementa outra versão de Theta* ou APF.
- `field_bridge.py`: posições globais e aplicação cinemática de velocidades.

A cópia preserva inclusive o custo APF igual a `1` presente no `know.py` de origem.
Logo, nessa versão o corredor APF não tem desconto em relação ao restante da grade.
Para atualizar o algoritmo após editar o repositório original:

```bash
cp ../ThethaStar/{know.py,campoVetorial.py,thtastart_v2_p1inguin.py} \
  src/behaviour/bhv_simulator/bhv_simulator/planning/
colcon build --packages-select bhv_simulator
```

O torso tem footprint de 0,20×0,11 m. A integração usa `robot_radius=0.12` m e
`safety_margin=0.05` m; o algoritmo copiado ainda adiciona sua própria folga de
discretização. Antes o raio configurado era 0,25 m e a margem 0,10 m, fechando
passagens disponíveis ao corpo simplificado. Adapte esses valores se mudar o robô.
O parâmetro `max_linear_speed` vale 0,5 m/s. Para sobrescrever parâmetros, execute
com `planning:=false` e, em outro terminal:

```bash
ros2 run bhv_simulator trajectory_planner --ros-args -p use_sim_time:=true \
  -p robot_radius:=0.12 -p safety_margin:=0.05 -p max_linear_speed:=0.5
```

O controle roda a 20 Hz de simulação, sem guardar uma fila de comandos antigos.
A busca ocorre quando a cena muda, o caminho fica bloqueado ou o robô sai do
segmento. Variações acumuladas abaixo de 3 cm na bola/2 cm nos obstáculos não
disparam busca, mas os obstáculos atuais são sempre usados na verificação de
colisão. Busca limitada a 2 Hz; enquanto precisa recalcular, o comando é zero.
O campo potencial não é executado repetidamente a cada ciclo de seguimento.

A aproximação para a 25 cm da bola. Verifica-se também o arco de movimento durante
o timeout de 0,3 s dos comandos. Dados antigos/ausentes ou rota inválida param o
robô. O watchdog de comunicação usa tempo monotônico e funciona com Webots pausado.

## Mundo e referenciais

O mundo usa Y vertical (`NUE`): `map.x=-webots.z`, `map.y=-webots.x`,
`map.z=webots.y`. Posições em metros, orientação em radianos.
`Robot3D` aponta para -Z local; yaw zero aponta para +X em `map`, em direção à bola
inicial. A barra azul indica essa frente. O corpo começa com yaw zero.

O Robot interno não tem dinâmica independente: o corpo visível acompanha o
Transform cuja pose o Supervisor publica. Antes havia um Robot com física dentro
do Transform teletransportado, podendo se mover/inclinar independentemente.
O movimento continua simplificado: não há marcha, equilíbrio, chute ou resposta
física do corpo às colisões; a prevenção é feita pelo planejador/controlador.

Os obstáculos são cilindros estáticos `Obstacle1`, `Obstacle2`, `Obstacle3`, com
raio 0,2 m. As posições e dimensões são lidas do mundo, não fixadas no algoritmo.
Reposicione-os na interface do Webots; reinicie o controlador se adicionar/remover
DEFs. Bola e corpo mantêm os nomes `Ball` e `Robot3D`.

| Tópico | Tipo |
| --- | --- |
| `/simulation/robot_pose` | `geometry_msgs/PoseStamped` |
| `/simulation/ball_position` | `geometry_msgs/PointStamped` |
| `/simulation/obstacles` | `visualization_msgs/MarkerArray` |
| `/planning/path` | `nav_msgs/Path` |
| `/cmd_vel` | `geometry_msgs/Twist` |
| `/clock` | `rosgraph_msgs/Clock` |

Todos os dados globais usam `map`. Velocidade X é para frente; angular Z positiva
é anti-horária. Para comparar rota calculada e posição real, visualize Path e
PoseStamped no RViz com frame fixo `map`. Nenhum TF adicional é publicado.

## Verificação

```bash
PYTHONPATH=src/behaviour/bhv_simulator:$PYTHONPATH \
  python3 -m pytest src/behaviour/bhv_simulator/test -q
python3 src/behaviour/bhv_simulator/controllers/bhv_sim/bhv_sim.py --check-imports
```

O controlador carrega ROS e procura `install/setup.bash` nos diretórios ancestrais.
O mundo precisa ficar ao lado de `controllers/`; copiar só o `.wbt` perde a
associação com `controllers/bhv_sim/bhv_sim.py`.

Medição comparativa no Webots desta máquina, no cenário inicial (uma execução
por versão, excluindo a inicialização da janela): antes, RTF ≈0,23 e primeira
rota de 4,68 m; depois, RTF ≈0,91 e rota de 3,31 m. A versão otimizada chegou a
26 cm da bola em 7,84 s simulados / 8,66 s reais. Esses valores dependem do
hardware/carga; não são um benchmark geral do algoritmo. A primeira execução
chegou a falhar ao gravar o relatório por falta de espaço em disco, depois de
emitir as medidas no terminal. Os testes numéricos verificam separadamente a
folga dos obstáculos e o seguimento sem voltas completas.
