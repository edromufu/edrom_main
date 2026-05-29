# Documentação DSD — Sistema de Comportamento do Robô

## O que é DSD

DSD (Dynamic Stack-based Decision) é uma arquitetura de comportamento reativa onde decisões e ações são empilhadas dinamicamente. Ao contrário de máquinas de estado tradicionais com transições fixas, no DSD o estado atual é o topo de uma pilha que é reavaliada a cada tick. Isso garante que mudanças críticas no mundo (queda, perda de bola) sejam tratadas imediatamente, independente de onde o robô estava na sequência de comportamentos.

---

## Estrutura de Arquivos

```
src/behaviour/
├── transitions_and_states/          # Pacote principal do comportamento
│   ├── transitions_and_states/
│   │   ├── dsd_state_machine.py     # Lógica pura da DSD (sem ROS)
│   │   ├── dsd_node.py              # Nó ROS2 que hospeda a DSD
│   │   └── behaviour_node.py        # Implementação legada (pré-DSD)
│   ├── launch/
│   │   ├── behaviour_dsd.launch.py  # Launch principal (DSD)
│   │   └── behaviour.launch.py      # Launch legado
│   ├── setup.py
│   └── package.xml
└── modularized_bhv_msgs/            # Pacote de mensagens ROS
    ├── msg/
    │   ├── StateMachineMsg.msg      # Entrada de sensores para a DSD
    │   └── CurrentStateMsg.msg      # Saída de estado da DSD
    └── srv/
        └── MoveRequest.srv
```

---

## WorldState — Estado do Mundo

`WorldState` é um dataclass que representa tudo que a DSD precisa saber sobre o ambiente. É atualizado pelo nó ROS a cada mensagem de sensor e lido pela DSD a cada tick.

| Campo | Tipo | Descrição |
|---|---|---|
| `ball_position` | `BallPosition` | Posição da bola na visão: `NONE`, `CENTER`, `LEFT_TOP`, `LEFT_BOTTOM`, `RIGHT_TOP`, `RIGHT_BOTTOM` |
| `ball_close` | `bool` | Bola está próxima o suficiente para chutar |
| `ball_found` | `bool` | Bola está sendo detectada pela visão |
| `ball_angle` | `float` | Ângulo pan da cabeça, usado como proxy do ângulo da bola |
| `fall_state` | `FallState` | Estado de queda: `UP`, `FRONT`, `BACK`, `LEFT`, `RIGHT` |
| `recovering` | `bool` | Robô está em processo de recuperação de queda |
| `hor_motor_out_of_center` | `HeadHorizontalPosition` | Posição horizontal da cabeça: `CENTER`, `LEFT`, `RIGHT` |
| `trunk_aligned` | `bool` | Tronco alinhado com a bola para o chute |
| `head_kick_check` | `bool` | Cabeça em inclinação adequada para o chute |
| `kick_done` | `bool` | Confirmação de que o chute foi executado |

---

## Elementos da DSD

Todos os elementos herdam de `StackElement`, que define `on_enter` e `on_exit` (hooks opcionais). Há dois tipos:

### DecisionElement
Avalia o mundo e retorna uma string que determina qual elemento entra na pilha. Reavaliado a cada tick (por padrão).

### ActionElement
Executa uma ação no mundo. Permanece no topo da pilha enquanto retorna `"RUNNING"`. Ao retornar `"DONE"` é removido da pilha. Pode declarar `do_not_reevaluate() → True` para bloquear interrupções (usado no chute).

---

## Decisões

### `RootDecision`
Ponto de entrada da pilha. Verifica segurança antes de qualquer outra coisa.

- `fall_state != UP` ou `recovering` → `"RECOVERY"`
- Caso contrário → `"STANDING_OK"`

### `BallFoundDecision`
Verifica se a visão está detectando a bola.

- `ball_found = True` → `"YES"`
- `ball_found = False` → `"NO"`

### `BallCloseDecision`
Verifica se a bola está perto o suficiente para iniciar o alinhamento de chute.

- `ball_close = True` → `"YES"`
- `ball_close = False` → `"NO"`

### `TrunkAlignedDecision`
Verifica se o tronco está alinhado com a bola.

- `trunk_aligned = True` → `"YES"`
- `trunk_aligned = False` → `"NO"`

### `HeadKickCheckDecision`
Verifica se a cabeça está na inclinação correta para o chute.

- `head_kick_check = True` → `"YES"`
- `head_kick_check = False` → `"NO"`

### `HorMotorDecision` / `BallPositionDecision`
Auxiliares (atualmente não conectados ao fluxo principal do `map_decision`). Disponíveis para expansão futura.

---

## Ações

### `RecoveryAction`
Bloqueia toda locomoção enquanto o robô está caído. Seta `recovering = True` ao detectar queda e `recovering = False` quando o robô volta a `FallState.UP`. Retorna `"DONE"` somente após a recuperação completa.

### `SearchBallAction`
Gira o robô para procurar a bola com base em `hor_motor_out_of_center`. Se a cabeça estiver à esquerda, gira no sentido anti-horário; se à direita, gira no sentido horário. Termina quando `ball_found = True`.

### `GoToBallAction`
Move o robô em direção à bola usando `ball_position` para correção lateral e `ball_angle` para controle preciso quando a bola está centralizada. Termina quando `ball_close = True` ou a bola é perdida.

### `AlignTrunkAction`
Alinha o tronco do robô com a bola antes do chute. Fica em `"RUNNING"` até `trunk_aligned = True`. Pode ser interrompida por queda.

### `AlignHeadForKickAction`
Ajusta a inclinação da cabeça para a posição correta de chute. Fica em `"RUNNING"` até `head_kick_check = True`. Pode ser interrompida por queda.

### `KickAction`
Executa o chute. Declara `do_not_reevaluate() = True`, ou seja, **não pode ser interrompida** por mudanças no mundo durante a execução. Termina quando `kick_done = True`.

### `DoneAction`
Ação terminal. Fica em `"RUNNING"` indefinidamente (missão concluída).

---

## Fluxo de Decisão Completo

```
RootDecision
├── RECOVERY ──────────────────────→ RecoveryAction
│                                     (bloqueia até robô se levantar)
└── STANDING_OK
    └── BallFoundDecision
        ├── NO ────────────────────→ SearchBallAction
        │                            (gira procurando a bola)
        └── YES
            └── BallCloseDecision
                ├── NO ────────────→ GoToBallAction
                │                    (caminha até a bola)
                └── YES
                    └── TrunkAlignedDecision
                        ├── NO ────→ AlignTrunkAction
                        │            (alinha tronco com a bola)
                        └── YES
                            └── HeadKickCheckDecision
                                ├── NO ──→ AlignHeadForKickAction
                                │          (ajusta inclinação da cabeça)
                                └── YES
                                    └── KickAction
                                         (chuta; não interrompível)
```

**Ordem de alinhamento para o chute:** tronco primeiro (`AlignTrunkAction`), depois cabeça (`AlignHeadForKickAction`).

---

## Motor da DSD — `SimpleDSD`

### Inicialização
Instancia todos os elementos (decisões e ações), empilha `RootDecision` e chama `expand_top_decisions()` para expandir decisões encadeadas até chegar numa ação.

### `expand_top_decisions()`
Enquanto o topo da pilha for uma `DecisionElement`, avalia-a e empilha o próximo elemento retornado por `map_decision`. Garante que a pilha sempre termine com uma `ActionElement` no topo.

### `reevaluate()`
A cada tick, percorre a pilha de baixo para cima procurando decisões cujo resultado mudou desde a última avaliação. Na primeira mudança encontrada, descarta todos os elementos acima dela, insere o novo elemento mapeado e chama `expand_top_decisions()`. Ações com `do_not_reevaluate() = True` bloqueiam esta etapa completamente.

### `step()`
Sequência de um tick:
1. Chama `reevaluate()` para detectar mudanças no mundo
2. Executa `perform()` da ação no topo
3. Se retornar `"DONE"`, remove da pilha e chama `expand_top_decisions()`

### `map_decision(decision, result)`
Função de mapeamento puro: dado uma decisão e seu resultado, retorna o próximo elemento a ser empilhado. Toda a estrutura do grafo de comportamento está aqui.

---

## Integração ROS2 — `dsd_node.py`

### Parâmetros

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `tick_hz` | `20.0` | Frequência de execução da DSD (Hz) |
| `use_getup_srv` | `false` | Reservado para integração com serviço de levantamento |

### Tópicos Subscritos

| Tópico | Tipo | Descrição |
|---|---|---|
| `/sensor_observer/state_machine_vars` | `StateMachineMsg` | Variáveis de sensor do mundo; atualiza WorldState a cada mensagem |
| `/goal_joint_states` | `JointState` | Posição do motor pan da cabeça; atualiza `ball_angle` |
| `/kick_done` | `Bool` | Confirmação de chute concluído; atualiza `kick_done` |

### Tópicos Publicados

| Tópico | Tipo | Frequência | Descrição |
|---|---|---|---|
| `/cmd_vel` | `Twist` | 20 Hz | Comandos de velocidade linear e angular |
| `/head_control/state` | `String` | 20 Hz | Modo da cabeça: `IDLE`, `SEARCHING`, `TRACKING` |
| `/transitions_and_states/state_machine` | `CurrentStateMsg` | 20 Hz | Estado legado (compatibilidade com rotinas) |
| `/transitions_and_states/dsd_state` | `CurrentStateMsg` | 20 Hz | Nome da ação ativa (debug) |

### Mapeamento de Comandos por Ação Ativa

| Ação | `cmd_vel` | `head_control` |
|---|---|---|
| `SearchBallAction` | Gira com base em `hor_motor_out_of_center` (±0.4 rad/s) | `SEARCHING` |
| `GoToBallAction` | Avança (0.15 m/s) + corrige lateral com base em `ball_position` | `TRACKING` |
| `AlignTrunkAction` | Parado (0, 0) | `TRACKING` |
| `AlignHeadForKickAction` | Parado (0, 0) | `TRACKING` |
| `KickAction` | Parado | `IDLE` |
| `RecoveryAction` / `StandUpAction` | Parado | `IDLE` |

### Estado Legado
O campo `current_state` publicado em `/transitions_and_states/state_machine` usa strings compatíveis com as rotinas existentes:

| Ação DSD | Estado Legado |
|---|---|
| `RecoveryAction`, `StandUpAction` | `getting_up` |
| `SearchBallAction` | `searching` |
| `GoToBallAction` | `walking` |
| `AlignTrunkAction`, `AlignHeadForKickAction` | `aligning_body` |
| `KickAction` | `kicking` |
| `DoneAction` | `idle` |

---

## Mensagens ROS — `modularized_bhv_msgs`

### `StateMachineMsg.msg`
Publicada pelo `ros_packer` (pacote `sensor_observer`). Contém todas as variáveis do mundo necessárias para a DSD.

```
string ball_position          # "none", "Center", "Left Top", "Left Bottom", "Right Top", "Right Bottom"
bool   ball_close             # bola próxima para chute
bool   ball_found             # bola detectada pela visão
string fall_state             # "Up", "Front", "Back", "Left", "Right"
string hor_motor_out_of_center # "Center", "Left", "Right"
bool   head_kick_check        # cabeça em posição de chute
bool   trunk_aligned          # tronco alinhado com a bola
```

### `CurrentStateMsg.msg`
Publicada pela DSD para informar o estado atual para outros pacotes.

```
string current_state
```

---

## Launch — `behaviour_dsd.launch.py`

```bash
ros2 launch transitions_and_states behaviour_dsd.launch.py [args]
```

| Argumento | Padrão | Descrição |
|---|---|---|
| `simulation` | `false` | Inclui `bhv_simulator` se `true` |
| `tick_hz` | `20.0` | Frequência do loop DSD |
| `use_getup_srv` | `false` | Habilita serviço de levantamento |
| `imu_connected` | `false` | Inclui nó IMU se `true` |
| `imu_port` | `/dev/ttyIMU` | Porta serial do IMU |

Nós iniciados pelo launch:

- `dsd_behavior` — nó principal DSD (`transitions_and_states/dsd_node`)
- `ros_packer` — empacota dados de sensor em `StateMachineMsg` (`sensor_observer`)
- `kicking_routine` — escuta o estado `kicking` e dispara o chute (`states_routine`)
- `imu_read` — leitura do IMU (opcional, `imu_ros_arduino`)

---

## Dependências entre Pacotes

```
sensor_observer (ros_packer)
        │  publica StateMachineMsg
        ▼
transitions_and_states (dsd_node)
        │  publica cmd_vel, head_control/state, CurrentStateMsg
        ▼
states_routine (kicking_routine)   ←── escuta CurrentStateMsg "kicking"
locomotion / head_control          ←── escuta cmd_vel / head_control/state
```

---

## Como Adicionar um Novo Comportamento

1. **Criar a Decision ou Action** em `dsd_state_machine.py` herdando de `DecisionElement` ou `ActionElement`.
2. **Instanciar** o novo elemento no `__init__` de `SimpleDSD`.
3. **Conectar** ao grafo dentro de `map_decision` adicionando o bloco `if isinstance(decision, ...)` adequado.
4. **Se for uma Action**, adicionar o tratamento de comando no `_tick_cb` de `dsd_node.py` (bloco `if top_name == "...":`).
5. **Se publicar estado legado**, adicionar o mapeamento em `action_name_to_legacy_state`.
6. **Atualizar `StateMachineMsg.msg`** se a nova condição precisar de uma variável nova, e refletir a leitura em `_sensor_cb`.
