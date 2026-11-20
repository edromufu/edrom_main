# Leitor GameController

Protocolo UDP do guia “Leitura do GameController no ROS 2 Humble”: porta 3838,
cabeçalho RGme, versão 20, 158 bytes. O leitor usa a biblioteca padrão Python;
o envio de status RGrt não está implementado.

## Executar

Na raiz do workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select modularized_bhv_msgs transitions_and_states
source install/setup.bash
ros2 run transitions_and_states gc_reader --ros-args -p team_num:=0 -p player_num:=1
```

Substitua os números pelos do time e jogador. O leitor escuta todas as interfaces;
o PC deve estar na mesma sub-rede do GameController. O parâmetro `port` permite
usar outra porta em testes; normalmente mantenha 3838.

Em outro terminal com o ambiente ROS e o workspace carregados:

```bash
ros2 topic echo /gc/state modularized_bhv_msgs/msg/GameControllerMsg
ros2 interface show modularized_bhv_msgs/msg/GameControllerMsg
```

## Fluxo de informação

1. `gc_protocol.py` interpreta os bytes UDP.
2. `gc_reader.py` publica uma `GameControllerMsg` em `/gc/state`.
3. `StateMachineReceiver.game_controller_callback()` guarda `connected`, `state`,
   `penalty` e `player_found` em atributos, como `kick_done_callback()` guarda o Bool.
4. Ao chegar uma mensagem de percepção, `update_state()` passa esses valores como
   argumentos de `request_state_machine_update()`, junto de `kick_done`.
5. A máquina atualiza suas condições, aplica as regras de jogo existentes e publica
   o estado em `/transitions_and_states/state_machine`.

O tópico não usa String nem serialização JSON. O dicionário interno do parser serve
somente para decodificar o pacote; ele é convertido em campos ROS antes da publicação.
Uma mensagem única mantém estado, penalidade e conexão pertencentes ao mesmo pacote.

## Campos e decisões

A mensagem contém estado, fase, `stopped`, `set_play`, tempos, saída de bola,
placares, times e jogadores. A visão do jogador configurado aparece diretamente em
`penalty`, `penalty_name`, `score`, `opponent_score`, `has_kickoff`, `is_goalkeeper`,
`message_budget`, `secs_till_unpenalised` e `cautions`.

- Confira `connected` antes de usar os dados.
- `team_found` e `player_found` distinguem ausência de penalidade zero.
- `kicking_team = -1` indica ausência de time com a saída de bola.
- Conectado, mas sem jogador correspondente: a lógica usa INITIAL/idle, sem reaproveitar
  penalidade antiga.
- INITIAL, SET e FINISHED permitem idle e getting_up; READY permite movimentação sem
  chute; PLAYING usa as prioridades existentes; penalidade limita a idle/getting_up.
- A política existente de desconexão foi preservada: PLAYING e penalidade zero.
- `stopped`, fase e demais dados são publicados, mas não acrescentam regras de decisão.

As callbacks de GameController e de chute somente armazenam dados. A decisão é
recalculada na próxima mensagem de percepção; não foi acrescentado um timer de decisão.

Pacotes inválidos são descartados. Sem pacote válido por 3 segundos, o leitor publica
`connected=false` com os demais campos reinicializados a cada 0,5 segundo, mesmo com
relógio de simulação pausado. Novos pacotes válidos restabelecem os dados.
Se o leitor for encerrado, ele não poderá publicar desconexão: o receptor ainda não
possui um watchdog próprio para essa situação.

## Migração do tópico

`/gc/state` mudou de `std_msgs/msg/String` para `modularized_bhv_msgs/msg/GameControllerMsg`.
Recompile as interfaces e carregue `install/setup.bash` em todos os PCs que usam o tópico.
Reinicie os nós antigos. Não execute o leitor String do PDF nem o arquivo legado
`game_controller_node` (UInt8) simultaneamente neste tópico.
