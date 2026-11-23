# edrom_utils

Ferramentas auxiliares do robô.

## offset_calibrator.py — calibração dos offsets dos motores

Edita os `calibration_offset` de `edrom_lowlevel/config/motors_direct.yaml`.

Ao ligar o torque, cada motor vai (devagar) para a posição de **junta = 0**, ou
seja, exatamente para o seu `calibration_offset` atual — o mesmo que o
`direct_controller` faria. Ajuste cada motor e salve.

```bash
# 1. pare o que estiver usando os motores (botão 4 / Ctrl+C no direct_control)
# 2. ligue a alimentação dos motores e apoie/segure o robô
source ~/edrom_main/install/setup.bash
python3 ~/edrom_main/src/edrom_utils/offset_calibrator.py

python3 offset_calibrator.py --mock      # testar a interface sem motores
python3 offset_calibrator.py --speed 40  # movimento mais rápido (padrão 20)
```

Na janela:
- **▶ Ligar torque e ir para os offsets** — liga todos e leva cada um ao seu offset.
- **Torque** (por motor) — liga/solta um motor só.
- **− / + / −− / ++**, slider ou digitando no campo (Enter) — muda o offset e o motor
  se move na hora. O passo é escolhido no topo.
- **Usar posição atual** — com o torque daquele motor desligado, posicione a junta
  à mão no zero e clique: o offset vira a posição lida.
- **💾 Salvar no yaml** — grava só os offsets alterados (comentários preservados),
  faz backup `motors_direct.yaml.<data>.bak` e copia para `install/` para valer
  sem `colcon build`.

Offsets em radianos, no sentido do motor (somados depois do `inverted`).
Juntas em laranja = alteradas e ainda não salvas.
