# ipm_calibrator — calibração da cabeça no tapete ArUco

Calibra a cabeça do robô usando o tapete ArUco A0
(`Coordenadas_e_Montagem_A0_A1.pdf`).

O resultado traz:
- os offsets do `head_pan` e do `head_tilt`;
- a rotação entre o IMU da cabeça e a câmera;
- a posição da câmera em relação ao tapete.

## Como usar

1. Imprima o tapete em tamanho real (100%) e posicione o robô com as pontas dos
   pés na linha grossa.
2. Coloque o robô na first pose (botão 2, modo POSE) e rode o driver do IMU da
   cabeça. O robô deve ficar nessa pose, sem andar e sem ser tocado, durante
   toda a captura. Sem os botões, o modo POSE é:
   ```bash
   ros2 launch aurea_walk walking.launch.py teleop:=False
   ros2 launch edrom_lowlevel direct_control.launch.py
   ```
3. Grave. A cabeça percorre as poses sozinha:
   ```bash
   ros2 launch ipm_calibrator aruco_record.launch.py varredura:=True imu_topic:=/imu/data_raw
   ```
4. Calibre. O arquivo gravado fica em `ipm_calibrator/dados/`:
   ```bash
   ros2 run ipm_calibrator calibrar src/edrom_utils/ipm_calibrator/dados/aruco_XXXX.jsonl
   ```
5. Se o erro de reprojeção ficar abaixo de ~1 px, grave os offsets no
   `motors_direct.yaml` (o comando faz backup) e recompile. Só os offsets da
   cabeça são alterados; os do corpo são do movimento e ficam intactos:
   ```bash
   ros2 run ipm_calibrator calibrar src/edrom_utils/ipm_calibrator/dados/aruco_XXXX.jsonl --salvar
   colcon build --packages-select edrom_lowlevel
   ```

## Observações

- Se a câmera não olhar para o tapete durante a varredura, ajuste o parâmetro
  `tilt_poses`.
- O zero das juntas: pan = 0 é a câmera olhando para a frente do tapete, e
  tilt = 0 é a câmera na horizontal.
