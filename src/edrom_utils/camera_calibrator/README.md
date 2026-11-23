# Calibração da câmera

## Como rodar

1. Em `src/edrom_vision/object_finder/config/usb_cam.yaml`, coloque o nome da câmera em `camera_name` (`verri` ou `milare`).
2. Feche o `vision.launch`, se estiver rodando (a câmera não pode estar em uso).
3. Rode:

```bash
ros2 run camera_calibrator camera_calibrator --size 8x6 --square 0.025
```

O script inicia o `usb_cam` e abre o calibrador. Na janela:

1. Mova o tabuleiro pela imagem (cantos, perto, longe e inclinado) até as barras **X**, **Y**, **Size** e **Skew** ficarem verdes.
2. Clique em **CALIBRATE** e espere (a janela congela durante o cálculo).
3. Clique em **SAVE**.
4. Aperte **q** na janela para sair e depois **Ctrl+C** no terminal. (O `cameracalibrator` do Humble não termina sozinho depois do **q**; o script trata o Ctrl+C e salva o arquivo.)

O resultado é salvo em `src/edrom_vision/object_finder/config/<camera_name>_camera-info.yaml`.

Depois de calibrar, rode `colcon build --packages-select object_finder` para instalar o arquivo novo. O `vision.launch` também lê o `usb_cam.yaml` (nome, dispositivo, resolução, formato), então usa a mesma câmera da calibração. Para trocar sem editar o arquivo:

```bash
ros2 launch object_finder vision.launch.py camera_name:=milare
```

## Parâmetros

| Parâmetro | O que é |
|---|---|
| `--size 8x6` | Número de **cantos internos** do tabuleiro (colunas x linhas). Um tabuleiro de 9×7 quadrados tem 8×6 cantos internos. |
| `--square 0.025` | Lado de um quadrado do tabuleiro, em **metros** (0.025 = 25 mm). Meça depois de imprimir. |
| `--rotation 180` | Gira a imagem mostrada no calibrador (a câmera está montada de cabeça para baixo). Padrão `180`; use `0` para a imagem crua. |

Com `--rotation 180`, o calibrador recebe a imagem girada (nó `image_rotate`, tópico `/camera/image_rotated`) e o resultado é convertido de volta para a imagem crua da câmera antes de salvar (`cx`, `cy` espelhados e `p1`, `p2` com sinal trocado). Assim o arquivo salvo vale para o `usb_cam` e o IPM, que usam a imagem crua.

As configurações da câmera (nome, dispositivo, resolução, formato) ficam em `src/edrom_vision/object_finder/config/usb_cam.yaml`.

## Observação

A calibração só vale para a resolução e o foco usados. Como o `vision.launch` e o calibrador usam o mesmo `usb_cam.yaml`, a resolução é sempre a mesma; se mudar a resolução, calibre de novo.

# Medir o desempenho do pipeline

Com o `vision.launch` rodando em um terminal, rode em outro:

```bash
ros2 run camera_calibrator desempenho               # relatório a cada 5 s
ros2 run camera_calibrator desempenho --janela 2    # a cada 2 s
```

**Ctrl+C** mostra o resumo da sessão inteira. Exemplo de relatório:

```
--- últimos 5.0 s ----------------------------------------
câmera   30.0 fps
YOLO     19.8 det/s  ( 66% dos frames)   latência    168 ms (p95  267, máx  308)
IPM       2.0 saídas/s     latência    188 ms (p95  252, máx  267)
CPU     usb_cam    6%    80 MB | finder  267%   827 MB | ipm    3%    90 MB
        máquina  84% (4 núcleos; 100% = 1 núcleo nos processos)
```

| Linha | O que é |
|---|---|
| `câmera` | Frames por segundo publicados pelo `usb_cam`. |
| `YOLO` | Detecções por segundo do `finder` e a fração dos frames da câmera que ele conseguiu processar (o resto é descartado). |
| `latência` | Tempo entre o frame ser publicado pela câmera e a detecção (ou a posição do IPM) ser publicada: média, percentil 95 e máximo. Não inclui o tempo de captura dentro da câmera. |
| `IPM` | Saídas por segundo do `soccer_ipm` (`balls_relative`). |
| `CPU` | Uso de CPU e memória de cada processo (100% = um núcleo inteiro) e da máquina. |

O medidor só assina tópicos leves (`camera_info` e as mensagens de detecção), então quase não interfere no que está medindo.
