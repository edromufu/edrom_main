
import argparse
import os
import signal
import subprocess
import tarfile
import time
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix

TARBALL = '/tmp/calibrationdata.tar.gz'


def desfazer_rotacao_180(calib):
    """
    Converte uma calibração feita na imagem girada 180 graus para a imagem
    crua da câmera (que é o que o usb_cam publica e o IPM usa).

    Girar 180 graus: u' = (w-1) - u, v' = (h-1) - v, e nas coordenadas
    normalizadas x' = -x, y' = -y. Então fx, fy e a distorção radial
    (k1, k2, k3) não mudam; cx, cy são espelhados; e a distorção tangencial
    (p1, p2) troca de sinal.
    """
    w, h = calib['image_width'], calib['image_height']

    k = calib['camera_matrix']['data']  # [fx 0 cx; 0 fy cy; 0 0 1]
    k[2], k[5] = (w - 1) - k[2], (h - 1) - k[5]

    p = calib['projection_matrix']['data']  # [fx' 0 cx' tx; 0 fy' cy' ty; 0 0 1 0]
    p[2], p[6] = (w - 1) - p[2], (h - 1) - p[6]

    d = calib['distortion_coefficients']['data']  # plumb_bob: [k1 k2 p1 p2 k3]
    if calib.get('distortion_model', 'plumb_bob') != 'plumb_bob':
        raise SystemExit(f"Modelo de distorção {calib['distortion_model']} não suportado")
    d[2], d[3] = -d[2], -d[3]

    # Calibração monocular: rectification_matrix é a identidade, não muda.
    return calib


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--size', default='8x6')
    parser.add_argument('--square', default='0.025')
    parser.add_argument(
        '--rotation', type=int, default=180, choices=[0, 180],
        help='Gira a imagem mostrada no calibrador (câmera montada de cabeça para '
             'baixo). O arquivo salvo é sempre para a imagem crua da câmera.')
    args = parser.parse_args()

    # <ws>/install/camera_calibrator -> <ws>
    ws = Path(get_package_prefix('camera_calibrator')).parents[1]
    # Mesma pasta que o object_finder instala e o vision.launch lê.
    # O src pode estar em <ws>/src ou um nível abaixo (ex.: <ws>/visao-cbr26/src,
    # quando o colcon build é rodado na raiz do repositório).
    config = Path('edrom_vision/object_finder/config')
    candidates = [ws / 'src' / config, *sorted(ws.glob(f'*/src/{config}'))]
    files = next((c for c in candidates if (c / 'usb_cam.yaml').is_file()), None)
    if files is None:
        raise SystemExit(
            'usb_cam.yaml não encontrado. Procurei em:\n  '
            + '\n  '.join(str(c) for c in candidates))
    params_file = files / 'usb_cam.yaml'

    # camera_name (verri ou milare) vem do usb_cam.yaml
    params = yaml.safe_load(params_file.read_text())
    camera_name = next(iter(params.values()))['ros__parameters']['camera_name']

    if os.path.exists(TARBALL):
        os.remove(TARBALL)

    camera = subprocess.Popen([
        'ros2', 'run', 'usb_cam', 'usb_cam_node_exe',
        '--ros-args', '-r', '__ns:=/camera', '--params-file', str(params_file),
    ])
    time.sleep(2)
    if camera.poll() is not None:
        raise SystemExit('usb_cam não iniciou. A câmera já está em uso (vision.launch rodando?)')

    processos = [camera]
    imagem = '/camera/image_raw'
    if args.rotation == 180:
        # Republica a imagem girada para o calibrador mostrar em pé
        processos.append(subprocess.Popen([
            'ros2', 'run', 'camera_calibrator', 'image_rotate',
            '--ros-args', '-r', '__ns:=/camera',
        ]))
        imagem = '/camera/image_rotated'

    print('\n>>> CALIBRATE, SAVE, aperte q na janela e depois Ctrl+C aqui para salvar. <<<\n')
    try:
        # O cameracalibrator do Humble não termina sozinho depois do q (uma
        # thread interna fica presa); o Ctrl+C é tratado abaixo e o resultado é salvo.
        subprocess.run([
            'ros2', 'run', 'camera_calibration', 'cameracalibrator',
            '--size', args.size,
            '--square', args.square,
            '--camera_name', camera_name,
            '--no-service-check',
            '--ros-args', '-r', f'image:={imagem}',
        ])
    except KeyboardInterrupt:
        # Ctrl+C no terminal fecha o calibrador; segue para salvar o que foi
        # gravado no SAVE (o tarball é apagado no início, então é desta rodada)
        pass
    finally:
        for p in reversed(processos):
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=5)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                p.kill()

    if not os.path.exists(TARBALL):
        raise SystemExit('Nada salvo: clique em SAVE antes de sair.')

    output = files / f'{camera_name}_camera-info.yaml'
    with tarfile.open(TARBALL) as tar:
        ost = tar.extractfile('ost.yaml').read()
    if args.rotation == 180:
        calib = desfazer_rotacao_180(yaml.safe_load(ost))
        output.write_text(yaml.safe_dump(calib, sort_keys=False))
    else:
        output.write_bytes(ost)
    print(f'Calibração salva em {output}')


if __name__ == '__main__':
    main()