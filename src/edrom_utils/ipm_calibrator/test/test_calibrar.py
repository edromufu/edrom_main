"""Gera um .jsonl sintético com parâmetros conhecidos e confere se o calibrar os recupera."""

import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml

from ipm_calibrator import calibrar

PACOTE = Path(__file__).resolve().parents[1]
K = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]])
D = np.array([-0.2, 0.05, 0, 0, 0])
# off_pan, off_tilt, tronco_pitch, tronco_roll, roll_cam, pivô (3), braço (3)
X_REAL = np.array([0.05, -0.08, 0.03, -0.02, np.pi + 0.01, -0.02, 0.0, 0.40, 0.04, 0.0, 0.03])
S_PAN, S_TILT = 1, -1
R_CI_REAL = calibrar.rot('xyz', [0.05, -0.03, 0.1])


def gerar_jsonl(caminho, rng):
    marcadores = yaml.safe_load(open(PACOTE / 'config' / 'tapete_a0.yaml'))['marcadores']
    with open(caminho, 'w') as f:
        f.write(json.dumps({'tipo': 'camera_info', 'K': K.ravel().tolist(), 'D': D.tolist()}) + '\n')
        for pan in (-0.6, -0.3, 0.0, 0.3, 0.6):
            for tilt in (-1.0, -0.75, -0.5, -0.3):
                R_MO, c = calibrar.modelo(X_REAL, pan, tilt, S_PAN, S_TILT)
                R_CM = R_MO.T
                rvec, tvec = cv2.Rodrigues(R_CM)[0], -R_CM @ c
                ids, cantos = [], []
                for i, (x, y, z, lado) in marcadores.items():
                    m = lado / 2
                    pts = np.array([[x - m, y + m, z], [x - m, y - m, z],
                                    [x + m, y - m, z], [x + m, y + m, z]])
                    if np.any((pts - c) @ R_MO[:, 2] < 0.05):
                        continue
                    uv = cv2.projectPoints(pts, rvec, tvec, K, D)[0].reshape(4, 2)
                    if np.all((uv >= 0) & (uv < [640, 480])):
                        ids.append(int(i))
                        cantos.append(uv)
                acc = 9.81 * R_CI_REAL.T @ R_CM[:, 2]
                for _ in range(5):
                    f.write(json.dumps({
                        'tipo': 'frame', 'stamp_ns': 0, 'ids': ids,
                        'cantos': [(c4 + rng.normal(0, 0.3, (4, 2))).tolist() for c4 in cantos],
                        'acc': (acc + rng.normal(0, 0.05, 3)).tolist(),
                        'juntas': [pan, tilt],
                    }) + '\n')


def test_recupera_parametros(tmp_path):
    caminho = tmp_path / 'sintetico.jsonl'
    gerar_jsonl(caminho, np.random.default_rng(0))
    marcadores = yaml.safe_load(open(PACOTE / 'config' / 'tapete_a0.yaml'))['marcadores']
    tapete = {int(i): v[:3] for i, v in marcadores.items()}

    K_lido, poses = calibrar.carregar(caminho, tapete)
    assert len(poses) == 20

    x, desvio, s_pan, s_tilt, rms = calibrar.ajustar_cabeca(poses, K_lido, np.pi)
    assert (s_pan, s_tilt) == (S_PAN, S_TILT)
    assert np.allclose(x[:4], X_REAL[:4], atol=np.radians(0.2))
    assert np.allclose(x[5:], X_REAL[5:], atol=0.005)
    assert rms < 0.5

    R_CI, erros = calibrar.ajustar_imu(poses)
    angulo = np.linalg.norm(cv2.Rodrigues(R_CI @ R_CI_REAL.T)[0])
    assert np.degrees(angulo) < 0.5
    assert erros.max() < 1.0


def test_salvar_offsets(tmp_path):
    src = PACOTE.parents[1]
    original = src / 'edrom_lowlevel' / 'config' / 'motors_direct.yaml'
    yaml_motores = tmp_path / 'motors_direct.yaml'
    shutil.copy(original, yaml_motores)

    # junta real = comando + s*off; head_pan e head_tilt não invertidos, offset atual 0
    calibrar.salvar_offsets(yaml_motores, 0.05, -0.08, 1, -1)
    # Segunda vez: corrige o offset que já existe (em vez de acrescentar outro)
    calibrar.salvar_offsets(yaml_motores, 0.01, 0.0, 1, -1)

    antes = yaml.safe_load(open(original))['direct_controller']['ros__parameters']
    depois = yaml.safe_load(open(yaml_motores))['direct_controller']['ros__parameters']
    assert depois['head_pan']['calibration_offset'] == -0.06
    assert depois['head_tilt']['calibration_offset'] == -0.08
    # Corpo intacto: só a cabeça muda
    for nome in antes['joint_names']:
        if nome not in ('head_pan', 'head_tilt'):
            assert depois[nome] == antes[nome]
    assert len(list(tmp_path.glob('*.bak'))) >= 1

