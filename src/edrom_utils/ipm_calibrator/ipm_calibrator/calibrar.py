"""
Calibração da cabeça no tapete ArUco a partir de um .jsonl do aruco_record.

Calcula:
  1. offsets de head_pan e head_tilt (e, com --salvar, grava no motors_direct.yaml);
  2. rotação fixa IMU da cabeça -> câmera (gravidade medida pelo IMU x pelo tapete);
  3. geometria da cabeça: pivô (pan/tilt) e braço até a câmera, no referencial do tapete.

Modelo (referencial do tapete: origem entre os pés, +X frente, +Y esquerda, +Z cima):
    R_cabeça = Ry(tronco_pitch) Rx(tronco_roll) Rz(s_pan*pan + off_pan) Ry(s_tilt*tilt + off_tilt)
    R_câmera = R_cabeça Rx(roll_câmera) R_BO
    centro da câmera = pivô + R_cabeça * braço
Convenções: pan = 0 -> câmera olhando para +X do tapete; tilt = 0 -> câmera na
horizontal em relação ao tronco (o ângulo de montagem da câmera entra no offset).
Os sinais s_pan/s_tilt (sentido de cada junta) são detectados automaticamente.

Uso:
    ros2 run ipm_calibrator calibrar dados/aruco_XXXX.jsonl
    ros2 run ipm_calibrator calibrar dados/aruco_XXXX.jsonl --salvar
"""

import argparse
import json
from pathlib import Path
import re
import shutil
from datetime import datetime

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

# Eixos da câmera (x frente, y esquerda, z cima) -> referencial óptico (x direita, y baixo, z frente)
R_BO = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def rot(eixo, angulo):
    return Rotation.from_euler(eixo, angulo).as_matrix()


def centro_marcador(cantos, K, D):
    """Centro projetivo: interseção das diagonais, após tirar a distorção."""
    p = cv2.undistortPoints(np.array(cantos, float).reshape(-1, 1, 2), K, D, P=K).reshape(4, 2)
    h = np.hstack([p, np.ones((4, 1))])
    x = np.cross(np.cross(h[0], h[2]), np.cross(h[1], h[3]))
    return x[:2] / x[2]


def carregar(caminho_jsonl, tapete, max_px=2.0):
    """Agrupa os frames por pose da cabeça e faz a média dos centros de cada ArUco."""
    K = D = None
    grupos = {}
    for linha in open(caminho_jsonl):
        r = json.loads(linha)
        if r['tipo'] == 'camera_info':
            K, D = np.array(r['K']).reshape(3, 3), np.array(r['D'])
        elif r['juntas'] is not None:
            g = grupos.setdefault(tuple(np.round(r['juntas'], 4)), {'centros': {}, 'acc': []})
            for i, c in zip(r['ids'], r['cantos']):
                if i in tapete:
                    g['centros'].setdefault(i, []).append(centro_marcador(c, K, D))
            if r['acc'] is not None:
                g['acc'].append(r['acc'])

    poses = []
    for (pan, tilt), g in grupos.items():
        ids = sorted(g['centros'])
        if len(ids) < 4:
            continue
        mundo = np.array([tapete[i] for i in ids])
        img = np.array([np.mean(g['centros'][i], axis=0) for i in ids])
        _, rvec, tvec = cv2.solvePnP(mundo, img, K, None, flags=cv2.SOLVEPNP_IPPE)
        proj = cv2.projectPoints(mundo, rvec, tvec, K, None)[0].reshape(-1, 2)
        erro = np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=1)))
        if erro > max_px:
            print(f'  pose pan={pan:+.3f} tilt={tilt:+.3f} descartada: PnP com {erro:.1f} px')
            continue
        R_CM = cv2.Rodrigues(rvec)[0]  # tapete -> câmera
        poses.append({
            'pan': pan, 'tilt': tilt, 'mundo': mundo, 'img': img,
            'R_CM': R_CM, 'centro': (-R_CM.T @ tvec).ravel(),
            'acc': np.mean(g['acc'], axis=0) if g['acc'] else None,
        })
    return K, poses


def modelo(x, pan, tilt, s_pan, s_tilt):
    off_pan, off_tilt, tronco_pitch, tronco_roll, roll_cam = x[:5]
    pivo, braco = x[5:8], x[8:11]
    R_cab = (rot('y', tronco_pitch) @ rot('x', tronco_roll)
             @ rot('z', s_pan * pan + off_pan) @ rot('y', s_tilt * tilt + off_tilt))
    return R_cab @ rot('x', roll_cam) @ R_BO, pivo + R_cab @ braco


def residuos(x, poses, K, s_pan, s_tilt):
    r = []
    for p in poses:
        R_MO, c = modelo(x, p['pan'], p['tilt'], s_pan, s_tilt)
        cam = (p['mundo'] - c) @ R_MO          # pontos no referencial óptico
        uv = cam @ K.T
        r.append(uv[:, :2] / uv[:, 2:] - p['img'])
    return np.concatenate(r).ravel()


def ajustar_cabeca(poses, K, roll_cam0):
    """Ajusta o modelo para os 4 sentidos possíveis das juntas e fica com o melhor."""
    x0 = np.zeros(11)
    x0[4] = roll_cam0
    x0[5:8] = np.mean([p['centro'] for p in poses], axis=0)
    melhor = None
    for s_pan in (1, -1):
        for s_tilt in (1, -1):
            res = least_squares(residuos, x0, args=(poses, K, s_pan, s_tilt))
            # Inverter os dois sinais equivale a um tronco de cabeça para baixo: descarta
            if np.any(np.abs(res.x[2:4]) > 0.5):
                continue
            if melhor is None or res.cost < melhor[0].cost:
                melhor = (res, s_pan, s_tilt)
    res, s_pan, s_tilt = melhor
    n = res.fun.size
    sigma2 = 2 * res.cost / max(n - res.x.size, 1)
    desvio = np.sqrt(np.diag(np.linalg.pinv(res.jac.T @ res.jac)) * sigma2)
    return res.x, desvio, s_pan, s_tilt, np.sqrt(2 * res.cost / (n / 2))


def ajustar_imu(poses):
    """Rotação R_CI (IMU -> câmera) que leva a gravidade do IMU na 'cima' vista no tapete."""
    pares = [(p['acc'] / np.linalg.norm(p['acc']), p['R_CM'][:, 2])
             for p in poses if p['acc'] is not None]
    if len(pares) < 3:
        return None, None
    u_imu, u_cam = map(np.array, zip(*pares))
    U, S, Vt = np.linalg.svd(u_imu.T @ u_cam)
    R_CI = Vt.T @ np.diag([1, 1, np.linalg.det(Vt.T @ U.T)]) @ U.T
    erros = np.degrees(np.arccos(np.clip(np.sum((u_imu @ R_CI.T) * u_cam, axis=1), -1, 1)))
    return R_CI, erros


def salvar_offsets(motors_yaml, off_pan, off_tilt, s_pan, s_tilt):
    """
    Corrige o calibration_offset atual da cabeça e grava (com backup).

    Só head_pan e head_tilt são alterados: os offsets do corpo são do movimento
    (walking) e não podem ser mexidos por esta calibração.
    """
    texto = motors_yaml.read_text()
    params = yaml.safe_load(texto)['direct_controller']['ros__parameters']

    for nome, off, s in (('head_pan', off_pan, s_pan), ('head_tilt', off_tilt, s_tilt)):
        # junta real = comando + s*off; motor = ±junta + calibration_offset
        atual = float(params[nome].get('calibration_offset', 0.0))
        sinal_motor = -1 if params[nome].get('inverted', False) else 1
        novo = atual - sinal_motor * s * off
        print(f'  {nome}: calibration_offset {atual:+.4f} -> {novo:+.4f}')

        # Bloco "  head_pan:" seguido das linhas mais indentadas (id, protocol, ...)
        m = re.search(rf'^([ \t]*){nome}:[ \t]*\n((?:\1[ \t]+\S.*\n)*)', texto, re.M)
        if not m:
            raise RuntimeError(f'{nome} não está em forma de bloco no {motors_yaml}')
        bloco = m.group(2)
        recuo = re.match(r'[ \t]*', bloco).group(0) if bloco else m.group(1) + '  '
        linha = f'{recuo}calibration_offset: {novo:.4f}\n'
        if re.search(r'^\s*calibration_offset:', bloco, re.M):
            bloco = re.sub(r'^\s*calibration_offset:.*\n', linha, bloco, flags=re.M)
        else:
            bloco += linha
        texto = texto[:m.start(2)] + bloco + texto[m.end(2):]

    shutil.copy2(motors_yaml, f'{motors_yaml}.{datetime.now():%Y%m%d_%H%M%S}.bak')
    motors_yaml.write_text(texto)
    print(f'Gravado em {motors_yaml}. Rode colcon build --packages-select edrom_lowlevel.')


def main():
    ws = Path(get_package_prefix('ipm_calibrator')).parents[1]
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('jsonl', type=Path)
    ap.add_argument('--tapete', type=Path, default=Path(
        get_package_share_directory('ipm_calibrator')) / 'config' / 'tapete_a0.yaml')
    ap.add_argument('--rotacao', type=int, default=180,
                    help='rotação da câmera montada (image_rotation do vision.launch)')
    ap.add_argument('--salvar', action='store_true', help='grava os offsets no motors_direct.yaml')
    ap.add_argument('--motors-yaml', type=Path,
                    default=ws / 'src' / 'edrom_lowlevel' / 'config' / 'motors_direct.yaml')
    args = ap.parse_args()

    marcadores = yaml.safe_load(open(args.tapete))['marcadores']
    tapete = {int(i): v[:3] for i, v in marcadores.items()}

    K, poses = carregar(args.jsonl, tapete)
    print(f'{len(poses)} poses da cabeça usadas')
    if len(poses) < 6:
        raise SystemExit('Poses insuficientes (mínimo 6, varrendo pan e tilt).')

    x, desvio, s_pan, s_tilt, rms = ajustar_cabeca(poses, K, np.radians(args.rotacao))
    graus = np.degrees
    print('\nCabeça (convenção: junta real = comando + sinal * offset)')
    print(f'  offset pan : {graus(x[0]):+.2f}° ± {graus(desvio[0]):.2f}°  (sinal {s_pan:+d})')
    print(f'  offset tilt: {graus(x[1]):+.2f}° ± {graus(desvio[1]):.2f}°  (sinal {s_tilt:+d})')
    print(f'  tronco     : pitch {graus(x[2]):+.2f}°, roll {graus(x[3]):+.2f}°')
    print(f'  pivô       : {np.round(x[5:8], 3)} m   braço até a câmera: {np.round(x[8:11], 3)} m')
    print(f'  erro de reprojeção: {rms:.2f} px (RMS)')

    R_CI, erros = ajustar_imu(poses)
    if R_CI is None:
        print('\nIMU: menos de 3 poses com acelerômetro, rotação IMU -> câmera não calculada')
    else:
        print('\nIMU -> câmera')
        print(f'  rpy: {np.round(graus(Rotation.from_matrix(R_CI).as_euler("xyz")), 2)}°')
        print(f'  erro da gravidade: média {erros.mean():.2f}°, máx {erros.max():.2f}°')

    saida = args.jsonl.with_name(args.jsonl.stem + '_calibracao.yaml')
    yaml.safe_dump({
        'offset_pan_rad': float(x[0]), 'offset_tilt_rad': float(x[1]),
        'sinal_pan': s_pan, 'sinal_tilt': s_tilt,
        'pivo_m': x[5:8].round(4).tolist(), 'braco_camera_m': x[8:11].round(4).tolist(),
        'tronco_pitch_roll_rad': x[2:4].round(5).tolist(),
        'roll_camera_rad': float(x[4]),
        'R_imu_para_camera': None if R_CI is None else R_CI.round(6).tolist(),
        'erro_reprojecao_px': float(rms),
    }, open(saida, 'w'), sort_keys=False)
    print(f'\nResultado salvo em {saida}')

    if args.salvar:
        salvar_offsets(args.motors_yaml, x[0], x[1], s_pan, s_tilt)


if __name__ == '__main__':
    main()
