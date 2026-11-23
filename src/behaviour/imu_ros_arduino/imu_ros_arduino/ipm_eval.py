#!/usr/bin/env python3
"""Compara os filtros de atitude usando o IPM como métrica.

Entrada (gravadas ao mesmo tempo, mesmo relógio do PC):
  --imu    CSV do imu_filters com record:=true (amostras BRUTAS)
  --aruco  JSONL do ipm_calibrator/aruco_record (cantos dos ArUcos no chão)

Para cada filtro, reroda a estimação sobre as MESMAS amostras (mesmo código
do nó online), calcula a pose da câmera no instante de cada imagem e projeta
os cantos dos marcadores no chão (z = 0 em base_footprint), como o soccer_ipm.

Métricas (nenhuma depende do yaw, que a IMU não observa):
  lado_rmse      erro do lado projetado vs --marker-size. Absoluto: inclui
                 erros fixos (altura da câmera, montagem) e os do filtro.
  dist_rms       variação da distância entre centros de dois marcadores ao
                 longo da gravação (deveria ser constante). Mede o erro
                 DINÂMICO do filtro (balanço, passos, impactos). Precisa de
                 >= 2 marcadores na mesma imagem. É a métrica de ranking.
  pos_std        desvio da posição de cada marcador em base_footprint.
                 Só faz sentido se os pés não se moveram (ex.: balançar o
                 tronco no lugar); com o robô andando, ignore.
  valido         fração dos marcadores cujos 4 cantos caíram no chão.

Opções:
  --search-offset  procura o atraso IMU->câmera que minimiza o erro
                   (latência da câmera); use o valor em stamp_offset_ms.
  --fit            ajusta pivot_height, camera_mount_rpy[0] e [1] por filtro
                   (mínimos quadrados no lado), para separar o erro fixo de
                   montagem do erro do filtro.

Roteiro sugerido: ArUcos de tamanho conhecido no chão à frente do robô;
gravar parado (~5 s, calibra o giro), depois balançar/andar no lugar.
"""
import argparse
import csv
import json
import math
import os
import sys

import cv2
import numpy as np
import yaml

from imu_ros_arduino.attitude import DEFAULTS, FILTER_NAMES, ImuPipeline
from imu_ros_arduino.camera_model import CAMERA_DEFAULTS, CameraModel, project_to_ground


def flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            out.update(flatten(v, key + '.'))
        else:
            out[key] = v
    return out


def load_config(path):
    params = {**DEFAULTS, **CAMERA_DEFAULTS}
    if path:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        # Formato de parâmetros ROS: {nó: {ros__parameters: {...}}}
        for v in data.values():
            if isinstance(v, dict) and 'ros__parameters' in v:
                data = v['ros__parameters']
                break
        params.update(flatten(data))
    return params


def load_imu(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f'{path}: sem amostras')
    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    return cols


def load_aruco(path):
    K = D = None
    frames = []
    with open(path) as f:
        for line in f:
            rec = json.loads(line)
            if rec['tipo'] == 'camera_info':
                K = np.array(rec['K'], float).reshape(3, 3)
                D = np.array(rec['D'], float)
            elif rec['tipo'] == 'frame':
                frames.append(rec)
    if K is None:
        sys.exit(f'{path}: sem camera_info (o aruco_record recebeu /camera/camera_info?)')
    # Observações achatadas: uma por marcador por imagem
    obs_frame, obs_id, corners = [], [], []
    for i, fr in enumerate(frames):
        for mid, c in zip(fr['ids'], fr['cantos']):
            obs_frame.append(i)
            obs_id.append(mid)
            corners.append(c)
    if not corners:
        sys.exit(f'{path}: nenhum ArUco detectado')
    pts = np.array(corners, float).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(pts, K, D if D.size else None).reshape(-1, 4, 2)
    stamps = np.array([fr['stamp_ns'] for fr in frames], float)
    return stamps, np.array(obs_frame), np.array(obs_id), norm


def replay(imu, params, filters):
    """Reroda os filtros. -> (stamps_ns, {filtro: (roll, pitch)})"""
    pipe = ImuPipeline(params, filters, log=lambda m: print(f'  {m}'))
    n = len(imu['stamp_ns'])
    stamps, ang = [], {f: ([], []) for f in filters}
    acc = np.column_stack([imu['ax'], imu['ay'], imu['az']])
    gyr = np.column_stack([imu['gx'], imu['gy'], imu['gz']])
    for i in range(n):
        if pipe.process(imu['t_esp_us'][i], acc[i], gyr[i]) is None:
            continue
        stamps.append(imu['stamp_ns'][i])
        for name, (r, p) in pipe.angles().items():
            ang[name][0].append(r)
            ang[name][1].append(p)
        if i % 20000 == 0 and i:
            print(f'  {i}/{n} amostras', flush=True)
    if not stamps:
        sys.exit('a calibração do giro não terminou (gravação curta demais?)')
    return np.array(stamps), {k: (np.array(r), np.array(p)) for k, (r, p) in ang.items()}


class Evaluator:
    def __init__(self, imu, imu_stamps, angles, aruco, marker_size):
        self.imu_stamps = imu_stamps
        self.angles = angles
        self.head_t = imu['stamp_ns']
        self.pan = imu['head_pan']
        self.tilt = imu['head_tilt']
        self.f_stamps, self.obs_frame, self.obs_id, self.norm = aruco
        self.size = marker_size

    def project(self, name, params, offset_ns, sel=None):
        """-> cantos no chão (N, 4, 2) para as observações sel (todas por padrão)."""
        cam = CameraModel(params)
        roll, pitch = self.angles[name]
        obs = np.arange(len(self.obs_frame)) if sel is None else sel
        frames = np.unique(self.obs_frame[obs])
        t = self.f_stamps[frames] - offset_ns   # instante da imagem no relógio da IMU
        inside = (t >= self.imu_stamps[0]) & (t <= self.imu_stamps[-1])
        r_f = np.interp(t, self.imu_stamps, roll)
        p_f = np.interp(t, self.imu_stamps, pitch)
        pan_f = np.interp(t, self.head_t, self.pan)
        tilt_f = np.interp(t, self.head_t, self.tilt)
        out = np.full((len(obs), 4, 2), np.nan)
        pos = {f: k for k, f in enumerate(frames)}
        groups = {}
        for j, o in enumerate(obs):
            groups.setdefault(self.obs_frame[o], []).append(j)
        for f, js in groups.items():
            k = pos[f]
            if not inside[k]:
                continue
            R, tr = cam.pose(r_f[k], p_f[k], pan_f[k], tilt_f[k])
            out[js] = project_to_ground(R, tr, self.norm[obs[js]].reshape(-1, 2)).reshape(-1, 4, 2)
        return out

    def edge_errors(self, ground):
        e = np.linalg.norm(np.roll(ground, -1, axis=1) - ground, axis=2) - self.size
        return e   # (N, 4)

    def metrics(self, ground, obs=None):
        obs = np.arange(len(self.obs_frame)) if obs is None else obs
        valid = np.all(np.isfinite(ground), axis=(1, 2))
        m = {'valido': float(valid.mean()) if len(valid) else 0.0}
        e = self.edge_errors(ground[valid])
        m['lado_rmse_mm'] = float(np.sqrt(np.mean(e ** 2)) * 1000) if e.size else math.nan
        m['lado_vies_mm'] = float(np.mean(e) * 1000) if e.size else math.nan
        centers = ground.mean(axis=1)
        frames, ids = self.obs_frame[obs], self.obs_id[obs]
        # distância entre pares de marcadores na mesma imagem
        pairs = {}
        by_frame = {}
        for j in np.flatnonzero(valid):
            by_frame.setdefault(frames[j], []).append(j)
        for js in by_frame.values():
            for a in range(len(js)):
                for b in range(a + 1, len(js)):
                    ja, jb = sorted((js[a], js[b]), key=lambda j: ids[j])
                    d = np.linalg.norm(centers[ja] - centers[jb])
                    pairs.setdefault((ids[ja], ids[jb]), []).append(d)
        dev = np.concatenate([np.array(v) - np.median(v) for v in pairs.values() if len(v) > 1]) \
            if pairs else np.array([])
        m['dist_rms_mm'] = float(np.sqrt(np.mean(dev ** 2)) * 1000) if dev.size else math.nan
        stds = []
        for mid in np.unique(ids[valid]):
            c = centers[valid & (ids == mid)]
            if len(c) > 1:
                stds.append(math.sqrt(c[:, 0].var() + c[:, 1].var()))
        m['pos_std_mm'] = float(np.mean(stds) * 1000) if stds else math.nan
        return m


def score(m):
    """Métrica de ranking: dinâmica (pares) se houver, senão o lado."""
    return m['dist_rms_mm'] if math.isfinite(m['dist_rms_mm']) else m['lado_rmse_mm']


def fit_extrinsics(ev, name, params, offset_ns, max_obs=400):
    try:
        from scipy.optimize import least_squares
    except ImportError:
        print('  scipy não disponível: --fit ignorado')
        return params, None
    n = len(ev.obs_frame)
    sel = np.linspace(0, n - 1, min(n, max_obs)).astype(int)
    base_rpy = list(params['camera.camera_mount_rpy'])

    def with_x(x):
        p = dict(params)
        p['camera.pivot_height'] = x[0]
        p['camera.camera_mount_rpy'] = [x[1], x[2], base_rpy[2]]
        return p

    def resid(x):
        e = ev.edge_errors(ev.project(name, with_x(x), offset_ns, sel))
        e = e.ravel()
        return np.where(np.isfinite(e), e, 0.05)   # canto fora do chão: penaliza

    x0 = [params['camera.pivot_height'], base_rpy[0], base_rpy[1]]
    res = least_squares(resid, x0, x_scale=[0.05, 0.02, 0.02], loss='soft_l1', f_scale=0.01)
    return with_x(res.x), res.x


def fmt(v, nd=1):
    return '   -  ' if not math.isfinite(v) else f'{v:6.{nd}f}'


def print_table(title, results):
    print(f'\n{title}')
    print(f'  {"filtro":<14}{"dist_rms":>10}{"lado_rmse":>11}{"lado_viés":>11}{"pos_std":>9}{"válido":>8}   (mm)')
    for name, m in sorted(results.items(), key=lambda kv: score(kv[1])):
        print(f'  {name:<14}{fmt(m["dist_rms_mm"]):>10}{fmt(m["lado_rmse_mm"]):>11}'
              f'{fmt(m["lado_vies_mm"]):>11}{fmt(m["pos_std_mm"]):>9}{m["valido"] * 100:7.0f}%')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--imu', required=True, help='CSV gravado pelo imu_filters (record:=true)')
    ap.add_argument('--aruco', required=True, help='JSONL do aruco_record')
    ap.add_argument('--marker-size', type=float, required=True, help='lado do ArUco em metros (só a parte preta)')
    ap.add_argument('--config', help='YAML de parâmetros do imu_filters (default: o do pacote instalado)')
    ap.add_argument('--filters', default=','.join(FILTER_NAMES))
    ap.add_argument('--offset-ms', type=float, default=0.0,
                    help='atraso da imagem em relação à IMU (a imagem representa stamp - offset)')
    ap.add_argument('--search-offset', type=float, nargs='?', const=150.0, metavar='MAX_MS',
                    help='procura o melhor offset em [-MAX_MS, MAX_MS] (padrão 150)')
    ap.add_argument('--fit', action='store_true', help='ajusta altura do pivô e montagem da câmera por filtro')
    ap.add_argument('--out', help='pasta dos resultados (padrão: ao lado do --aruco)')
    ap.add_argument('--plot', action='store_true', help='salva gráficos PNG')
    args = ap.parse_args()

    config = args.config
    if not config:
        try:
            from ament_index_python.packages import get_package_share_directory
            config = os.path.join(get_package_share_directory('imu_ros_arduino'), 'config', 'imu_filters.yaml')
        except Exception:
            config = None
    params = load_config(config)
    print(f'config: {config or "(padrões)"}')
    filters = [f.strip() for f in args.filters.split(',') if f.strip()]

    print('lendo IMU e reprocessando os filtros...')
    imu = load_imu(args.imu)
    imu_stamps, angles = replay(imu, params, filters)
    aruco = load_aruco(args.aruco)
    print(f'{len(aruco[0])} imagens com ArUco, {len(aruco[1])} marcadores, '
          f'ids {sorted(set(aruco[2].tolist()))}')
    ev = Evaluator(imu, imu_stamps, angles, aruco, args.marker_size)

    offset_ns = args.offset_ms * 1e6
    offset_curve = []
    if args.search_offset:
        main_f = [f for f in filters if f != 'acc'] or filters
        best = None
        for off in np.arange(-args.search_offset, args.search_offset + 1e-9, 5.0):
            s = float(np.median([score(ev.metrics(ev.project(f, params, off * 1e6))) for f in main_f]))
            offset_curve.append((off, s))
            if best is None or s < best[1]:
                best = (off, s)
        offset_ns = best[0] * 1e6
        print(f'\nmelhor offset IMU->câmera: {best[0]:.0f} ms (use stamp_offset_ms: {best[0]:.0f} '
              f'no imu_filters para o TF online casar com a imagem)')

    results, grounds = {}, {}
    for f in filters:
        grounds[f] = ev.project(f, params, offset_ns)
        results[f] = ev.metrics(grounds[f])
    print_table(f'Com a config ({offset_ns / 1e6:.0f} ms de offset):', results)

    fitted = {}
    if args.fit:
        print('\najustando extrínsecos por filtro...')
        fit_results = {}
        for f in filters:
            p_fit, x = fit_extrinsics(ev, f, params, offset_ns)
            if x is None:
                break
            fitted[f] = x
            fit_results[f] = ev.metrics(ev.project(f, p_fit, offset_ns))
            print(f'  {f:<14} pivot_height={x[0]:.4f} m  camera_mount_rpy=[{x[1]:.4f}, {x[2]:.4f}, '
                  f'{params["camera.camera_mount_rpy"][2]:.4f}]')
        if fit_results:
            print_table('Depois do ajuste (sobra só o erro do filtro):', fit_results)

    out = args.out or os.path.splitext(args.aruco)[0] + '_eval'
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'resumo.csv'), 'w', newline='') as fcsv:
        w = csv.writer(fcsv)
        w.writerow(['filtro', 'dist_rms_mm', 'lado_rmse_mm', 'lado_vies_mm', 'pos_std_mm', 'valido',
                    'offset_ms', 'fit_pivot_height', 'fit_mount_roll', 'fit_mount_pitch'])
        for f, m in results.items():
            x = fitted.get(f, [math.nan] * 3)
            w.writerow([f, m['dist_rms_mm'], m['lado_rmse_mm'], m['lado_vies_mm'], m['pos_std_mm'],
                        m['valido'], offset_ns / 1e6, *x])
    with open(os.path.join(out, 'por_marcador.csv'), 'w', newline='') as fcsv:
        w = csv.writer(fcsv)
        w.writerow(['stamp_ns', 'id', 'filtro', 'x_m', 'y_m', 'erro_lado_mm'])
        for f, g in grounds.items():
            e = np.nanmean(ev.edge_errors(g), axis=1) * 1000
            c = g.mean(axis=1)
            for j in range(len(g)):
                w.writerow([int(ev.f_stamps[ev.obs_frame[j]]), ev.obs_id[j], f,
                            c[j, 0], c[j, 1], e[j]])
    if offset_curve:
        with open(os.path.join(out, 'offset.csv'), 'w', newline='') as fcsv:
            csv.writer(fcsv).writerows([('offset_ms', 'score_mm'), *offset_curve])

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        t0 = imu_stamps[0]
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(12, 9))
        for f in filters:
            ax[0].plot((imu_stamps - t0) / 1e9, np.degrees(angles[f][0]), label=f, lw=0.8)
            ax[1].plot((imu_stamps - t0) / 1e9, np.degrees(angles[f][1]), label=f, lw=0.8)
            e = np.nanmean(np.abs(ev.edge_errors(grounds[f])), axis=1) * 1000
            ax[2].plot((ev.f_stamps[ev.obs_frame] - t0) / 1e9, e, '.', ms=2, label=f)
        ax[0].set_ylabel('roll (°)')
        ax[1].set_ylabel('pitch (°)')
        ax[2].set_ylabel('|erro do lado| (mm)')
        ax[2].set_xlabel('t (s)')
        ax[0].legend(ncol=len(filters))
        for a in ax:
            a.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out, 'filtros.png'), dpi=120)
    print(f'\nresultados em {out}')


if __name__ == '__main__':
    main()
