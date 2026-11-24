"""Pose da câmera em base_footprint a partir de roll/pitch da IMU DA CABEÇA.

Cadeia (REP-103, x frente, y esquerda, z cima):
  base_footprint   no chão, alinhado à gravidade, yaw = yaw do tronco
  -> pescoço:        (0, 0, pivot_height) + neck_offset (tronco considerado em pé)
  -> orientação da cabeça: Rz(pan) Ry(pitch) Rx(roll), com roll/pitch da IMU
     (o tilt já está no pitch; o yaw só vem do pan, a IMU não mede yaw)
  -> câmera:         camera_offset na cabeça, rotação fixa camera_mount_rpy
  -> óptico:         z para frente, x para a direita, y para baixo (o que o
                     camera_info / IPM usam)

A IMU é considerada presa à cabeça (eixos já mapeados por 'axes'). O
desalinhamento fixo IMU/câmera entra em camera_mount_rpy. Com a câmera montada
de cabeça para baixo (image_rotation 180 na visão), camera_mount_rpy[0] = pi.
"""
import math

import numpy as np

CAMERA_DEFAULTS = {
    'camera.pivot_height': 0.30,               # m, altura do ponto em torno do qual o tronco inclina
    'camera.neck_offset': [0.0, 0.0, 0.20],    # m, pivô -> eixo do pan (no tronco)
    'camera.camera_offset': [0.03, 0.0, 0.05],  # m, eixo do tilt -> câmera (na cabeça)
    'camera.camera_mount_rpy': [math.pi, 0.0, 0.0],  # rad
    'camera.pan_sign': 1.0,
    'camera.pan_offset': 0.0,                  # rad
    'camera.fixed_pan': 0.0,                   # rad, usado sem joint_states
}

# Eixos do frame óptico escritos no frame da câmera (x frente, y esq., z cima)
R_LINK_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_matrix(r, p, y):
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def matrix_to_quaternion(R):
    """-> (x, y, z, w)"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


class CameraModel:
    def __init__(self, params):
        p = dict(CAMERA_DEFAULTS)
        p.update(params)
        self.p = p
        self.pivot = np.array([0.0, 0.0, float(p['camera.pivot_height'])])
        self.neck = np.asarray(p['camera.neck_offset'], float)
        self.cam = np.asarray(p['camera.camera_offset'], float)
        self.R_mount = rpy_matrix(*[float(v) for v in p['camera.camera_mount_rpy']])
        self.pan_sign = float(p['camera.pan_sign'])
        self.pan_offset = float(p['camera.pan_offset'])

    def pose(self, roll, pitch, pan=None, tilt=None):
        """-> (R, t): frame óptico em base_footprint (p_base = R p_opt + t).
        roll/pitch: IMU da cabeça. tilt é ignorado (já está no roll/pitch)."""
        if pan is None or not math.isfinite(pan):
            pan = float(self.p['camera.fixed_pan'])
        R_head = rot_z(self.pan_sign * pan + self.pan_offset) @ rot_y(pitch) @ rot_x(roll)
        t = self.pivot + self.neck + R_head @ self.cam   # tronco considerado em pé
        R = R_head @ self.R_mount @ R_LINK_OPTICAL
        return R, t


def project_to_ground(R, t, normalized):
    """Pontos normalizados (sem distorção) Nx2 -> pontos no chão (z=0) Nx2.
    Raios que não descem até o chão viram NaN."""
    n = np.asarray(normalized, float).reshape(-1, 2)
    rays = np.column_stack([n, np.ones(len(n))]) @ R.T
    with np.errstate(divide='ignore', invalid='ignore'):
        s = -t[2] / rays[:, 2]
    s[(rays[:, 2] >= -1e-9) | (s <= 0)] = np.nan
    return t[:2] + rays[:, :2] * s[:, None]
