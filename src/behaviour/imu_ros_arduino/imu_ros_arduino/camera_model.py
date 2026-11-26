"""Pose da câmera em base_footprint a partir de roll/pitch da IMU DA CABEÇA.

Cadeia (REP-103, x frente, y esquerda, z cima):
  base_footprint   no chão, alinhado à gravidade, yaw = yaw do tronco
  -> pescoço:      ponto `neck` (cruzamento dos eixos de pan e tilt), tronco em pé
  -> cabeça:       Rz(pan) Ry(pitch) Rx(roll), roll/pitch da IMU (o tilt já está
                   no pitch; o yaw só vem do pan, a IMU não mede yaw)
  -> câmera:       camera_offset na cabeça, rotação fixa camera_mount_rpy
  -> óptico:       z para frente, x para a direita, y para baixo (camera_info)

Os eixos da IMU já estão mapeados para os da cabeça por 'axes'. O
desalinhamento fixo IMU/câmera entra em camera_mount_rpy.
"""
import math

import numpy as np

CAMERA_DEFAULTS = {
    'camera.neck': [0.0, 0.0, 0.50],            # m, cruzamento pan/tilt em base_footprint
    'camera.camera_offset': [0.03, 0.0, 0.05],  # m, pescoço -> câmera (na cabeça)
    'camera.camera_mount_rpy': [0.0, 0.0, 0.0],  # rad
    'camera.pan_sign': 1.0,
    'camera.pan_offset': 0.0,                  # rad
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
        p = {**CAMERA_DEFAULTS, **params}
        self.neck = np.asarray(p['camera.neck'], float)
        self.cam = np.asarray(p['camera.camera_offset'], float)
        self.R_mount = rpy_matrix(*[float(v) for v in p['camera.camera_mount_rpy']])
        self.pan_sign = float(p['camera.pan_sign'])
        self.pan_offset = float(p['camera.pan_offset'])

    def head_yaw(self, pan):
        """Yaw da cabeça em base_footprint; pan desconhecido (NaN) conta como 0."""
        return self.pan_sign * (pan if math.isfinite(pan) else 0.0) + self.pan_offset

    def pose(self, roll, pitch, pan):
        """-> (R, t): frame óptico em base_footprint (p_base = R p_opt + t)."""
        R_head = rpy_matrix(roll, pitch, self.head_yaw(pan))
        t = self.neck + R_head @ self.cam
        return R_head @ self.R_mount @ R_LINK_OPTICAL, t
