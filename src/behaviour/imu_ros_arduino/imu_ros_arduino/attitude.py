"""Roll/pitch de um MPU-6050 com EKF, a partir das amostras brutas.

Convenção (REP-103): corpo com x para frente, y para a esquerda, z para cima.
roll/pitch de Euler ZYX; parado e nivelado o acelerômetro mede +1 g em z e a
gravidade vista no corpo é [-sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch)].
Yaw não é observável sem magnetômetro e não é estimado.
"""
from collections import deque
import math

import numpy as np

# Escalas configuradas no firmware (firmware-milare/src/main.cpp)
ACCEL_LSB_PER_G = 16384.0      # ±2 g
GYRO_LSB_PER_DPS = 65.5        # ±500 °/s
GRAVITY = 9.80665
DEG = math.pi / 180.0

CALIB_SAMPLES = 400            # 2 s parado a 200 Hz
CALIB_MAX_STD = 0.5 * DEG      # rad/s, desvio máximo do giro para considerar "parado"
CALIB_MAX_ACCEL_DEV = 0.05     # g, | |a| - 1 g | máximo durante a calibração
CALIB_TIMEOUT_S = 10.0         # depois disso usa a janela mais parada vista
MAX_DT = 0.05                  # s, limita dt após perdas de amostra


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def accel_tilt(ax, ay, az):
    return math.atan2(ay, az), math.atan2(-ax, math.sqrt(ay * ay + az * az))


def axes_matrix(axes):
    """['-y', 'x', 'z'] -> matriz 3x3 M com corpo = M @ sensor."""
    m = np.zeros((3, 3))
    used = set()
    for row, spec in enumerate(axes):
        spec = spec.strip().lower()
        sign = -1.0 if spec.startswith('-') else 1.0
        axis = spec.lstrip('+-')
        if axis not in ('x', 'y', 'z') or axis in used:
            raise ValueError(f'axes inválido: {axes} (use uma permutação de x, y, z com sinal)')
        used.add(axis)
        m[row, 'xyz'.index(axis)] = sign
    return m


class Ekf:
    """EKF de estado [roll, pitch, bias_gx, bias_gy].

    Predição pela cinemática de Euler com o giro; correção pela direção da
    gravidade no acelerômetro. Em impactos (|a| longe de 1 g) a confiança no
    acelerômetro cai e, acima de ACCEL_REJECT, a correção é descartada.
    """
    GYRO_NOISE = 0.002         # rad/s/sqrt(Hz)
    BIAS_NOISE = 0.0001        # rad/s/sqrt(s)
    ACCEL_NOISE = 0.05         # ~rad
    ACCEL_DEV_SCALE = 0.05     # g, onde R começa a crescer
    ACCEL_REJECT = 0.3         # g, acima disso ignora o acelerômetro
    COS_PITCH_MIN = 0.05

    def __init__(self):
        self.x = np.zeros(4)
        self.P = np.zeros((4, 4))

    def begin(self, a):
        roll, pitch = accel_tilt(*a)
        self.x = np.array([roll, pitch, 0.0, 0.0])
        self.P = np.diag([0.1 ** 2, 0.1 ** 2, 0.01 ** 2, 0.01 ** 2])

    def _normalize(self):
        x = self.x
        if x[1] > math.pi / 2:
            x[1] = math.pi - x[1]
            x[0] += math.pi
        elif x[1] < -math.pi / 2:
            x[1] = -math.pi - x[1]
            x[0] += math.pi
        x[0] = wrap(x[0])

    def _trust(self, norm_g):
        """Confiança no acelerômetro (0 = descartado)."""
        dev = abs(norm_g - 1.0)
        if norm_g < 1e-6 or dev > self.ACCEL_REJECT:
            return 0.0
        k = dev / self.ACCEL_DEV_SCALE
        return 1.0 / (1.0 + k * k)

    def update(self, g, a, dt):
        # Predição
        phi, th = self.x[0], self.x[1]
        pc, qc, r = g[0] - self.x[2], g[1] - self.x[3], g[2]
        sphi, cphi = math.sin(phi), math.cos(phi)
        cth = math.cos(th)
        if abs(cth) < self.COS_PITCH_MIN:
            cth = math.copysign(self.COS_PITCH_MIN, cth)
        tth = math.sin(th) / cth
        self.x[0] += (pc + sphi * tth * qc + cphi * tth * r) * dt
        self.x[1] += (cphi * qc - sphi * r) * dt
        self._normalize()
        F = np.eye(4)
        F[0, :] += np.array([(cphi * qc - sphi * r) * tth,
                             (sphi * qc + cphi * r) / (cth * cth),
                             -1.0, -sphi * tth]) * dt
        F[1, :] += np.array([-sphi * qc - cphi * r, 0.0, 0.0, -cphi]) * dt
        qa = self.GYRO_NOISE ** 2 * dt
        qb = self.BIAS_NOISE ** 2 * dt
        self.P = F @ self.P @ F.T + np.diag([qa, qa, qb, qb])

        # Correção com o acelerômetro
        norm = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        trust = self._trust(norm)
        if trust <= 0.0:
            return
        z = np.asarray(a) / norm
        phi, th = self.x[0], self.x[1]
        sphi, cphi, sth, cth = math.sin(phi), math.cos(phi), math.sin(th), math.cos(th)
        h = np.array([-sth, sphi * cth, cphi * cth])
        H = np.array([[0.0, -cth, 0.0, 0.0],
                      [cphi * cth, -sphi * sth, 0.0, 0.0],
                      [-sphi * cth, -cphi * sth, 0.0, 0.0]])
        r_var = self.ACCEL_NOISE ** 2 / trust   # = noise² (1 + k²)
        PHt = self.P @ H.T
        S = H @ PHt + r_var * np.eye(3)
        try:
            K = np.linalg.solve(S, PHt.T).T
        except np.linalg.LinAlgError:
            return
        self.x += K @ (z - h)
        self._normalize()
        self.P = self.P - K @ PHt.T
        self.P = 0.5 * (self.P + self.P.T)

    @property
    def roll(self):
        return float(self.x[0])

    @property
    def pitch(self):
        return float(self.x[1])

    def std(self):
        return math.sqrt(max(self.P[0, 0], 0.0)), math.sqrt(max(self.P[1, 1], 0.0))


class EspClock:
    """Converte o micros() do ESP32 (uint32) em tempo contínuo e em stamp do PC.

    O stamp do PC de cada amostra vem do instante de chegada (pc_ns), que só
    tem atraso a mais (serial/USB/escalonamento). O deslocamento PC - ESP é
    estimado pelo MÍNIMO de (pc_ns - t_esp) numa janela, que acompanha a
    deriva do cristal do ESP32 e ignora os atrasos.
    """

    WRAP = 1 << 32

    def __init__(self, window=400):
        self.window = window
        self.reset()

    def reset(self):
        self.last_raw = None
        self.wraps = 0
        self.mins = deque()   # (indice, offset) com offsets crescentes
        self.n = 0

    def update(self, t_us32, pc_ns):
        """Retorna (t_esp_us contínuo, stamp_ns, reiniciou)."""
        restarted = False
        if self.last_raw is not None and t_us32 < self.last_raw:
            if self.last_raw - t_us32 > self.WRAP // 2:
                self.wraps += 1                     # micros() deu a volta
            else:
                self.reset()                        # ESP32 reiniciou
                restarted = True
        self.last_raw = t_us32
        t_us = t_us32 + self.wraps * self.WRAP
        off = pc_ns - t_us * 1000
        while self.mins and self.mins[-1][1] >= off:
            self.mins.pop()
        self.mins.append((self.n, off))
        while self.mins[0][0] <= self.n - self.window:
            self.mins.popleft()
        self.n += 1
        return t_us, t_us * 1000 + self.mins[0][1], restarted


def parse_udp_line(line):
    """b'IMU|IMU2 seq t_us ax ay az gx gy gz pc_ns' -> (seq, t_us, ax, ..., gz, pc_ns), ou None.

    Cada IMU chega na sua porta UDP, então a tag só é conferida, não separa nada.
    """
    parts = line.split()
    if len(parts) != 10 or parts[0] not in (b'IMU', b'IMU2'):
        return None
    try:
        return tuple(int(v) for v in parts[1:])
    except ValueError:
        return None


class ImuPipeline:
    """Amostras brutas -> unidades físicas no corpo -> calibração do giro -> EKF."""

    def __init__(self, axes, log=print):
        self.M = axes_matrix(axes)
        self.log = log
        self.ekf = Ekf()
        self.calib_buf = deque(maxlen=CALIB_SAMPLES)
        self.calib_best = None      # (std máximo, bias, a médio) da janela mais parada vista
        self.calib_start_us = None
        self.gyro_bias = None       # rad/s, no corpo
        self.last_t_us = None

    @property
    def calibrated(self):
        return self.gyro_bias is not None

    def _calibrate(self, t_us, a, g):
        if self.calib_start_us is None:
            self.calib_start_us = t_us
        self.calib_buf.append((a, g))
        if len(self.calib_buf) < CALIB_SAMPLES:
            return False
        acc = np.array([s[0] for s in self.calib_buf])
        gyr = np.array([s[1] for s in self.calib_buf])
        gstd = float(np.max(np.std(gyr, axis=0)))
        adev = float(np.max(np.abs(np.linalg.norm(acc, axis=1) - 1.0)))
        if self.calib_best is None or gstd < self.calib_best[0]:
            self.calib_best = (gstd, gyr.mean(axis=0), acc.mean(axis=0))
        ok = gstd <= CALIB_MAX_STD and adev <= CALIB_MAX_ACCEL_DEV
        if not ok and (t_us - self.calib_start_us) * 1e-6 <= CALIB_TIMEOUT_S:
            for _ in range(CALIB_SAMPLES // 4):      # desliza a janela
                self.calib_buf.popleft()
            return False
        gstd, bias, a_mean = (gstd, gyr.mean(axis=0), acc.mean(axis=0)) if ok else self.calib_best
        self.gyro_bias = bias
        self.ekf.begin(a_mean)
        self.log(('calibração do giro ok: ' if ok else
                  'AVISO: IMU não ficou parada; calibração com a janela mais parada: ')
                 + f'bias (°/s) {np.round(bias / DEG, 3).tolist()}, desvio {gstd / DEG:.3f} °/s')
        return True

    def process(self, t_us, acc_counts, gyro_counts):
        """Retorna (a em g, giro em rad/s sem bias) depois da calibração, ou None enquanto calibra."""
        a = self.M @ (np.asarray(acc_counts, float) / ACCEL_LSB_PER_G)
        g = self.M @ (np.asarray(gyro_counts, float) / GYRO_LSB_PER_DPS * DEG)
        if not self.calibrated:
            if self._calibrate(t_us, a, g):
                self.last_t_us = t_us
            return None
        dt = 0.0 if self.last_t_us is None else (t_us - self.last_t_us) * 1e-6
        self.last_t_us = t_us
        g = g - self.gyro_bias
        self.ekf.update(g, a, min(max(dt, 0.0), MAX_DT))
        return a, g
