"""Estimação de atitude (roll/pitch) a partir das amostras brutas do MPU-6050.

Usado pelo nó imu_filters_node.

Convenção (REP-103): corpo com x para frente, y para a esquerda, z para cima.
roll/pitch de Euler ZYX; parado e nivelado o acelerômetro mede +1 g em z e a
gravidade vista no corpo é [-sin(pitch), sin(roll) cos(pitch), cos(roll) cos(pitch)].
Yaw não é observável sem magnetômetro e não é estimado.

Filtros (todos recebem o giro já sem o bias da calibração inicial):
  mahony         Mahony em quaternion, com estimativa de bias (porte do firmware antigo)
  ekf            EKF [roll, pitch, bias_x, bias_y] (porte do firmware antigo)
Mahony e EKF descartam / desconfiam do acelerômetro com a mesma regra quando
|a| se afasta de 1 g (impactos), para uma comparação justa.
"""
from collections import deque
import math

import numpy as np

# Escalas configuradas no firmware (firmware-milare/src/main.cpp)
ACCEL_LSB_PER_G = 16384.0      # ±2 g
GYRO_LSB_PER_DPS = 65.5        # ±500 °/s
GRAVITY = 9.80665
DEG = math.pi / 180.0

FILTER_NAMES = ('mahony', 'ekf')

# Parâmetros padrão (mesmos nomes dos parâmetros ROS do nó e do YAML)
DEFAULTS = {
    # Eixos do corpo em função dos eixos do sensor, ex. ['-y', 'x', 'z']:
    # x_corpo = -y_sensor, y_corpo = x_sensor, z_corpo = z_sensor
    'axes': ['x', 'y', 'z'],
    'gyro_calib.samples': 400,            # 2 s parado a 200 Hz
    'gyro_calib.max_std_dps': 0.5,        # desvio máximo do giro para considerar "parado"
    'gyro_calib.max_accel_dev_g': 0.05,   # | |a| - 1 g | máximo durante a calibração
    'gyro_calib.timeout_s': 10.0,         # depois disso usa a janela mais parada vista
    'max_dt': 0.05,                       # s, limita dt após perdas de amostra
    'mahony.kp': 1.0,                     # 1/s
    'mahony.ki': 0.02,
    'mahony.integral_limit': 0.1,         # rad/s
    'mahony.accel_dev_scale': 0.05,       # g
    'mahony.accel_reject': 0.3,           # g
    'ekf.gyro_noise': 0.002,              # rad/s/sqrt(Hz)
    'ekf.bias_noise': 0.0001,             # rad/s/sqrt(s)
    'ekf.accel_noise': 0.05,              # ~rad
    'ekf.accel_dev_scale': 0.05,          # g
    'ekf.accel_reject': 0.3,              # g
}


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


def trust_factor(norm_g, dev_scale, reject):
    """Fator de confiança no acelerômetro (0 = descartado)."""
    dev = abs(norm_g - 1.0)
    if norm_g < 1e-6 or dev > reject:
        return 0.0
    k = dev / dev_scale
    return 1.0 / (1.0 + k * k)


class Mahony:
    def __init__(self, p):
        self.kp = p['mahony.kp']
        self.ki = p['mahony.ki']
        self.ilim = p['mahony.integral_limit']
        self.dev_scale = p['mahony.accel_dev_scale']
        self.reject = p['mahony.accel_reject']
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.i = [0.0, 0.0, 0.0]

    def begin(self, a):
        roll, pitch = accel_tilt(*a)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        self.q = [cr * cp, sr * cp, cr * sp, -sr * sp]
        self.i = [0.0, 0.0, 0.0]

    def update(self, g, a, dt):
        q0, q1, q2, q3 = self.q
        gx, gy, gz = g
        norm = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        trust = trust_factor(norm, self.dev_scale, self.reject)
        used = trust > 0.0
        if used:
            ax, ay, az = a[0] / norm, a[1] / norm, a[2] / norm
            vx = 2.0 * (q1 * q3 - q0 * q2)
            vy = 2.0 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
            e = (ay * vz - az * vy, az * vx - ax * vz, ax * vy - ay * vx)
            ki = self.ki * trust
            self.i = [max(-self.ilim, min(self.ilim, self.i[k] + ki * e[k] * dt)) for k in range(3)]
            kp = self.kp * trust
            gx += kp * e[0]
            gy += kp * e[1]
            gz += kp * e[2]
        gx += self.i[0]
        gy += self.i[1]
        gz += self.i[2]
        h = 0.5 * dt
        q0, q1, q2, q3 = (q0 + (-q1 * gx - q2 * gy - q3 * gz) * h,
                          q1 + (q0 * gx + q2 * gz - q3 * gy) * h,
                          q2 + (q0 * gy - q1 * gz + q3 * gx) * h,
                          q3 + (q0 * gz + q1 * gy - q2 * gx) * h)
        n = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if n > 1e-9:
            self.q = [q0 / n, q1 / n, q2 / n, q3 / n]
        return used

    @property
    def roll(self):
        q0, q1, q2, q3 = self.q
        return math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))

    @property
    def pitch(self):
        q0, q1, q2, q3 = self.q
        return math.asin(max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1))))


class Ekf:
    COS_PITCH_MIN = 0.05

    def __init__(self, p):
        self.gyro_noise = p['ekf.gyro_noise']
        self.bias_noise = p['ekf.bias_noise']
        self.accel_noise = p['ekf.accel_noise']
        self.dev_scale = p['ekf.accel_dev_scale']
        self.reject = p['ekf.accel_reject']
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
        qa = self.gyro_noise ** 2 * dt
        qb = self.bias_noise ** 2 * dt
        self.P = F @ self.P @ F.T + np.diag([qa, qa, qb, qb])

        # Correção com o acelerômetro
        norm = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        trust = trust_factor(norm, self.dev_scale, self.reject)
        if trust <= 0.0:
            return False
        z = np.asarray(a) / norm
        phi, th = self.x[0], self.x[1]
        sphi, cphi, sth, cth = math.sin(phi), math.cos(phi), math.sin(th), math.cos(th)
        h = np.array([-sth, sphi * cth, cphi * cth])
        H = np.array([[0.0, -cth, 0.0, 0.0],
                      [cphi * cth, -sphi * sth, 0.0, 0.0],
                      [-sphi * cth, -cphi * sth, 0.0, 0.0]])
        r_var = self.accel_noise ** 2 / trust   # = noise² (1 + k²)
        PHt = self.P @ H.T
        S = H @ PHt + r_var * np.eye(3)
        try:
            K = np.linalg.solve(S, PHt.T).T
        except np.linalg.LinAlgError:
            return False
        self.x += K @ (z - h)
        self._normalize()
        self.P = self.P - K @ PHt.T
        self.P = 0.5 * (self.P + self.P.T)
        return True

    @property
    def roll(self):
        return float(self.x[0])

    @property
    def pitch(self):
        return float(self.x[1])

    def std(self):
        return math.sqrt(max(self.P[0, 0], 0.0)), math.sqrt(max(self.P[1, 1], 0.0))


FILTER_CLASSES = {
    'mahony': Mahony,
    'ekf': Ekf,
}


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


IMU_TAGS = {b'IMU': 0, b'IMU2': 1}   # tag da linha -> índice da IMU


def parse_udp_line(line):
    """b'IMU|IMU2 seq t_us ax ay az gx gy gz pc_ns' -> (idx, seq, ..., pc_ns), ou None.

    idx = 0 para IMU (0x68) e 1 para IMU2 (0x69).
    """
    parts = line.split()
    if len(parts) != 10 or parts[0] not in IMU_TAGS:
        return None
    try:
        return (IMU_TAGS[parts[0]],) + tuple(int(v) for v in parts[1:])
    except ValueError:
        return None


class ImuPipeline:
    """Amostras brutas -> unidades físicas no corpo -> calibração do giro -> filtros."""

    def __init__(self, params, filters=FILTER_NAMES, log=print):
        self.p = dict(DEFAULTS)
        self.p.update(params)
        self.log = log
        self.M = axes_matrix(self.p['axes'])
        self.filter_names = list(filters)
        self.filters = {n: FILTER_CLASSES[n](self.p) for n in self.filter_names}
        self.calib_n = int(self.p['gyro_calib.samples'])
        self.calib_buf = deque(maxlen=self.calib_n)
        self.calib_best = None      # (std máximo, bias) da janela mais parada vista
        self.calib_start_us = None
        self.gyro_bias = None       # rad/s, no corpo
        self.last_t_us = None
        self.accel_used = {}

    @property
    def calibrated(self):
        return self.gyro_bias is not None

    def to_body(self, acc_counts, gyro_counts):
        a = self.M @ (np.asarray(acc_counts, float) / ACCEL_LSB_PER_G)          # g
        g = self.M @ (np.asarray(gyro_counts, float) / GYRO_LSB_PER_DPS * DEG)  # rad/s
        return a, g

    def recalibrate(self):
        self.calib_buf.clear()
        self.calib_best = None
        self.calib_start_us = None
        self.gyro_bias = None

    def _calibrate(self, t_us, a, g):
        if self.calib_start_us is None:
            self.calib_start_us = t_us
        self.calib_buf.append((a, g))
        if len(self.calib_buf) < self.calib_n:
            return False
        acc = np.array([s[0] for s in self.calib_buf])
        gyr = np.array([s[1] for s in self.calib_buf])
        gstd = float(np.max(np.std(gyr, axis=0)))
        adev = float(np.max(np.abs(np.linalg.norm(acc, axis=1) - 1.0)))
        bias = gyr.mean(axis=0)
        if self.calib_best is None or gstd < self.calib_best[0]:
            self.calib_best = (gstd, bias, acc.mean(axis=0))
        if gstd <= self.p['gyro_calib.max_std_dps'] * DEG and adev <= self.p['gyro_calib.max_accel_dev_g']:
            self._finish_calib(bias, acc.mean(axis=0), gstd, ok=True)
            return True
        if (t_us - self.calib_start_us) * 1e-6 > self.p['gyro_calib.timeout_s']:
            gstd, bias, a_mean = self.calib_best
            self._finish_calib(bias, a_mean, gstd, ok=False)
            return True
        for _ in range(self.calib_n // 4):      # desliza a janela
            self.calib_buf.popleft()
        return False

    def _finish_calib(self, bias, a_mean, gstd, ok):
        self.gyro_bias = bias
        for f in self.filters.values():
            f.begin(a_mean)
        msg = (f'bias do giro (°/s): {np.round(bias / DEG, 3).tolist()}, '
               f'desvio na janela {gstd / DEG:.3f} °/s')
        self.log(('calibração do giro ok: ' if ok else
                  'AVISO: robô não ficou parado; calibração com a janela mais parada: ') + msg)

    def process(self, t_us, acc_counts, gyro_counts):
        """Processa uma amostra. Retorna (a_g, g_rad_s sem bias, dt) depois da
        calibração, ou None enquanto calibra. Os ângulos ficam em self.filters."""
        a, g = self.to_body(acc_counts, gyro_counts)
        if not self.calibrated:
            if self._calibrate(t_us, a, g):
                self.last_t_us = t_us
            return None
        dt = 0.0 if self.last_t_us is None else (t_us - self.last_t_us) * 1e-6
        self.last_t_us = t_us
        dt = min(max(dt, 0.0), self.p['max_dt'])
        g = g - self.gyro_bias
        for name, f in self.filters.items():
            self.accel_used[name] = f.update(g, a, dt)
        return a, g, dt

    def angles(self):
        return {n: (f.roll, f.pitch) for n, f in self.filters.items()}
