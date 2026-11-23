"""
Mede o desempenho do pipeline de visão enquanto o vision.launch roda em outro terminal.

    ros2 run camera_calibrator desempenho               # relatório a cada 5 s
    ros2 run camera_calibrator desempenho --janela 2    # a cada 2 s

Ctrl+C mostra o resumo da sessão inteira.

O que é medido:
  - câmera: frames por segundo publicados pelo usb_cam (pelo /camera/camera_info,
    que sai junto com cada imagem e tem o mesmo timestamp, mas é leve: assim o
    medidor não rouba CPU do pipeline deserializando imagens);
  - YOLO (finder): detecções por segundo, quantos frames da câmera ele processou
    e a latência frame publicado pela câmera -> detecção publicada;
  - IPM: saídas por segundo e latência frame publicado -> posição publicada;
  - CPU e memória de cada processo (usb_cam, finder, ipm) e da máquina.

A latência é medida casando o timestamp de cada detecção com o do camera_info
do mesmo frame (o finder e o IPM mantêm o header da imagem), e comparando as
horas de chegada aqui. Não usa o valor do timestamp em si: o usb_cam carimba
as imagens com um relógio deslocado (~0,4 s nesta máquina), o que tornaria
"agora - stamp" enganoso. Não inclui o tempo entre a captura no sensor e a
publicação pelo usb_cam.
"""

import argparse
from collections import OrderedDict
import sys
import time

import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo
import soccer_vision_2d_msgs.msg as sv2dm
import soccer_vision_3d_msgs.msg as sv3dm

# Processos do vision.launch: nome no relatório -> fim do caminho do executável
PROCESSOS = {
    'usb_cam': '/usb_cam_node_exe',
    'finder': '/object_finder/finder',
    'ipm': '/soccer_ipm/ipm',
}

# Quantos frames lembrar para casar detecções com a chegada do frame (~10 s a 30 fps)
MAX_FRAMES = 300


class Medidas:
    """Acumula chegadas e latências de um tópico (janela atual e sessão inteira)."""

    def __init__(self):
        self.janela, self.total = [], []
        self.n_janela = self.n_total = 0

    def registrar(self, latencia_ms):
        self.n_janela += 1
        self.n_total += 1
        if latencia_ms is not None:
            self.janela.append(latencia_ms)
            self.total.append(latencia_ms)

    def fechar_janela(self):
        n, lat = self.n_janela, self.janela
        self.n_janela, self.janela = 0, []
        return n, lat


def resumo_latencia(lat):
    if not lat:
        return '      -'
    a = np.asarray(lat)
    return f'{a.mean():6.0f} ms (p95 {np.percentile(a, 95):4.0f}, máx {a.max():4.0f})'


class Desempenho(Node):

    def __init__(self, janela):
        super().__init__('desempenho')
        self.janela = janela
        self.inicio = self.ultimo = time.monotonic()

        self.camera = Medidas()
        self.yolo = Medidas()
        self.ipm = Medidas()

        # timestamp do frame -> hora (monotônica) em que o camera_info chegou
        self.chegada = OrderedDict()

        confiavel = QoSProfile(depth=10)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.ao_receber_frame, qos_profile_sensor_data)
        # O finder publica as 4 mensagens a cada frame; a de bolas basta para contar
        self.create_subscription(
            sv2dm.BallArray, 'balls_in_image',
            lambda m: self.yolo.registrar(self.latencia_ms(m)), confiavel)
        self.create_subscription(
            sv3dm.BallArray, 'balls_relative',
            lambda m: self.ipm.registrar(self.latencia_ms(m)), confiavel)

        self.processos = {}
        self.cpu_total = []
        self.cpu_proc = {nome: [] for nome in PROCESSOS}
        psutil.cpu_percent()  # primeira leitura zera a referência
        self.create_timer(janela, self.relatorio)
        self.get_logger().info(
            f'Medindo a cada {janela:g} s. Ctrl+C para o resumo final.')

    @staticmethod
    def stamp(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def ao_receber_frame(self, msg):
        self.chegada[self.stamp(msg)] = time.monotonic()
        if len(self.chegada) > MAX_FRAMES:
            self.chegada.popitem(last=False)
        self.camera.registrar(None)

    def latencia_ms(self, msg):
        t = self.chegada.get(self.stamp(msg))
        if t is None:
            return None
        return (time.monotonic() - t) * 1000

    def achar_processos(self):
        """(Re)encontra os processos do launch; chamado a cada janela."""
        for nome, trecho in PROCESSOS.items():
            p = self.processos.get(nome)
            if p is not None and p.is_running():
                continue
            self.processos.pop(nome, None)
            for proc in psutil.process_iter(['cmdline']):
                if any(parte.endswith(trecho) for parte in (proc.info['cmdline'] or [])[:2]):
                    proc.cpu_percent()  # primeira leitura zera a referência
                    self.processos[nome] = proc
                    break

    def relatorio(self):
        agora = time.monotonic()
        dt, self.ultimo = agora - self.ultimo, agora

        n_cam, _ = self.camera.fechar_janela()
        n_yolo, lat_yolo = self.yolo.fechar_janela()
        n_ipm, lat_ipm = self.ipm.fechar_janela()

        linhas = [f'--- últimos {dt:.1f} s ' + '-' * 40]
        linhas.append(f'câmera  {n_cam / dt:5.1f} fps')
        proc = f'  ({100 * n_yolo / n_cam:3.0f}% dos frames)' if n_cam else ''
        linhas.append(f'YOLO    {n_yolo / dt:5.1f} det/s{proc}   latência {resumo_latencia(lat_yolo)}')
        if n_ipm:
            linhas.append(f'IPM     {n_ipm / dt:5.1f} saídas/s     latência {resumo_latencia(lat_ipm)}')
        else:
            linhas.append('IPM     sem saída (sem bola na imagem ou sem TF da câmera)')

        self.achar_processos()
        cpu = psutil.cpu_percent()
        self.cpu_total.append(cpu)
        partes = []
        for nome in PROCESSOS:
            p = self.processos.get(nome)
            if p is None:
                partes.append(f'{nome} não encontrado')
                continue
            try:
                c = p.cpu_percent()
                mem = p.memory_info().rss / 2**20
            except psutil.NoSuchProcess:
                partes.append(f'{nome} encerrou')
                continue
            self.cpu_proc[nome].append(c)
            partes.append(f'{nome} {c:4.0f}% {mem:5.0f} MB')
        linhas.append('CPU     ' + ' | '.join(partes))
        linhas.append(f'        máquina {cpu:3.0f}% ({psutil.cpu_count()} núcleos; '
                      f'100% = 1 núcleo nos processos)')
        print('\n'.join(linhas), flush=True)

    def resumo_final(self):
        dt = time.monotonic() - self.inicio
        if dt < 1:
            return
        cam, yolo, ipm = self.camera.n_total, self.yolo.n_total, self.ipm.n_total
        print(f'\n=== Resumo de {dt:.0f} s ' + '=' * 40)
        print(f'câmera  {cam / dt:5.1f} fps')
        proc = f'  ({100 * yolo / cam:3.0f}% dos frames)' if cam else ''
        print(f'YOLO    {yolo / dt:5.1f} det/s{proc}   latência {resumo_latencia(self.yolo.total)}')
        if ipm:
            print(f'IPM     {ipm / dt:5.1f} saídas/s     latência {resumo_latencia(self.ipm.total)}')
        cpus = ' | '.join(
            f'{nome} {np.mean(v):4.0f}%' for nome, v in self.cpu_proc.items() if v)
        if cpus:
            print(f'CPU média  {cpus} | máquina {np.mean(self.cpu_total):3.0f}%')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--janela', type=float, default=5.0,
                        help='Intervalo entre relatórios, em segundos (padrão 5)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Desempenho(args.janela)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.resumo_final()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
