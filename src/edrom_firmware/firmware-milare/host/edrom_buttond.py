#!/usr/bin/env python3
"""Daemon dos botões do robô EDROM.

Roda no boot do PC (systemd), lê o ESP32 pela USB e inicia/para o bringup,
sem precisar de SSH, teclado ou cabo de rede.

Botões:
  1         -> FIRST POSE: walking.launch.py + direct_control.launch.py
  2         -> BRINGUP completo (modo JOGO; desativado até o novo bringup,
               ligue com EDROM_JOGO_ENABLED=1)
  3 (toque) -> ANDAR: /cmd_vel x = WALK_SPEED (0.12 m/s) para frente, ao SOLTAR
               (com o first pose ou o bringup rodando)
  segurar 3 -> PARAR DE ANDAR (2 s): /cmd_vel = 0 e, HALT_DELAY_S (1.5 s) depois,
               /stop_walking. Por isso o 3 age ao soltar: segurar não anda antes.
  4         -> DESLIGA tudo (modos e visão)
  5         -> VISÃO + SEARCH AND TRACK (object_finder + cabeça), para testar
               detecção e rastreamento. Sobe o first pose antes se preciso,
               porque a cabeça é movida pelo direct_controller.
  segurar 4 -> desliga tudo e desliga o PC

Só um modo (first pose / bringup) roda por vez; a visão do botão 5 roda em
paralelo ao first pose, como um grupo separado.

IMU: o mesmo ESP32 lê o MPU-6050 e manda amostras brutas ("IMU ...").
Este daemon é o único dono da serial, então repassa cada linha IMU por UDP
local (EDROM_IMU_UDP), acrescentando o instante de chegada no PC em ns
(time.time_ns(), mesmo relógio dos stamps do ROS):
    "IMU <seq> <t_us> <ax> <ay> <az> <gx> <gy> <gz> <pc_ns>"
As linhas "IMU2 ..." (segundo MPU-6050, 0x69) vão do mesmo jeito para
EDROM_IMU2_UDP: uma porta por IMU, cada uma com a sua instância do nó.
O nó imu_filters (pacote imu_ros_arduino) recebe, filtra e publica no ROS.
Uma thread só lê a serial, para o repasse não parar enquanto um launch
está sendo parado.

Configuração por variáveis de ambiente (veja edrom-buttons.service):
  EDROM_BTN_PORT     porta serial do ESP32 (padrão /dev/edrom_buttons)
  EDROM_BTN_BAUD     baud da serial (padrão 460800, igual ao firmware)
  EDROM_IMU_UDP      destino host:porta das linhas IMU (padrão 127.0.0.1:5600)
  EDROM_IMU2_UDP     destino host:porta das linhas IMU2 (padrão 127.0.0.1:5601)
  EDROM_WS           workspace (padrão ~/edrom_main)
  EDROM_LAUNCH       comando ros2 launch do modo JOGO
  EDROM_POSE_LAUNCHES  comandos do modo POSE, separados por ';'
  EDROM_VISION_LAUNCHES  comandos do botão 5 (visão), separados por ';'
  EDROM_LOG_DIR      pasta dos logs do bringup (padrão $EDROM_WS/logs_robot)
"""
import datetime
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
import time

import serial

PORT = os.environ.get('EDROM_BTN_PORT', '/dev/edrom_buttons')
BAUD = int(os.environ.get('EDROM_BTN_BAUD', '460800'))


def _udp_dest(var, default):
    host, port = os.environ.get(var, default).rsplit(':', 1)
    return host, int(port)


IMU_UDP = _udp_dest('EDROM_IMU_UDP', '127.0.0.1:5600')
IMU2_UDP = _udp_dest('EDROM_IMU2_UDP', '127.0.0.1:5601')
IMU_ERR_LOG_S = 30.0   # repete o aviso de IMU com erro no máximo a cada 30 s
WS = os.path.expanduser(os.environ.get('EDROM_WS', '~/edrom_main'))
LAUNCH = os.environ.get(
    'EDROM_LAUNCH',
    "ros2 launch edrom_bringup robot.launch.py camera_idx:='0' "
    "img_output:=False teleop:=False",
)
POSE_LAUNCHES = os.environ.get(
    'EDROM_POSE_LAUNCHES',
    'ros2 launch aurea_walk walking.launch.py teleop:=False;'
    'ros2 launch edrom_lowlevel direct_control.launch.py',
)
# Modo JOGO desativado até o novo bringup ficar pronto.
# Para ativar: Environment=EDROM_JOGO_ENABLED=1 no edrom-buttons.service
JOGO_ENABLED = os.environ.get('EDROM_JOGO_ENABLED', '0') == '1'
# Botão 5: visão sem janela (o serviço não tem tela) + controle da cabeça
VISION_LAUNCHES = os.environ.get(
    'EDROM_VISION_LAUNCHES',
    'ros2 launch object_finder vision.launch.py img_output:=False;'
    'ros2 run vision_controller search_and_track_node --ros-args -r __node:=head_controller_node;'
    # sem o behaviour, alguém precisa mandar SEARCHING/TRACKING para a cabeça
    'python3 ' + os.path.join(os.path.dirname(os.path.abspath(__file__)), 'head_test_supervisor.py'),
)
MODES = {
    'jogo': [LAUNCH] if JOGO_ENABLED else [],
    'pose': [c.strip() for c in POSE_LAUNCHES.split(';') if c.strip()],
    'visao': [c.strip() for c in VISION_LAUNCHES.split(';') if c.strip()],
}
LOG_DIR = os.environ.get('EDROM_LOG_DIR', os.path.join(WS, 'logs_robot'))

# Comando do botão 3 (roda no ambiente ROS, sem bloquear o daemon).
# Ex.: 'ros2 topic pub --once /robot_command std_msgs/msg/String "{data: calibrar}"'
CUSTOM_CMDS = {
    3: os.environ.get('EDROM_BTN3_CMD', ''),
}

WALK_SPEED = float(os.environ.get('EDROM_WALK_SPEED', '0.12'))   # m/s
HALT_DELAY_S = float(os.environ.get('EDROM_HALT_DELAY', '1.5'))   # s
BTN3_HOLD_MS = 2000   # igual a HOLD_MS[2] do firmware
HELPER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ros_cmd_helper.py')

ROS_ENV = f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash'
STOP_TIMEOUT_S = 10.0
CONNECT_GRACE_S = 3.0      # ignora botões logo após conectar/resetar o ESP32
# ESP32 mudo (sem HELLO) por esse tempo: reseta pelo EN. No boot do PC algo
# (ex.: ModemManager) pode mexer em DTR/RTS e deixar o ESP32 no modo gravação.
SILENT_RESET_S = 3.0
SILENT_RESET_MIN_GAP_S = 10.0
POWEROFF_MIN_UPTIME_S = 180  # nunca desliga o PC nos primeiros 3 min (evita loop de boot)
# Modo de teste: mostra o que cada botão faria, sem subir launches nem desligar o PC
DRY_RUN = os.environ.get('EDROM_DRY_RUN', '0') == '1'
ALLOW_POWEROFF = os.environ.get('EDROM_ALLOW_POWEROFF', '1') == '1'
START_GRACE_S = 5.0  # tempo em "START" antes de considerar "RUN"


def log(msg):
    print(f'[edrom_buttond] {msg}', flush=True)


class Robot:
    """Gerencia o modo ativo (lista de processos ros2 launch)."""

    def __init__(self):
        self.mode = None
        self.procs = []
        self.started_at = 0.0
        self.crashed = False

    def running(self):
        return bool(self.procs) and all(p.poll() is None for p in self.procs)

    def state(self):
        if self.running():
            return 'START' if time.time() - self.started_at < START_GRACE_S else 'RUN'
        if self.procs:  # algum launch terminou sem ser pedido
            codes = [p.poll() for p in self.procs]
            log(f'modo {self.mode} caiu sozinho (códigos {codes}); parando o resto')
            self._kill_all()
            self.crashed = True
        return 'ERR' if self.crashed else 'IDLE'

    def start(self, mode):
        if not MODES[mode]:
            log(f'modo {mode} desativado: nada a fazer')
            return
        if self.running() and self.mode == mode:
            log(f'modo {mode} já está rodando')
            return
        if self.procs:
            self.stop()
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.mode = mode
        for i, cmd in enumerate(MODES[mode]):
            path = os.path.join(LOG_DIR, f'{mode}_{stamp}_{i}.log')
            if DRY_RUN:
                log(f'[TESTE] [{mode}] rodaria: {cmd}')
                self.procs.append(subprocess.Popen(
                    ['sleep', 'infinity'], start_new_session=True))
                continue
            log(f'[{mode}] iniciando: {cmd}  (log: {path})')
            logf = open(path, 'w')
            self.procs.append(subprocess.Popen(
                f'{ROS_ENV} && exec {cmd}',
                shell=True,
                executable='/bin/bash',
                cwd=WS,
                stdout=logf,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                start_new_session=True,  # grupo próprio: mata a árvore toda
            ))
            logf.close()
        self.started_at = time.time()
        self.crashed = False

    def restart(self):
        mode = self.mode
        if mode is None:
            log('nada para reiniciar')
            return
        self.stop()
        self.start(mode)

    def stop(self):
        self.crashed = False
        if self.procs:
            log(f'parando modo {self.mode}')
        self._kill_all()

    def _kill_all(self):
        alive = [p for p in self.procs if p.poll() is None]
        # SIGINT = Ctrl+C: o ros2 launch encerra os nós de forma limpa
        for sig, wait in ((signal.SIGINT, STOP_TIMEOUT_S),
                          (signal.SIGTERM, 3.0),
                          (signal.SIGKILL, 2.0)):
            if not alive:
                break
            for p in alive:
                try:
                    os.killpg(p.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.time() + wait
            for p in alive:
                try:
                    p.wait(timeout=max(0.0, deadline - time.time()))
                except subprocess.TimeoutExpired:
                    pass
            alive = [p for p in alive if p.poll() is None]
            if alive:
                log(f'{len(alive)} processo(s) não pararam com {sig.name}, escalando')
        self.procs = []


class RosBridge:
    """Processo ros_cmd_helper.py sempre aberto, para publicar sem atraso."""

    def __init__(self):
        self.proc = None

    def ensure_started(self):
        """Sobe a ponte se ainda não estiver rodando. Chamado ao subir o modo
        pose, para a descoberta do DDS já estar feita quando o 1 for apertado."""
        if DRY_RUN:
            return
        if self.proc is None or self.proc.poll() is not None:
            os.makedirs(LOG_DIR, exist_ok=True)
            logf = open(os.path.join(LOG_DIR, 'ros_bridge.log'), 'a')
            self.proc = subprocess.Popen(
                f'{ROS_ENV} && exec python3 -u {HELPER}', shell=True,
                executable='/bin/bash', cwd=WS, stdin=subprocess.PIPE,
                stdout=logf, stderr=subprocess.STDOUT, text=True,
                start_new_session=True)
            logf.close()
            log('ponte ROS iniciada')

    def send(self, cmd):
        if DRY_RUN:
            log(f'[TESTE] publicaria: {cmd}')
            return
        self.ensure_started()
        try:
            self.proc.stdin.write(cmd + '\n')
            self.proc.stdin.flush()
            log(f'enviado: {cmd}')
        except (BrokenPipeError, OSError) as e:
            log(f'ponte ROS caiu ({e}); tente de novo')
            self.proc = None

    def close(self):
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
                self.proc.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except Exception:
                    pass


# Grupo da visão (botão 5): roda em paralelo ao modo first pose
VISION = Robot()


def walking_ready(robot):
    if robot.running() and robot.mode in ('pose', 'jogo'):
        return True
    log('walking não está rodando: aperte 1 (first pose) antes')
    return False


def run_custom(n):
    cmd = CUSTOM_CMDS.get(n)
    if not cmd:
        log(f'botão {n} sem comando configurado')
        return
    if DRY_RUN:
        log(f'[TESTE] botão {n} rodaria: {cmd}')
        return
    log(f'botão {n}: {cmd}')
    subprocess.Popen(f'{ROS_ENV} && {cmd}', shell=True, executable='/bin/bash',
                     cwd=WS, start_new_session=True,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def uptime_s():
    with open('/proc/uptime') as f:
        return float(f.read().split()[0])


def handle(line, robot, bridge, connected_at, pins, accepted, fw):
    parts = line.split()
    if len(parts) == 3 and parts[0] == 'HELLO':
        # estado bruto dos pinos (1 = solto, 0 = em GND); loga quando muda
        if parts[2] != pins[0]:
            log(f'pinos dos botões 1..5: {parts[2]}  (1=solto, 0=fechado/GND)')
            pins[0] = parts[2]
        return
    if parts and parts[0] in ('BTN', 'HOLD', 'UP') and time.time() - connected_at < CONNECT_GRACE_S:
        log(f'ignorado (ESP32 acabou de conectar): {line}')
        return
    if parts and parts[0] == 'BTN' and parts[-1].isdigit():
        accepted.add(int(parts[-1]))
    if parts and parts[0] in ('HOLD', 'UP') and len(parts) >= 2 and parts[1].isdigit() \
            and int(parts[1]) not in accepted:
        log(f'ignorado (aperto inicial não foi aceito): {line}')
        return
    if parts and parts[0] == 'UP':
        fw['has_up'] = True
    if parts and parts[0] == 'UP' and len(parts) == 3 and parts[1].isdigit() and parts[2].isdigit():
        n, held_ms = int(parts[1]), int(parts[2])
        if n == 3 and held_ms < BTN3_HOLD_MS:   # toque curto no 3: anda
            log(f'botão 3 solto após {held_ms} ms: andar')
            if walking_ready(robot):
                bridge.send(f'VEL {WALK_SPEED}')
        return
    if len(parts) != 2 or not parts[1].isdigit():
        return
    kind, n = parts[0], int(parts[1])
    if kind == 'BTN':
        log(f'botão {n}')
        if n == 1:
            if VISION.running() and robot.mode != 'pose':
                VISION.stop()
            robot.start('pose')
            if robot.running():
                bridge.ensure_started()
        elif n == 2:
            if MODES['jogo']:
                VISION.stop()  # o bringup já sobe a visão e a cabeça
            robot.start('jogo')
            if robot.running():
                bridge.ensure_started()
        elif n == 3:
            # firmware novo: age ao soltar (toque = anda) ou ao segurar 2 s (para).
            # firmware antigo (sem "UP"): anda já no aperto, como antes.
            if not fw['has_up'] and walking_ready(robot):
                bridge.send(f'VEL {WALK_SPEED}')
        elif n == 4:
            VISION.stop()
            robot.stop()
        elif n == 5:
            if robot.running() and robot.mode == 'jogo':
                log('o bringup já inclui visão e search and track')
            else:
                if not (robot.running() and robot.mode == 'pose'):
                    robot.start('pose')  # a cabeça precisa do direct_controller
                    bridge.ensure_started()
                VISION.start('visao')
    elif kind == 'HOLD' and n == 3:
        log('botão 3 segurado: parar de andar')
        if walking_ready(robot):
            bridge.send(f'HALT {HALT_DELAY_S}')
    elif kind == 'HOLD' and n == 4:
        if not ALLOW_POWEROFF:
            log('botão 4 segurado: desligar o PC está desativado')
            return
        if uptime_s() < POWEROFF_MIN_UPTIME_S:
            log('botão 4 segurado, mas o PC ligou há pouco: NÃO vou desligar')
            return
        if DRY_RUN:
            log('[TESTE] botão 4 segurado: desligaria o PC')
            return
        log('botão 4 segurado: desligando o PC')
        VISION.stop()
        robot.stop()
        subprocess.call(['sudo', '-n', '/usr/bin/systemctl', 'poweroff'])


class SerialReader(threading.Thread):
    """Lê a serial sem parar: linhas IMU vão direto para o UDP, o resto
    (botões, HELLO, IMUSTAT) vai para a fila da thread principal.
    Em erro de serial coloca None na fila e termina."""

    def __init__(self, ser, lines):
        super().__init__(daemon=True)
        self.ser = ser
        self.lines = lines
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.n_imu = 0

    def run(self):
        buf = b''
        try:
            while True:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue
                now_ns = time.time_ns()
                buf += chunk
                *done, buf = buf.split(b'\n')
                if len(buf) > 512:   # lixo sem '\n' (ex.: boot do ESP32 em outro baud)
                    buf = b''
                for raw in done:
                    raw = raw.strip()
                    dest = (IMU_UDP if raw.startswith(b'IMU ') else
                            IMU2_UDP if raw.startswith(b'IMU2 ') else None)
                    if dest:
                        self.n_imu += 1
                        try:
                            self.udp.sendto(raw + b' %d' % now_ns, dest)
                        except OSError:
                            pass   # ninguém escutando ainda: descarta
                    elif raw:
                        self.lines.put(raw.decode(errors='ignore'))
        except (serial.SerialException, OSError) as e:
            self.lines.put(None)
            log(f'leitura da serial falhou ({e})')
        finally:
            self.udp.close()


def handle_imustat(parts, imu_state):
    # "IMUSTAT OK 0x68" / "IMUSTAT ERR 0x00": loga quando muda (erro: no máximo a cada 30 s)
    # Firmware com 2 IMUs: "IMUSTAT OK 0x68 OK 0x68" (os 2 últimos campos são da IMU2, 0x69)
    status = ' '.join(parts[1:3])
    now = time.time()
    if status != imu_state['status'] or (
            parts[1:2] == ['ERR'] and now - imu_state['logged_at'] > IMU_ERR_LOG_S):
        log(f'IMU: {status}' + ('  (MPU-6050 não responde no I2C: SDA=21, SCL=22)'
                                if parts[1:2] == ['ERR'] else ''))
        imu_state['status'] = status
        imu_state['logged_at'] = now
    status2 = ' '.join(parts[3:5])
    if status2 and (status2 != imu_state['status2'] or (
            parts[3:4] == ['ERR'] and now - imu_state['logged_at2'] > IMU_ERR_LOG_S)):
        log(f'IMU2: {status2}' + ('  (MPU-6050 0x69 não responde no I2C: AD0 em 3V3?)'
                                  if parts[3:4] == ['ERR'] else ''))
        imu_state['status2'] = status2
        imu_state['logged_at2'] = now


def open_port():
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = BAUD
    ser.timeout = 0.2
    # Evita resetar o ESP32 ao abrir a porta
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def reset_esp(ser):
    """Pulso no EN com GPIO0 solto (DTR=0): boot normal, não modo gravação."""
    try:
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        ser.reset_input_buffer()
    except (serial.SerialException, OSError) as e:
        log(f'não consegui resetar o ESP32 ({e})')


def main():
    robot = Robot()
    bridge = RosBridge()

    def on_exit(*_):
        bridge.close()
        VISION.stop()
        robot.stop()
        sys.exit(0)

    signal.signal(signal.SIGTERM, on_exit)
    signal.signal(signal.SIGINT, on_exit)

    if DRY_RUN:
        log('*** MODO TESTE: nenhum launch será iniciado e o PC não será desligado ***')
    log(f'aguardando ESP32 em {PORT}')
    while True:
        try:
            ser = open_port()
        except (serial.SerialException, OSError):
            robot.state()  # mantém o estado do bringup atualizado
            VISION.state()
            time.sleep(1.0)
            continue

        log(f'ESP32 conectado ({BAUD} baud); IMU -> udp://{IMU_UDP[0]}:{IMU_UDP[1]}, '
            f'IMU2 -> udp://{IMU2_UDP[0]}:{IMU2_UDP[1]}')
        connected_at = time.time()
        pins = ['']
        accepted = set()
        fw = {'has_up': False}  # vira True no primeiro "UP" (firmware novo)
        imu_state = {'status': '', 'logged_at': 0.0, 'status2': '', 'logged_at2': 0.0}
        last_tx = 0.0
        last_rx = time.time()
        last_reset = 0.0
        lines = queue.Queue()
        reader = SerialReader(ser, lines)
        reader.start()
        try:
            while True:
                try:
                    line = lines.get(timeout=0.2)
                except queue.Empty:
                    line = ''
                if line is None:
                    raise serial.SerialException('thread de leitura parou')
                if line.startswith('IMUSTAT'):
                    handle_imustat(line.split(), imu_state)
                elif line:
                    handle(line, robot, bridge, connected_at, pins, accepted, fw)
                now = time.time()
                if line:
                    last_rx = now
                elif now - last_rx > SILENT_RESET_S and now - last_reset > SILENT_RESET_MIN_GAP_S:
                    log(f'ESP32 sem responder há {now - last_rx:.0f} s: resetando pelo EN')
                    reset_esp(ser)
                    last_reset = last_rx = connected_at = time.time()
                    accepted.clear()  # botões só depois do novo período de graça
                if now - last_tx > 0.5:
                    st = robot.state()
                    if VISION.state() == 'ERR':
                        st = 'ERR'  # visão caiu: LED pisca rápido
                    ser.write(f'S {st}\n'.encode())
                    last_tx = now
        except (serial.SerialException, OSError) as e:
            log(f'ESP32 desconectado ({e}); bringup continua como estava')
            try:
                ser.close()
            except Exception:
                pass
            reader.join(timeout=2.0)
            time.sleep(1.0)


if __name__ == '__main__':
    main()
