#!/usr/bin/env python3
"""Daemon dos botões do robô EDROM.

Roda no boot do PC (systemd), lê o ESP32 pela USB e inicia/para o bringup,
sem precisar de SSH, teclado ou cabo de rede.

Botões:
  1         -> ANDAR: /cmd_vel x = WALK_SPEED (0.10 m/s) para frente
               (só com o modo POSE rodando)
  2         -> modo POSE: walking.launch.py + direct_control.launch.py
               (vai para a first pose, sem visão/comportamento)
  3         -> PARAR DE ANDAR: /cmd_vel = 0 e, HALT_DELAY_S (1.5 s) depois,
               /stop_walking (volta para a pose inicial)
  4         -> para o que estiver rodando (mata os launches)
  5         -> reinicia o modo atual
  segurar 4 -> para tudo e desliga o PC

O modo JOGO (bringup completo) existe mas está sem botão até o novo bringup.

Configuração por variáveis de ambiente (veja edrom-buttons.service):
  EDROM_BTN_PORT     porta serial do ESP32 (padrão /dev/edrom_buttons)
  EDROM_WS           workspace (padrão ~/edrom_main)
  EDROM_LAUNCH       comando ros2 launch do modo JOGO
  EDROM_POSE_LAUNCHES  comandos do modo POSE, separados por ';'
  EDROM_LOG_DIR      pasta dos logs do bringup (padrão $EDROM_WS/logs_robot)
"""
import datetime
import os
import signal
import subprocess
import sys
import time

import serial

PORT = os.environ.get('EDROM_BTN_PORT', '/dev/edrom_buttons')
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
MODES = {
    'jogo': [LAUNCH] if JOGO_ENABLED else [],
    'pose': [c.strip() for c in POSE_LAUNCHES.split(';') if c.strip()],
}
LOG_DIR = os.environ.get('EDROM_LOG_DIR', os.path.join(WS, 'logs_robot'))

# Comando do botão 3 (roda no ambiente ROS, sem bloquear o daemon).
# Ex.: 'ros2 topic pub --once /robot_command std_msgs/msg/String "{data: calibrar}"'
CUSTOM_CMDS = {
    3: os.environ.get('EDROM_BTN3_CMD', ''),
}

WALK_SPEED = float(os.environ.get('EDROM_WALK_SPEED', '0.10'))   # m/s
HALT_DELAY_S = float(os.environ.get('EDROM_HALT_DELAY', '1.5'))   # s
HELPER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ros_cmd_helper.py')

ROS_ENV = f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash'
STOP_TIMEOUT_S = 10.0
CONNECT_GRACE_S = 3.0      # ignora botões logo após conectar/resetar o ESP32
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

    def send(self, cmd):
        if DRY_RUN:
            log(f'[TESTE] publicaria: {cmd}')
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


def walking_ready(robot):
    if robot.running() and robot.mode == 'pose':
        return True
    log('walking não está rodando: aperte 2 (modo pose) antes')
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


def handle(line, robot, bridge, connected_at, pins, accepted):
    parts = line.split()
    if len(parts) == 3 and parts[0] == 'HELLO':
        # estado bruto dos pinos (1 = solto, 0 = em GND); loga quando muda
        if parts[2] != pins[0]:
            log(f'pinos dos botões 1..5: {parts[2]}  (1=solto, 0=fechado/GND)')
            pins[0] = parts[2]
        return
    if parts and parts[0] in ('BTN', 'HOLD') and time.time() - connected_at < CONNECT_GRACE_S:
        log(f'ignorado (ESP32 acabou de conectar): {line}')
        return
    if parts and parts[0] == 'BTN' and parts[-1].isdigit():
        accepted.add(int(parts[-1]))
    if parts and parts[0] == 'HOLD' and parts[-1].isdigit() and int(parts[-1]) not in accepted:
        log(f'ignorado (aperto inicial não foi aceito): {line}')
        return
    if len(parts) != 2 or not parts[1].isdigit():
        return
    kind, n = parts[0], int(parts[1])
    if kind == 'BTN':
        log(f'botão {n}')
        if n == 1:
            if walking_ready(robot):
                bridge.send(f'VEL {WALK_SPEED}')
        elif n == 3:
            if walking_ready(robot):
                bridge.send(f'HALT {HALT_DELAY_S}')
        elif n == 2:
            robot.start('pose')
        elif n == 4:
            robot.stop()
        elif n == 5:
            robot.restart()
        else:
            run_custom(n)
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
        robot.stop()
        subprocess.call(['sudo', '-n', '/usr/bin/systemctl', 'poweroff'])


def open_port():
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = 115200
    ser.timeout = 0.2
    # Evita resetar o ESP32 ao abrir a porta
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def main():
    robot = Robot()
    bridge = RosBridge()

    def on_exit(*_):
        bridge.close()
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
            time.sleep(1.0)
            continue

        log('ESP32 conectado')
        connected_at = time.time()
        pins = ['']
        accepted = set()
        last_tx = 0.0
        try:
            while True:
                raw = ser.readline()
                if raw:
                    handle(raw.decode(errors='ignore').strip(), robot, bridge, connected_at, pins, accepted)
                now = time.time()
                if now - last_tx > 0.5:
                    ser.write(f'S {robot.state()}\n'.encode())
                    last_tx = now
        except (serial.SerialException, OSError) as e:
            log(f'ESP32 desconectado ({e}); bringup continua como estava')
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1.0)


if __name__ == '__main__':
    main()
