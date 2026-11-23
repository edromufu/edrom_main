#!/usr/bin/env python3
"""Calibrador de offsets dos motores (motors_direct.yaml).

Liga os motores com todas as juntas em zero, ou seja, cada motor vai para o
seu calibration_offset (o mesmo que o direct_controller faz com q = 0):

    valor_motor = rad_to_value(±0 + calibration_offset)

Você ajusta o offset de cada motor individualmente e, ao salvar, os novos
offsets são escritos no yaml (comentários e formatação preservados).

Uso:
    source ~/edrom_main/install/setup.bash
    python3 ~/edrom_main/src/edrom_utils/offset_calibrator.py   # yaml do edrom_lowlevel
    python3 offset_calibrator.py --mock          # testa a interface sem motores
    python3 offset_calibrator.py --port /dev/ttyUSB1

IMPORTANTE: não rode junto com o direct_controller (mesma porta serial).
Pare o modo pose (botão 4) antes.
"""
import argparse
import glob
import math
import os
import re
import shutil
import sys
import time
import tkinter as tk
from tkinter import messagebox, ttk

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_YAML = os.path.normpath(
    os.path.join(HERE, '..', 'edrom_lowlevel', 'config', 'motors_direct.yaml'))
INSTALL_YAML = os.path.expanduser(
    '~/edrom_main/install/edrom_lowlevel/share/edrom_lowlevel/config/motors_direct.yaml')

# --- Tabela de controle (igual ao direct_controller.hpp) ---
MX = dict(torque=64, goal=116, present=132, profile_vel=112)   # protocolo 2.0
AX = dict(torque=24, goal=30, present=36, moving_speed=32)     # protocolo 1.0

OFFSET_LIMIT = math.pi          # limite absoluto do offset (rad)
SLIDER_RANGE = 1.0              # alcance do slider (rad); o campo aceita mais


# ---------------------------------------------------------------- conversões
def rad_to_value(rad, protocol):
    """Igual a DirectController::rad_to_value (truncamento do static_cast)."""
    if protocol == 2.0:
        return int((rad + math.pi) * (4095.0 / (2.0 * math.pi)))
    return int((rad + 150.0 * math.pi / 180.0) * (1023.0 / (300.0 * math.pi / 180.0)))


def value_to_rad(value, protocol):
    if protocol == 2.0:
        return value * (2.0 * math.pi / 4095.0) - math.pi
    return value * (300.0 * math.pi / 180.0 / 1023.0) - 150.0 * math.pi / 180.0


def value_limits(protocol):
    return (0, 4095) if protocol == 2.0 else (0, 1023)


# ---------------------------------------------------------------- yaml
def load_motors(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    params = data['direct_controller']['ros__parameters']
    motors = []
    for name in params['joint_names']:
        p = params.get(name, {}) or {}
        motors.append(dict(
            name=name,
            id=int(p.get('id', 0)),
            protocol=float(p.get('protocol', 2.0)),
            inverted=bool(p.get('inverted', False)),
            offset=float(p.get('calibration_offset', 0.0)),
        ))
    return params, motors


def fmt_offset(v):
    s = f'{v:.4f}'
    return '0.0' if s in ('0.0000', '-0.0000') else s


def write_offsets(path, offsets):
    """Troca só os valores de calibration_offset, preservando o resto do arquivo.

    Suporta as duas formas usadas no yaml:
      nome: {id: 1, protocol: 2.0, inverted: true, calibration_offset: 0.0}  # coment.
      nome:
        id: 19
        protocol: 1.0
    """
    with open(path) as f:
        lines = f.read().split('\n')

    remaining = dict(offsets)
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        m = re.match(r'^(\s*)([A-Za-z_][\w]*)\s*:\s*(.*)$', line)
        name = m.group(2) if m else None
        if name in remaining:
            indent, rest = m.group(1), m.group(3)
            val = fmt_offset(remaining.pop(name))
            if rest.startswith('{'):
                # forma em linha {…}
                if re.search(r'calibration_offset\s*:', line):
                    line = re.sub(r'(calibration_offset\s*:\s*)[-+0-9.eE]+', r'\g<1>' + val, line)
                else:
                    line = re.sub(r'\s*\}', f', calibration_offset: {val}}}', line, count=1)
                out.append(line)
                i += 1
                continue
            if rest.strip() == '' or rest.strip().startswith('#'):
                # forma em bloco: linhas seguintes mais indentadas
                out.append(line)
                i += 1
                block_start = len(out)
                child_indent = None
                found = False
                while i < len(lines):
                    nxt = lines[i]
                    if nxt.strip() == '':
                        break
                    ind = len(nxt) - len(nxt.lstrip())
                    if ind <= len(indent):
                        break
                    if child_indent is None:
                        child_indent = nxt[:ind]
                    if re.match(r'^\s*calibration_offset\s*:', nxt):
                        nxt = re.sub(r'(calibration_offset\s*:\s*)[-+0-9.eE]+', r'\g<1>' + val, nxt)
                        found = True
                    out.append(nxt)
                    i += 1
                if not found:
                    ci = child_indent if child_indent is not None else indent + '  '
                    out.insert(len(out) if len(out) > block_start else block_start,
                               f'{ci}calibration_offset: {val}')
                continue
        out.append(line)
        i += 1

    if remaining:
        raise RuntimeError(f'não achei no yaml: {", ".join(remaining)}')

    text = '\n'.join(out)
    # confere que o resultado ainda é um yaml válido com os valores certos
    check = yaml.safe_load(text)['direct_controller']['ros__parameters']
    for n, v in offsets.items():
        got = float(check[n].get('calibration_offset', 0.0))
        if abs(got - float(fmt_offset(v))) > 1e-9:
            raise RuntimeError(f'verificação falhou para {n}: {got} != {v}')
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(text)
    os.replace(tmp, path)


# ---------------------------------------------------------------- barramento
class Bus:
    """Acesso aos Dynamixel (protocolos 1.0 e 2.0) via dynamixel_sdk."""

    def __init__(self, port, baud):
        from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        self.ok = COMM_SUCCESS
        self.port = PortHandler(port)
        self.ph = {1.0: PacketHandler(1.0), 2.0: PacketHandler(2.0)}
        if not self.port.openPort():
            raise RuntimeError(f'não consegui abrir {port}')
        if not self.port.setBaudRate(baud):
            raise RuntimeError(f'não consegui definir baud {baud}')

    def _chk(self, proto, res, err):
        return res == self.ok and err == 0

    def ping(self, m):
        _, res, err = self.ph[m['protocol']].ping(self.port, m['id'])
        return res == self.ok

    def read(self, m, addr, size):
        ph = self.ph[m['protocol']]
        fn = {1: ph.read1ByteTxRx, 2: ph.read2ByteTxRx, 4: ph.read4ByteTxRx}[size]
        val, res, err = fn(self.port, m['id'], addr)
        return val if res == self.ok else None

    def write(self, m, addr, size, val):
        ph = self.ph[m['protocol']]
        fn = {1: ph.write1ByteTxRx, 2: ph.write2ByteTxRx, 4: ph.write4ByteTxRx}[size]
        res, err = fn(self.port, m['id'], addr, int(val))
        return res == self.ok

    # alto nível
    def torque(self, m, on):
        t = MX if m['protocol'] == 2.0 else AX
        return self.write(m, t['torque'], 1, 1 if on else 0)

    def goal(self, m, value):
        t = MX if m['protocol'] == 2.0 else AX
        return self.write(m, t['goal'], 4 if m['protocol'] == 2.0 else 2, value)

    def present(self, m):
        t = MX if m['protocol'] == 2.0 else AX
        return self.read(m, t['present'], 4 if m['protocol'] == 2.0 else 2)

    def get_speed(self, m):
        if m['protocol'] == 2.0:
            return self.read(m, MX['profile_vel'], 4)
        return self.read(m, AX['moving_speed'], 2)

    def set_speed(self, m, v):
        if m['protocol'] == 2.0:
            return self.write(m, MX['profile_vel'], 4, v)
        return self.write(m, AX['moving_speed'], 2, v)

    def close(self):
        self.port.closePort()


class MockBus:
    """Motores simulados, para testar a interface sem hardware."""

    def __init__(self, *_):
        self.pos, self.tq = {}, {}

    def ping(self, m):
        return m['id'] != 19  # simula um motor sem resposta

    def torque(self, m, on):
        self.tq[m['id']] = on
        return True

    def goal(self, m, v):
        if self.tq.get(m['id']):
            self.pos[m['id']] = v
        return True

    def present(self, m):
        lo, hi = value_limits(m['protocol'])
        return self.pos.setdefault(m['id'], (lo + hi) // 2 + 30)

    def get_speed(self, m):
        return 0

    def set_speed(self, m, v):
        return True

    def close(self):
        pass


# ---------------------------------------------------------------- utilidades
def resolve_port(yaml_port):
    """Usa a porta do yaml, mas evita pegar o ESP32 dos botões por engano."""
    real = os.path.realpath(yaml_port)
    ftdi = sorted(glob.glob('/dev/serial/by-id/usb-FTDI*'))
    if ftdi and os.path.exists(real) and all(os.path.realpath(f) != real for f in ftdi):
        print(f'[aviso] {yaml_port} não é o U2D2 (FTDI); usando {ftdi[0]}')
        return ftdi[0]
    if not os.path.exists(real) and ftdi:
        return ftdi[0]
    return yaml_port


def port_users(port):
    real = os.path.realpath(port)
    users = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            for fd in os.listdir(f'/proc/{pid}/fd'):
                if os.path.realpath(f'/proc/{pid}/fd/{fd}') == real:
                    with open(f'/proc/{pid}/comm') as f:
                        users.append(f'{f.read().strip()} (pid {pid})')
                    break
        except (PermissionError, FileNotFoundError):
            pass
    return users


# ---------------------------------------------------------------- interface
class App:
    REFRESH_MS = 300

    def __init__(self, root, bus, motors, yaml_path, speed, also_install):
        self.root, self.bus, self.motors = root, bus, motors
        self.yaml_path, self.speed, self.also_install = yaml_path, speed, also_install
        self.saved = {m['name']: m['offset'] for m in motors}
        self.orig_speed = {}
        self.rows = {}

        root.title(f'Calibrador de offsets — {os.path.basename(yaml_path)}')
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        top = ttk.Frame(root, padding=8)
        top.pack(fill='x')
        ttk.Button(top, text='▶ Ligar torque e ir para os offsets (todos)',
                   command=self.go_all).pack(side='left')
        ttk.Button(top, text='Soltar todos (torque off)',
                   command=self.release_all).pack(side='left', padx=6)
        ttk.Label(top, text='  Passo:').pack(side='left')
        self.step = tk.DoubleVar(value=0.01)
        ttk.Combobox(top, textvariable=self.step, width=7, state='readonly',
                     values=(0.001, 0.005, 0.01, 0.02, 0.05, 0.1)).pack(side='left')
        ttk.Label(top, text='rad').pack(side='left')
        ttk.Button(top, text='💾 Salvar no yaml', command=self.save).pack(side='right')
        ttk.Button(top, text='Desfazer (recarregar salvos)',
                   command=self.revert).pack(side='right', padx=6)

        hdr = ttk.Frame(root, padding=(8, 0))
        hdr.pack(fill='x')
        cols = [('Junta', 14), ('ID', 4), ('Status', 13), ('Torque', 7), ('', 4), ('', 3),
                ('Offset (rad)', 10), ('', 3), ('', 4), ('graus', 7), ('Ajuste fino', 30),
                ('Posição lida', 12), ('', 18), ('Salvo', 8)]
        for c, (t, w) in enumerate(cols):
            ttk.Label(hdr, text=t, width=w, font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=c)

        body = ttk.Frame(root, padding=(8, 0))
        body.pack(fill='both', expand=True)
        for r, m in enumerate(motors):
            self._make_row(body, r, m)

        self.status = tk.StringVar(value='Pronto. Aperte "Ligar torque e ir para os offsets".')
        ttk.Label(root, textvariable=self.status, padding=8, foreground='#444').pack(fill='x')

        self._ping_all()
        root.after(self.REFRESH_MS, self.refresh)

    # ---- linhas
    def _make_row(self, parent, r, m):
        n = m['name']
        row = dict(m=m, alive=False, torque=tk.BooleanVar(value=False),
                   off=tk.DoubleVar(value=m['offset']), deg=tk.StringVar(),
                   pres=tk.StringVar(value='—'), st=tk.StringVar(value='?'),
                   saved=tk.StringVar(value=fmt_offset(m['offset'])))
        self.rows[n] = row
        lbl = ttk.Label(parent, text=n + (' (inv)' if m['inverted'] else ''), width=14)
        lbl.grid(row=r, column=0, sticky='w')
        row['label'] = lbl
        ttk.Label(parent, text=str(m['id']), width=4).grid(row=r, column=1)
        row['st_lbl'] = ttk.Label(parent, textvariable=row['st'], width=13)
        row['st_lbl'].grid(row=r, column=2)
        ttk.Checkbutton(parent, variable=row['torque'],
                        command=lambda n=n: self.toggle_torque(n)).grid(row=r, column=3)
        ttk.Button(parent, text='−−', width=3,
                   command=lambda n=n: self.nudge(n, -5)).grid(row=r, column=4)
        ttk.Button(parent, text='−', width=2,
                   command=lambda n=n: self.nudge(n, -1)).grid(row=r, column=5)
        e = ttk.Entry(parent, width=10, justify='right')
        e.grid(row=r, column=6)
        e.bind('<Return>', lambda ev, n=n: self.from_entry(n))
        e.bind('<FocusOut>', lambda ev, n=n: self.from_entry(n))
        row['entry'] = e
        ttk.Button(parent, text='+', width=2,
                   command=lambda n=n: self.nudge(n, +1)).grid(row=r, column=7)
        ttk.Button(parent, text='++', width=3,
                   command=lambda n=n: self.nudge(n, +5)).grid(row=r, column=8)
        ttk.Label(parent, textvariable=row['deg'], width=7, anchor='e').grid(row=r, column=9)
        s = tk.Scale(parent, from_=-SLIDER_RANGE, to=SLIDER_RANGE, resolution=0.001,
                     orient='horizontal', length=220, showvalue=False,
                     variable=row['off'], command=lambda v, n=n: self.apply(n))
        s.grid(row=r, column=10, padx=4)
        ttk.Label(parent, textvariable=row['pres'], width=12, anchor='e').grid(row=r, column=11)
        ttk.Button(parent, text='Usar posição atual', width=17,
                   command=lambda n=n: self.capture(n)).grid(row=r, column=12, padx=4)
        ttk.Label(parent, textvariable=row['saved'], width=8, anchor='e').grid(row=r, column=13)
        self._show(n)

    def _show(self, n):
        row = self.rows[n]
        v = row['off'].get()
        row['entry'].delete(0, 'end')
        row['entry'].insert(0, f'{v:.4f}')
        row['deg'].set(f'{math.degrees(v):+.2f}°')
        dirty = abs(v - self.saved[n]) > 5e-5
        row['label'].configure(foreground='#c05000' if dirty else '')

    # ---- ações
    def _ping_all(self):
        missing = []
        for n, row in self.rows.items():
            m = row['m']
            row['alive'] = self.bus.ping(m)
            if row['alive']:
                row['st'].set('OK')
                row['st_lbl'].configure(foreground='#107010')
                sp = self.bus.get_speed(m)
                self.orig_speed[n] = sp if sp is not None else 0
            else:
                row['st'].set('sem resposta')
                row['st_lbl'].configure(foreground='#b00020')
                missing.append(n)
        if missing:
            self.status.set(f'Sem resposta: {", ".join(missing)} (alimentação/cabo?)')

    def set_offset(self, n, v):
        v = max(-OFFSET_LIMIT, min(OFFSET_LIMIT, v))
        self.rows[n]['off'].set(round(v, 4))
        self.apply(n)

    def apply(self, n, *_):
        row = self.rows[n]
        self._show(n)
        if row['alive'] and row['torque'].get():
            m = row['m']
            lo, hi = value_limits(m['protocol'])
            val = rad_to_value(row['off'].get(), m['protocol'])
            if not lo <= val <= hi:
                self.status.set(f'{n}: offset fora do alcance do motor ({val})')
                return
            self.bus.goal(m, val)

    def nudge(self, n, mult):
        self.set_offset(n, self.rows[n]['off'].get() + mult * float(self.step.get()))

    def from_entry(self, n):
        try:
            v = float(self.rows[n]['entry'].get().replace(',', '.'))
        except ValueError:
            self._show(n)
            return
        if abs(v - self.rows[n]['off'].get()) > 1e-9:
            self.set_offset(n, v)

    def toggle_torque(self, n):
        row = self.rows[n]
        if not row['alive']:
            row['torque'].set(False)
            return
        m = row['m']
        if row['torque'].get():
            self.bus.set_speed(m, self.speed)
            # vai para o offset atual antes de ligar o torque
            self.bus.goal(m, rad_to_value(row['off'].get(), m['protocol']))
            self.bus.torque(m, True)
            self.bus.goal(m, rad_to_value(row['off'].get(), m['protocol']))
        else:
            self.bus.torque(m, False)

    def go_all(self):
        for n, row in self.rows.items():
            if row['alive']:
                row['torque'].set(True)
                self.toggle_torque(n)
        self.status.set('Torque ligado: motores indo devagar para os offsets (juntas em zero).')

    def release_all(self):
        if not messagebox.askyesno('Soltar todos',
                                   'Desligar o torque de TODOS os motores?\n'
                                   'O robô vai cair se não estiver apoiado.'):
            return
        for n, row in self.rows.items():
            if row['alive']:
                row['torque'].set(False)
                self.bus.torque(row['m'], False)
        self.status.set('Torque desligado em todos.')

    def capture(self, n):
        """offset = posição atual do motor (útil com torque off, posicionando à mão)."""
        row = self.rows[n]
        if not row['alive']:
            return
        raw = self.bus.present(row['m'])
        if raw is None:
            self.status.set(f'{n}: falha ao ler posição')
            return
        self.set_offset(n, value_to_rad(raw, row['m']['protocol']))
        self.status.set(f'{n}: offset = posição atual ({row["off"].get():.4f} rad)')

    def revert(self):
        for n in self.rows:
            self.set_offset(n, self.saved[n])
        self.status.set('Offsets voltaram aos valores salvos.')

    def save(self):
        offsets = {n: row['off'].get() for n, row in self.rows.items()}
        changed = [n for n in offsets if abs(offsets[n] - self.saved[n]) > 5e-5]
        if not changed:
            self.status.set('Nada mudou desde o último salvamento.')
            return
        try:
            stamp = time.strftime('%Y%m%d_%H%M%S')
            shutil.copy2(self.yaml_path, f'{self.yaml_path}.{stamp}.bak')
            write_offsets(self.yaml_path, {n: offsets[n] for n in changed})
            extra = ''
            if self.also_install and os.path.exists(INSTALL_YAML) and \
                    os.path.realpath(INSTALL_YAML) != os.path.realpath(self.yaml_path):
                shutil.copy2(self.yaml_path, INSTALL_YAML)
                extra = ' + install/ (vale sem recompilar)'
        except Exception as e:
            messagebox.showerror('Erro ao salvar', str(e))
            return
        for n in offsets:
            self.saved[n] = float(fmt_offset(offsets[n]))
            self.rows[n]['saved'].set(fmt_offset(offsets[n]))
            self._show(n)
        self.status.set(f'Salvo ({len(changed)} alterado(s): {", ".join(changed)}){extra}. '
                        f'Backup: {os.path.basename(self.yaml_path)}.{stamp}.bak')

    def refresh(self):
        for n, row in self.rows.items():
            if not row['alive']:
                continue
            raw = self.bus.present(row['m'])
            if raw is None:
                row['pres'].set('erro')
            else:
                row['pres'].set(f'{value_to_rad(raw, row["m"]["protocol"]):+.4f}')
        self.root.after(self.REFRESH_MS, self.refresh)

    def on_close(self):
        unsaved = [n for n, row in self.rows.items()
                   if abs(row['off'].get() - self.saved[n]) > 5e-5]
        if unsaved and not messagebox.askyesno(
                'Sair sem salvar?', f'Há offsets não salvos:\n{", ".join(unsaved)}\n\nSair mesmo assim?'):
            return
        any_torque = any(r['torque'].get() for r in self.rows.values())
        off = any_torque and messagebox.askyesno(
            'Torque', 'Desligar o torque dos motores ao sair?\n'
                      '(Sim = robô fica solto; Não = motores ficam segurando a posição)')
        for n, row in self.rows.items():
            if not row['alive']:
                continue
            self.bus.set_speed(row['m'], self.orig_speed.get(n, 0))  # devolve velocidade original
            if off:
                self.bus.torque(row['m'], False)
        self.bus.close()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--yaml', default=DEFAULT_YAML, help='arquivo motors_direct.yaml')
    ap.add_argument('--port', help='porta do U2D2 (padrão: a do yaml)')
    ap.add_argument('--speed', type=int, default=20,
                    help='velocidade de movimento (MX: ×0.229 rpm, AX: ×0.111 rpm). Padrão 20 = lento')
    ap.add_argument('--no-install-copy', action='store_true',
                    help='não copiar o yaml salvo para install/ (aí precisa colcon build)')
    ap.add_argument('--mock', action='store_true', help='sem hardware (testar a interface)')
    args = ap.parse_args()

    params, motors = load_motors(args.yaml)
    if args.mock:
        bus = MockBus()
    else:
        port = args.port or resolve_port(params.get('usb_port', '/dev/ttyUSB0'))
        users = port_users(port)
        if users:
            print(f'ERRO: a porta {port} já está em uso por: {", ".join(users)}')
            print('Pare o direct_controller / modo pose (botão 4) antes de calibrar.')
            sys.exit(1)
        print(f'Abrindo {port} @ {params.get("baud_rate", 1000000)}')
        bus = Bus(port, int(params.get('baud_rate', 1000000)))

    root = tk.Tk()
    App(root, bus, motors, args.yaml, args.speed, not args.no_install_copy)
    root.mainloop()


if __name__ == '__main__':
    main()
