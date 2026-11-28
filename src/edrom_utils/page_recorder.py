#!/usr/bin/env python3
"""Gravador/reprodutor de "pages": sequências de poses dos motores.

Fluxo:
  record  -> solta o torque, você move o robô à mão e captura cada pose.
  play    -> liga o torque, vai até a primeira pose e percorre a sequência
             interpolando linearmente entre as poses.

As posições são salvas como valor bruto do Dynamixel (0-4095 nos MX, 0-1023 nos
AX), então a reprodução não depende de offsets, inversões ou conversões.
Arquivos em src/edrom_utils/pages/<nome>.json.

Uso:
    source ~/edrom_main/install/setup.bash
    python3 page_recorder.py record minha_page            # nova page (ou continua uma existente)
    python3 page_recorder.py play minha_page              # reproduz
    python3 page_recorder.py play minha_page --loop 3 --speed 0.5
    python3 page_recorder.py list
    python3 page_recorder.py show minha_page
    python3 page_recorder.py record teste --mock          # testa sem motores

Comandos dentro do `record` (Enter vazio = capturar):
    c            captura a pose atual
    t <seg>      tempo (s) para chegar nas próximas poses capturadas (padrão 1.0)
    u            desfaz a última pose
    l            lista as poses
    g <n>        vai (com torque) até a pose n, para conferir
    r            solta o torque de novo (depois de um `g`)
    s            salva
    q            salva e sai (qq = sai sem salvar)

IMPORTANTE: não rode junto com o direct_controller (mesma porta serial).
Apoie/segure o robô: ao ligar o torque no `play` ele se move.
"""
import argparse
import json
import os
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from offset_calibrator import (  # noqa: E402  (reaproveita barramento e yaml)
    DEFAULT_YAML, Bus, MockBus, load_motors, resolve_port, port_users, value_limits)

PAGES_DIR = os.path.join(HERE, 'pages')
RATE_HZ = 30.0


# ---------------------------------------------------------------- arquivos
def page_path(name):
    if name.endswith('.json') or os.sep in name:
        return name
    return os.path.join(PAGES_DIR, name + '.json')


def load_page(name):
    with open(page_path(name)) as f:
        return json.load(f)


def save_page(name, page):
    path = page_path(name)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(page, f, indent=2)
    os.replace(tmp, path)
    return path


# ---------------------------------------------------------------- hardware
def open_bus(args):
    params, motors = load_motors(args.yaml)
    if args.mock:
        bus = MockBus()
    else:
        port = args.port or resolve_port(params['usb_port'])
        users = port_users(port)
        if users:
            sys.exit(f'Porta {port} em uso por: {", ".join(users)}. '
                     'Pare o direct_controller antes.')
        bus = Bus(port, int(params.get('baud_rate', 1000000)))
    alive = []
    for m in motors:
        if bus.ping(m):
            alive.append(m)
        else:
            print(f'[aviso] motor {m["name"]} (id {m["id"]}) não respondeu; ignorado.')
    if not alive:
        sys.exit('Nenhum motor respondeu.')
    return bus, alive


def read_pose(bus, motors):
    pose = {}
    for m in motors:
        v = bus.present(m)
        if v is None:
            print(f'[erro] falha lendo {m["name"]}; pose não capturada.')
            return None
        pose[m['name']] = int(v)
    return pose


def set_torque(bus, motors, on):
    for m in motors:
        if not bus.torque(m, on):
            print(f'[aviso] falha ao {"ligar" if on else "soltar"} torque de {m["name"]}')


def send_pose(bus, motors, values):
    for m in motors:
        v = values.get(m['name'])
        if v is None:
            continue
        lo, hi = value_limits(m['protocol'])
        bus.goal(m, max(lo, min(hi, int(round(v)))))


def move(bus, motors, start, end, duration):
    """Interpola linearmente de `start` para `end` em `duration` segundos."""
    duration = max(duration, 0.0)
    t0 = time.monotonic()
    while True:
        a = 1.0 if duration == 0 else min((time.monotonic() - t0) / duration, 1.0)
        send_pose(bus, motors, {n: start[n] + (end[n] - start[n]) * a
                                for n in end if n in start})
        if a >= 1.0:
            break
        time.sleep(1.0 / RATE_HZ)


# ---------------------------------------------------------------- record
def print_poses(page):
    if not page['poses']:
        print('  (sem poses)')
    for i, p in enumerate(page['poses']):
        print(f'  [{i}] tempo={p["time"]:.2f}s')


def cmd_record(args):
    path = page_path(args.name)
    if os.path.exists(path):
        page = load_page(args.name)
        print(f'Continuando page existente com {len(page["poses"])} poses.')
    else:
        page = {'name': args.name, 'poses': []}
    bus, motors = open_bus(args)
    page['motors'] = {m['name']: m['id'] for m in motors}
    default_t = args.time
    dirty = False

    set_torque(bus, motors, False)
    print(__doc__.split('Comandos dentro do `record`')[1].split('IMPORTANTE')[0].strip('\n'))
    print('\nTorque solto: mova o robô à mão e capture.\n')
    try:
        while True:
            try:
                line = input(f'[{len(page["poses"])} poses | t={default_t}s] > ').strip()
            except EOFError:
                line = 'q'
            cmd, _, arg = line.partition(' ')
            cmd = cmd or 'c'
            if cmd == 'c':
                pose = read_pose(bus, motors)
                if pose:
                    page['poses'].append({'time': default_t, 'values': pose})
                    dirty = True
                    print(f'  pose {len(page["poses"]) - 1} capturada.')
            elif cmd == 't':
                try:
                    default_t = float(arg)
                except ValueError:
                    print('  uso: t <segundos>')
            elif cmd == 'u':
                if page['poses']:
                    page['poses'].pop()
                    dirty = True
            elif cmd == 'l':
                print_poses(page)
            elif cmd == 'g':
                try:
                    p = page['poses'][int(arg)]
                except (ValueError, IndexError):
                    print('  uso: g <n>')
                    continue
                cur = read_pose(bus, motors)
                if cur:
                    set_torque(bus, motors, True)
                    move(bus, motors, cur, p['values'], 2.0)
                    print('  na pose. `r` solta o torque.')
            elif cmd == 'r':
                set_torque(bus, motors, False)
            elif cmd == 's':
                print('  salvo em', save_page(args.name, page))
                dirty = False
            elif cmd == 'q':
                if dirty or not os.path.exists(path):
                    print('  salvo em', save_page(args.name, page))
                break
            elif cmd == 'qq':
                break
            else:
                print('  comando desconhecido')
    except KeyboardInterrupt:
        print()
        if dirty and input('Salvar antes de sair? [s/N] ').strip().lower() == 's':
            print('  salvo em', save_page(args.name, page))
    finally:
        set_torque(bus, motors, False)
        bus.close()


# ---------------------------------------------------------------- play
def cmd_play(args):
    page = load_page(args.name)
    if not page['poses']:
        sys.exit('Page sem poses.')
    bus, motors = open_bus(args)
    motors = [m for m in motors if m['name'] in page['motors']]
    missing = set(page['motors']) - {m['name'] for m in motors}
    if missing:
        print(f'[aviso] motores da page sem resposta: {", ".join(sorted(missing))}')

    cur = read_pose(bus, motors)
    if not cur:
        bus.close()
        sys.exit('Não consegui ler a posição atual.')
    first = page['poses'][0]['values']
    print(f'Page "{args.name}": {len(page["poses"])} poses, velocidade x{args.speed}.')
    print(f'O robô vai até a primeira pose em {args.start_time}s e depois executa.')
    if not args.yes and input('Segurando o robô? Enter para iniciar, q para cancelar: ').strip() == 'q':
        bus.close()
        return

    try:
        # trava na posição atual antes de ligar o torque (evita pulo)
        send_pose(bus, motors, cur)
        set_torque(bus, motors, True)
        move(bus, motors, cur, first, args.start_time)

        loops = 0
        while True:
            last = first
            for i, pose in enumerate(page['poses'][1:], start=1):
                print(f'  pose {i}/{len(page["poses"]) - 1}', end='\r')
                move(bus, motors, last, pose['values'], pose['time'] / args.speed)
                last = pose['values']
            loops += 1
            print(f'\nsequência {loops} concluída.')
            if args.loop and loops >= args.loop:
                break
            if not args.loop:
                break
            move(bus, motors, last, first, page['poses'][0]['time'] / args.speed)
    except KeyboardInterrupt:
        print('\nInterrompido.')
    finally:
        if args.release:
            set_torque(bus, motors, False)
            print('Torque solto.')
        else:
            print('Torque mantido ligado (use --release para soltar ao final).')
        bus.close()


# ---------------------------------------------------------------- list/show
def cmd_list(_):
    if not os.path.isdir(PAGES_DIR):
        print('Nenhuma page gravada.')
        return
    for f in sorted(os.listdir(PAGES_DIR)):
        if f.endswith('.json'):
            with open(os.path.join(PAGES_DIR, f)) as fh:
                p = json.load(fh)
            total = sum(x['time'] for x in p['poses'][1:])
            print(f'{f[:-5]:30s} {len(p["poses"])} poses, {total:.1f}s')


def cmd_show(args):
    page = load_page(args.name)
    print(f'{args.name}: {len(page["poses"])} poses')
    for i, p in enumerate(page['poses']):
        vals = ' '.join(f'{n}={v}' for n, v in p['values'].items())
        print(f'[{i}] t={p["time"]:.2f}s  {vals}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--yaml', default=DEFAULT_YAML, help='motors_direct.yaml')
    ap.add_argument('--port', help='porta serial (padrão: a do yaml)')
    ap.add_argument('--mock', action='store_true', help='motores simulados')
    sub = ap.add_subparsers(dest='cmd', required=True)

    r = sub.add_parser('record', help='gravar uma page')
    r.add_argument('name')
    r.add_argument('--time', type=float, default=1.0, help='tempo padrão entre poses (s)')
    r.set_defaults(fn=cmd_record)

    p = sub.add_parser('play', help='reproduzir uma page')
    p.add_argument('name')
    p.add_argument('--speed', type=float, default=1.0, help='multiplicador de velocidade')
    p.add_argument('--start-time', type=float, default=3.0,
                   help='tempo (s) para ir da posição atual à primeira pose')
    p.add_argument('--loop', type=int, default=0, help='repetições (0 = uma vez)')
    p.add_argument('--release', action='store_true', help='soltar o torque ao final')
    p.add_argument('-y', '--yes', action='store_true', help='não pedir confirmação')
    p.set_defaults(fn=cmd_play)

    sub.add_parser('list', help='listar pages').set_defaults(fn=cmd_list)
    s = sub.add_parser('show', help='mostrar valores de uma page')
    s.add_argument('name')
    s.set_defaults(fn=cmd_show)

    args = ap.parse_args()
    if getattr(args, 'speed', 1.0) <= 0:
        ap.error('--speed deve ser > 0')
    args.fn(args)


if __name__ == '__main__':
    main()
