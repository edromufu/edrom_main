#!/usr/bin/env bash
# Instala o daemon dos botões como serviço de boot.
# Uso (com o ESP32 conectado na USB):
#   ./install.sh                  # tenta achar o ESP32 sozinho
#   ./install.sh /dev/ttyUSB1     # ou informe a porta
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SCRIPT="$HERE/edrom_buttond.py"
WS="${EDROM_WS:-$HOME/edrom_main}"
USER_NAME="$(id -un)"

# --- 1. Achar o ESP32 (não confundir com o U2D2/FTDI dos motores) ---
DEV="${1:-}"
if [[ -z "$DEV" ]]; then
  for d in /dev/ttyUSB* /dev/ttyACM*; do
    [[ -e "$d" ]] || continue
    vid=$(udevadm info -q property -n "$d" | sed -n 's/^ID_VENDOR_ID=//p')
    # 10c4 = CP210x, 1a86 = CH340/CH9102 (conversores comuns no ESP32 DevKit)
    if [[ "$vid" == "10c4" || "$vid" == "1a86" ]]; then
      DEV="$d"; break
    fi
  done
fi
if [[ -z "$DEV" ]]; then
  echo "ESP32 não encontrado. Conecte-o e rode: $0 /dev/ttyUSBx"
  ls -l /dev/serial/by-id/ 2>/dev/null || true
  exit 1
fi

props=$(udevadm info -q property -n "$DEV")
VID=$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$props")
PID=$(sed -n 's/^ID_MODEL_ID=//p' <<<"$props")
SER=$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"$props")
echo "ESP32 em $DEV  (vid=$VID pid=$PID serial=${SER:-?})"

RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\""
[[ -n "$SER" ]] && RULE+=", ATTRS{serial}==\"$SER\""
# ID_MM_DEVICE_IGNORE: o ModemManager não sonda a porta no boot (ele mexe em
# DTR/RTS e pode deixar o ESP32 preso no modo gravação)
RULE+=", SYMLINK+=\"edrom_buttons\", MODE=\"0666\", ENV{ID_MM_DEVICE_IGNORE}=\"1\""
# U2D2 (FTDI) dos Dynamixel: o ModemManager também não deve abrir
RULE+=$'\nSUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ENV{ID_MM_DEVICE_IGNORE}="1"'

# --- 2. udev: nome fixo /dev/edrom_buttons ---
echo "$RULE" | sudo tee /etc/udev/rules.d/99-edrom-buttons.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

# --- 3. Permissão para o botão 4 (segurar) desligar o PC ---
echo "$USER_NAME ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff" |
  sudo tee /etc/sudoers.d/edrom-buttons >/dev/null
sudo chmod 440 /etc/sudoers.d/edrom-buttons
sudo visudo -cf /etc/sudoers.d/edrom-buttons

# --- 4. Serviço systemd ---
sed -e "s|@USER@|$USER_NAME|" -e "s|@WS@|$WS|g" -e "s|@SCRIPT@|$SCRIPT|" \
  "$HERE/edrom-buttons.service" |
  sudo tee /etc/systemd/system/edrom-buttons.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now edrom-buttons.service

echo
echo "Pronto. Ver logs:   journalctl -u edrom-buttons -f"
echo "Logs do bringup:    $WS/logs_robot/"
