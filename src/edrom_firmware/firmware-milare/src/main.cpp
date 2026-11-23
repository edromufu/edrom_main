// EDROM - Painel de botões (ESP32)
//
// Protocolo serial em texto simples (115200 baud), sem micro-ROS, para que
// funcione assim que o robô liga, antes de qualquer coisa do ROS estar no ar.
//
// ESP32 -> PC:
//   "HELLO EDROM_BTN <p1p2p3p4p5>"  a cada 1 s (heartbeat + estado bruto dos
//                       pinos: 1 = solto, 0 = em GND), p.ex. "HELLO EDROM_BTN 11111"
//   "BTN <n>"           botão n (1..5) pressionado
//   "HOLD <n>"          botão n segurado por HOLD_MS
//
// PC -> ESP32 (estado do robô, mostrado no LED onboard):
//   "S IDLE"   robô parado, pronto      -> LED pisca lento (1 s)
//   "S START"  bringup iniciando        -> LED pisca médio (250 ms)
//   "S RUN"    bringup rodando          -> LED aceso fixo
//   "S ERR"    bringup morreu sozinho   -> LED pisca rápido (80 ms)
//   sem mensagem do PC por 3 s          -> LED apagado com flash curto a cada 2 s
#include <Arduino.h>

// Botões entre o pino e GND (usa pull-up interno).
// Botões NORMALMENTE FECHADOS (NF/NC): em repouso o pino está em GND (LOW) e
// apertar abre o contato (HIGH). Se trocar por botões NA/NO, mude para false.
static const bool NORMALLY_CLOSED = true;
static const bool PRESSED = NORMALLY_CLOSED ? HIGH : LOW;
// Botões 1-3: D25, D26, D27. Botão 4: TX2 (GPIO17). Botão 5: RX2 (GPIO16).
static const uint8_t BTN_PINS[5] = {25, 26, 27, 17, 16};
static const uint32_t DEBOUNCE_MS = 40;
static const uint32_t HOLD_MS = 3000;
static const uint32_t HELLO_MS = 1000;
static const uint32_t HOST_TIMEOUT_MS = 3000;
// Após ligar, não gera eventos: só aprende o estado dos pinos (evita
// "apertos fantasmas" enquanto a alimentação/USB estabiliza)
static const uint32_t STARTUP_IGNORE_MS = 1000;
static const uint8_t LED_PIN = 2;

enum HostState { NO_HOST, IDLE, STARTING, RUNNING, ERROR_ST };

bool lastState[5];
uint32_t lastChange[5];
bool holdSent[5];

HostState hostState = NO_HOST;
uint32_t lastHostMsg = 0;
uint32_t lastHello = 0;
char rxBuf[32];
uint8_t rxLen = 0;

static void handleLine(const char* line) {
  if (strncmp(line, "S ", 2) != 0) return;
  const char* s = line + 2;
  if (strcmp(s, "IDLE") == 0) hostState = IDLE;
  else if (strcmp(s, "START") == 0) hostState = STARTING;
  else if (strcmp(s, "RUN") == 0) hostState = RUNNING;
  else if (strcmp(s, "ERR") == 0) hostState = ERROR_ST;
  else return;
  lastHostMsg = millis();
}

static void readHost() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen) {
        rxBuf[rxLen] = '\0';
        handleLine(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;  // linha grande demais: descarta
    }
  }
}

static void updateLed(uint32_t now) {
  HostState st = hostState;
  if (now - lastHostMsg > HOST_TIMEOUT_MS) st = NO_HOST;

  bool on;
  switch (st) {
    case IDLE:     on = (now / 1000) % 2; break;
    case STARTING: on = (now / 250) % 2; break;
    case RUNNING:  on = true; break;
    case ERROR_ST: on = (now / 80) % 2; break;
    default:       on = (now % 2000) < 50; break;
  }
  digitalWrite(LED_PIN, on);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    pinMode(BTN_PINS[i], INPUT_PULLUP);
    holdSent[i] = true;  // só permite HOLD depois de um aperto de verdade
  }
  delay(50);  // pull-ups estabilizarem
  for (int i = 0; i < 5; i++) {
    lastState[i] = digitalRead(BTN_PINS[i]);
    lastChange[i] = millis();
  }
  Serial.begin(115200);
}

void loop() {
  uint32_t now = millis();

  for (int i = 0; i < 5; i++) {
    bool s = digitalRead(BTN_PINS[i]);
    if (now < STARTUP_IGNORE_MS) {  // só acompanha, sem eventos
      lastState[i] = s;
      lastChange[i] = now;
      continue;
    }
    if (s != lastState[i] && now - lastChange[i] > DEBOUNCE_MS) {
      lastChange[i] = now;
      lastState[i] = s;
      holdSent[i] = false;
      if (s == PRESSED) Serial.printf("BTN %d\n", i + 1);
    }
    if (lastState[i] == PRESSED && !holdSent[i] && now - lastChange[i] > HOLD_MS) {
      holdSent[i] = true;
      Serial.printf("HOLD %d\n", i + 1);
    }
  }

  if (now - lastHello > HELLO_MS) {
    lastHello = now;
    char mask[6];
    for (int i = 0; i < 5; i++) mask[i] = digitalRead(BTN_PINS[i]) ? '1' : '0';
    mask[5] = '\0';
    Serial.printf("HELLO EDROM_BTN %s\n", mask);
  }

  readHost();
  updateLed(now);
  delay(2);
}
