// EDROM - Painel de botões + IMU MPU-6050 (ESP32)
//
// Protocolo serial em texto simples (SERIAL_BAUD), sem micro-ROS, para que
// funcione assim que o robô liga, antes de qualquer coisa do ROS estar no ar.
//
// ESP32 -> PC:
//   "HELLO EDROM_BTN <p1p2p3p4p5>"  a cada 1 s (heartbeat + estado bruto dos
//                       pinos: 1 = solto, 0 = em GND), p.ex. "HELLO EDROM_BTN 11111"
//   "BTN <n>"           botão n (1..5) pressionado
//   "HOLD <n>"          botão n segurado por HOLD_MS
//   "IMU <seq> <t_us> <ax> <ay> <az> <gx> <gy> <gz>"   a cada IMU_PERIOD_US
//                       amostra BRUTA do MPU-6050 (contagens int16, ±2 g e
//                       ±500 °/s). seq vai de 0 a 65535 (detecta perdas) e
//                       t_us é o micros() da leitura (uint32, dá a volta em
//                       ~71 min). Nenhum filtro roda aqui: calibração do giro,
//                       filtros e comparação ficam no PC (imu_ros_arduino).
//   "IMU2 <seq> <t_us> <ax> <ay> <az> <gx> <gy> <gz>"  idem, segundo MPU-6050
//                       (0x69, AD0 em 3V3), lido no mesmo tick, logo após o "IMU"
//   "IMUSTAT <OK|ERR> <who_am_i> <OK|ERR> <who_am_i2>"   a cada 1 s (MPUs
//                       0x68 e 0x69 respondendo ou não)
//
// PC -> ESP32 (estado do robô, mostrado no LED onboard):
//   "S IDLE"   robô parado, pronto      -> LED pisca lento (1 s)
//   "S START"  bringup iniciando        -> LED pisca médio (250 ms)
//   "S RUN"    bringup rodando          -> LED aceso fixo
//   "S ERR"    bringup morreu sozinho   -> LED pisca rápido (80 ms)
//   sem mensagem do PC por 3 s          -> LED apagado com flash curto a cada 2 s
#include <Arduino.h>
#include <Wire.h>

// 115200 não comporta 200 amostras/s de IMU em texto (~13 kB/s).
// O host (host/edrom_buttond.py, EDROM_BTN_BAUD) tem que usar o mesmo valor.
static const uint32_t SERIAL_BAUD = 460800;

// MPU-6050 no I2C padrão do ESP32: SDA = GPIO21, SCL = GPIO22
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t MPU2_ADDR = 0x69;           // segundo MPU-6050 (AD0 em 3V3)
static const uint32_t IMU_PERIOD_US = 5000;      // 200 Hz
static const uint32_t IMU_RETRY_MS = 1000;       // tenta reiniciar o MPU se ele sumir

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

bool imuOk = false;
uint8_t imuWhoAmI = 0;
uint16_t imuSeq = 0;
uint32_t imuNextUs = 0;
uint32_t imuLastTry = 0;

// IMU2 (0x69): estado próprio, o retry dela não mexe na IMU1
bool imu2Ok = false;
uint8_t imu2WhoAmI = 0;
uint16_t imu2Seq = 0;
uint32_t imu2LastTry = 0;

static bool mpuWrite(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

// Lê n bytes a partir de reg. false se o MPU não respondeu.
static bool mpuRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n, (uint8_t)true) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool imuInit(uint8_t addr, uint8_t* whoAmI) {
  uint8_t who = 0;
  if (!mpuRead(addr, 0x75, &who, 1)) return false;   // WHO_AM_I (0x68; clones: 0x70, 0x72, 0x98)
  *whoAmI = who;
  return mpuWrite(addr, 0x6B, 0x01)      // PWR_MGMT_1: acorda, clock do PLL do giro X
      && mpuWrite(addr, 0x1A, 0x03)      // CONFIG: DLPF ~44 Hz (reduz vibração dos servos)
      && mpuWrite(addr, 0x19, 0x04)      // SMPLRT_DIV: 1 kHz / (1 + 4) = 200 Hz
      && mpuWrite(addr, 0x1B, 0x08)      // GYRO_CONFIG: ±500 °/s (numa queda passa de 250 °/s)
      && mpuWrite(addr, 0x1C, 0x00);     // ACCEL_CONFIG: ±2 g
}

static int16_t be16(const uint8_t* p) { return (int16_t)((p[0] << 8) | p[1]); }

// Lê uma IMU e imprime "<tag> seq t_us ax ay az gx gy gz". Se ela estiver fora,
// tenta reiniciar no máximo a cada IMU_RETRY_MS. Cada IMU tem estado próprio:
// uma fora do ar não impede a leitura nem o retry da outra.
static void pollImu(const char* tag, uint8_t addr, bool& ok, uint8_t& whoAmI,
                    uint16_t& seq, uint32_t& lastTry, uint32_t nowMs) {
  if (!ok) {
    if (nowMs - lastTry < IMU_RETRY_MS) return;
    lastTry = nowMs;
    ok = imuInit(addr, &whoAmI);
    return;
  }
  uint8_t b[14];
  uint32_t t = micros();
  if (!mpuRead(addr, 0x3B, b, 14)) {   // ACCEL_XOUT_H .. GYRO_ZOUT_L (temperatura no meio)
    ok = false;
    lastTry = nowMs;
    return;
  }
  Serial.printf("%s %u %lu %d %d %d %d %d %d\n", tag, seq++, (unsigned long)t,
                be16(b), be16(b + 2), be16(b + 4), be16(b + 8), be16(b + 10), be16(b + 12));
}

// Tick de 200 Hz: lê a IMU1 e, logo em seguida, a IMU2.
static void updateImu(uint32_t nowMs) {
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - imuNextUs) < 0) return;
  // Mantém a grade de 200 Hz; se atrasou muito (ex.: I2C lento), reancora
  imuNextUs += IMU_PERIOD_US;
  if ((int32_t)(nowUs - imuNextUs) > (int32_t)IMU_PERIOD_US) imuNextUs = nowUs + IMU_PERIOD_US;

  pollImu("IMU", MPU_ADDR, imuOk, imuWhoAmI, imuSeq, imuLastTry, nowMs);
  pollImu("IMU2", MPU2_ADDR, imu2Ok, imu2WhoAmI, imu2Seq, imu2LastTry, nowMs);
}

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
  Serial.begin(SERIAL_BAUD);
  Wire.begin();             // SDA 21, SCL 22
  Wire.setClock(400000);    // 14 bytes em ~0,4 ms
  Wire.setTimeOut(5);       // MPU desconectado não pode travar os botões
  imuOk = imuInit(MPU_ADDR, &imuWhoAmI);
  imuLastTry = millis();
  imuNextUs = micros();
  imu2Ok = imuInit(MPU2_ADDR, &imu2WhoAmI);
  imu2LastTry = millis();
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
    Serial.printf("IMUSTAT %s 0x%02X %s 0x%02X\n", imuOk ? "OK" : "ERR", imuWhoAmI,
                  imu2Ok ? "OK" : "ERR", imu2WhoAmI);
  }

  updateImu(now);
  readHost();
  updateLed(now);
  delay(1);
}
