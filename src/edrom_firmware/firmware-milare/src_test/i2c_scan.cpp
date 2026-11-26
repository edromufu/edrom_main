// Diagnóstico do I2C das IMUs (sem micro-ROS, sem botões).
// Use: pio run -e i2cscan -t upload && pio device monitor -e i2cscan
//
// A cada 1 s, em 100 kHz e em 400 kHz:
//   - varre os endereços 0x08..0x77 e lista quem respondeu
//   - para 0x68 e 0x69: lê WHO_AM_I, acorda o MPU e faz 50 leituras de 14 bytes,
//     contando falhas e mostrando a última amostra (ax ay az gx gy gz)
// Esperado: "0x68 0x69" no scan. Só 0x68 com 2 MPUs ligados = AD0 da IMU2 não
// está em 3V3 (os dois brigam no 0x68). Nada no scan = SDA/SCL/VCC/GND.
#include <Arduino.h>
#include <Wire.h>

static int16_t be16(const uint8_t* p) { return (int16_t)((p[0] << 8) | p[1]); }

static bool rd(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n, (uint8_t)true) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool wr(uint8_t addr, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission(true) == 0;
}

static void testMpu(uint8_t addr) {
  uint8_t who = 0;
  if (!rd(addr, 0x75, &who, 1)) {
    Serial.printf("  0x%02X: sem resposta\n", addr);
    return;
  }
  bool woke = wr(addr, 0x6B, 0x01) && wr(addr, 0x1C, 0x00) && wr(addr, 0x1B, 0x08);
  delay(10);
  int fails = 0;
  uint8_t b[14] = {0};
  for (int i = 0; i < 50; i++) {
    if (!rd(addr, 0x3B, b, 14)) fails++;
    delay(2);
  }
  Serial.printf("  0x%02X: WHO_AM_I=0x%02X config=%s falhas=%d/50  "
                "a=[%d %d %d] g=[%d %d %d]\n",
                addr, who, woke ? "ok" : "ERRO", fails,
                be16(b), be16(b + 2), be16(b + 4), be16(b + 8), be16(b + 10), be16(b + 12));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA 21, SCL 22
  Wire.setTimeOut(5);
  Serial.println("\n=== Scanner I2C das IMUs ===");
}

void loop() {
  static const uint32_t CLOCKS[2] = {100000, 400000};
  for (uint32_t clk : CLOCKS) {
    Wire.setClock(clk);
    Serial.printf("[%lu kHz] scan:", (unsigned long)(clk / 1000));
    int found = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission(true) == 0) {
        Serial.printf(" 0x%02X", a);
        found++;
      }
    }
    Serial.println(found ? "" : " nenhum dispositivo");
    testMpu(0x68);
    testMpu(0x69);
  }
  Serial.println();
  delay(1000);
}
