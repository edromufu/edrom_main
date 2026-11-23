// Teste de placa: imprime no Serial qual botão foi pressionado/solto.
// Sem micro-ROS. Use: pio run -e test -t upload && pio device monitor -e test
#include <Arduino.h>

static const uint8_t BTN_PINS[5] = {25, 26, 27, 17, 16};
static const char* BTN_NAMES[5] = {"D25", "D26", "D27", "TX2", "RX2"};
static const uint32_t DEBOUNCE_MS = 40;

bool lastState[5];
uint32_t lastChange[5];

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 5; i++) {
    pinMode(BTN_PINS[i], INPUT_PULLUP);
    lastState[i] = digitalRead(BTN_PINS[i]);
    lastChange[i] = 0;
  }
  Serial.println("\n=== Teste de botoes ===");
}

void loop() {
  uint32_t now = millis();
  for (int i = 0; i < 5; i++) {
    bool s = digitalRead(BTN_PINS[i]);
    if (s != lastState[i] && now - lastChange[i] > DEBOUNCE_MS) {
      lastChange[i] = now;
      lastState[i] = s;
      Serial.printf("Botao %d (%s, GPIO%d): %s\n", i + 1, BTN_NAMES[i],
                    BTN_PINS[i], s == LOW ? "PRESSIONADO" : "solto");
    }
  }
}
