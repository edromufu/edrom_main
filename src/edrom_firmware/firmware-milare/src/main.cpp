#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int8.h>

// Botões entre o pino e GND (usa pull-up interno). Ajuste aos seus pinos.
// Botões 1-3: D25, D26, D27. Botão 4: RX2 (GPIO16). Botão 5: TX2 (GPIO17).
static const uint8_t BTN_PINS[5] = {25, 26, 27, 16, 17};
static const uint32_t DEBOUNCE_MS = 40;
static const uint8_t LED_PIN = 2;  // LED onboard: pisca se houver erro

rcl_publisher_t pub;
std_msgs__msg__Int8 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

bool lastState[5];
uint32_t lastChange[5];

// Erro de inicialização: pisca o LED para sempre
static void errorLoop() {
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

#define CHECK(fn) \
  if ((fn) != RCL_RET_OK) errorLoop()

void setup() {
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    pinMode(BTN_PINS[i], INPUT_PULLUP);
    lastState[i] = HIGH;
    lastChange[i] = 0;
  }

  // O Serial fica ocupado pelo micro-ROS: não use Serial.print.
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  CHECK(rclc_support_init(&support, 0, NULL, &allocator));
  CHECK(rclc_node_init_default(&node, "button_node", "", &support));
  CHECK(rclc_publisher_init_default(
      &pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      "button_event"));
}

void loop() {
  uint32_t now = millis();
  for (int i = 0; i < 5; i++) {
    bool s = digitalRead(BTN_PINS[i]);
    if (s != lastState[i] && now - lastChange[i] > DEBOUNCE_MS) {
      lastChange[i] = now;
      lastState[i] = s;
      if (s == LOW) {  // borda de descida = pressionado
        msg.data = i + 1;
        rcl_publish(&pub, &msg, NULL);
      }
    }
  }
}
