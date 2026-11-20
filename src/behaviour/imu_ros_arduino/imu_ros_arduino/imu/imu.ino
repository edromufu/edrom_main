#include <Wire.h>
#include <math.h>

#define MPU 0x68  // Endereço I2C do MPU-6050

// Escalas padrão do MPU-6050 (±2 g e ±250 °/s)
const float ACCEL_LSB_PER_G   = 16384.0f;
const float GYRO_LSB_PER_DPS  = 131.0f;
const float GRAVITY           = 9.80665f;

const float ALPHA = 0.98f;                 // Constante do filtro complementar
const unsigned long PRINT_PERIOD_US = 20000; // 50 Hz de saída na serial
const int GYRO_CALIB_SAMPLES = 500;

// Saídas em unidades físicas
float AcX, AcY, AcZ;   // m/s^2
float GyX, GyY, GyZ;   // °/s (bias removido)
float rollC = 0.0f;    // graus

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
unsigned long previousTime = 0, printTime = 0;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

// Lê um registrador de 16 bits com sinal.
// Os dois Wire.read() ficam em instruções separadas porque a ordem de
// avaliação de "Wire.read() << 8 | Wire.read()" não é garantida em C++,
// e o cast para int16_t garante o sinal também em placas onde int tem
// 32 bits (ESP32).
int16_t read16() {
  uint8_t high = Wire.read();
  uint8_t low = Wire.read();
  return (int16_t)((high << 8) | low);
}

void readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  ax = read16();
  ay = read16();
  az = read16();
  read16();          // Temperatura (ignorada)
  gx = read16();
  gy = read16();
  gz = read16();
}

void init_MPU() {
  writeRegister(0x6B, 0x00);  // PWR_MGMT_1: acorda o MPU-6050
  writeRegister(0x1A, 0x03);  // CONFIG: DLPF ~44 Hz (reduz vibração dos servos)
  writeRegister(0x1B, 0x00);  // GYRO_CONFIG: ±250 °/s
  writeRegister(0x1C, 0x00);  // ACCEL_CONFIG: ±2 g
}

// Média do giroscópio com o robô PARADO para remover o bias
void calibrateGyro() {
  long sumX = 0, sumY = 0, sumZ = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    readRaw(ax, ay, az, gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(2);
  }
  gyroBiasX = (float)sumX / GYRO_CALIB_SAMPLES;
  gyroBiasY = (float)sumY / GYRO_CALIB_SAMPLES;
  gyroBiasZ = (float)sumZ / GYRO_CALIB_SAMPLES;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  init_MPU();
  delay(100);
  calibrateGyro();

  // Inicializa o roll com o acelerômetro para não começar em zero
  int16_t ax, ay, az, gx, gy, gz;
  readRaw(ax, ay, az, gx, gy, gz);
  rollC = atan2f((float)ay, (float)az) * RAD_TO_DEG;

  previousTime = micros();
  printTime = previousTime;
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) * 1e-6f;  // segundos
  previousTime = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  readRaw(ax, ay, az, gx, gy, gz);

  AcX = ax / ACCEL_LSB_PER_G * GRAVITY;
  AcY = ay / ACCEL_LSB_PER_G * GRAVITY;
  AcZ = az / ACCEL_LSB_PER_G * GRAVITY;

  GyX = (gx - gyroBiasX) / GYRO_LSB_PER_DPS;
  GyY = (gy - gyroBiasY) / GYRO_LSB_PER_DPS;
  GyZ = (gz - gyroBiasZ) / GYRO_LSB_PER_DPS;

  // Filtro complementar do roll
  float rollAccel = atan2f(AcY, AcZ) * RAD_TO_DEG;
  rollC = (1.0f - ALPHA) * rollAccel + ALPHA * (rollC + GyX * dt);

  if (currentTime - printTime >= PRINT_PERIOD_US) {
    // Formato mantido: roll|AcX|AcY|AcZ|GyX|GyY|GyZ
    // roll em graus, aceleração em m/s^2, giroscópio em °/s
    Serial.print(rollC); Serial.print('|');
    Serial.print(AcX);   Serial.print('|');
    Serial.print(AcY);   Serial.print('|');
    Serial.print(AcZ);   Serial.print('|');
    Serial.print(GyX);   Serial.print('|');
    Serial.print(GyY);   Serial.print('|');
    Serial.print(GyZ);   Serial.print('\n');
    printTime = currentTime;
  }
}
