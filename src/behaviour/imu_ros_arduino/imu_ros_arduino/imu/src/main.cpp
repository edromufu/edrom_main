#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "ekf.h"
#include "mahony.h"

#define MPU 0x68  // Endereço I2C do MPU-6050

// Qual filtro alimenta o campo "roll" da saída para o ROS (imu_read.py):
//   0 = complementar (padrão)   1 = EKF   2 = Mahony
#ifndef IMU_FILTER
#define IMU_FILTER 0
#endif

// Com IMU_OUTPUT_COMPARE definido, a serial imprime um CSV comparando
// Mahony e EKF sobre as MESMAS amostras (para testes; NÃO é lido pelo
// imu_read.py). Colunas:
//   t_ms                         tempo desde o boot
//   roll_mahony, pitch_mahony    graus
//   roll_ekf, pitch_ekf          graus
//   roll_std_ekf, pitch_std_ekf  incerteza do EKF (graus)
//   bias_gx_mahony, bias_gy_mahony, bias_gx_ekf, bias_gy_ekf   °/s
//   accel_norm_g                 |a| em g (1.0 = parado)
//   accel_used_mahony, accel_used_ekf   1 = acelerômetro usado, 0 = descartado
//   us_mahony, us_ekf            tempo médio de processamento por amostra (µs)
//   loop_hz                      taxa média do loop
#define CSV_HEADER "t_ms,roll_mahony,pitch_mahony,roll_ekf,pitch_ekf,roll_std_ekf," \
                   "pitch_std_ekf,bias_gx_mahony,bias_gy_mahony,bias_gx_ekf,bias_gy_ekf," \
                   "accel_norm_g,accel_used_mahony,accel_used_ekf,us_mahony,us_ekf,loop_hz"

// Escalas do MPU-6050 (±2 g e ±500 °/s: numa queda o giro passa de 250 °/s)
const float ACCEL_LSB_PER_G   = 16384.0f;
const float GYRO_LSB_PER_DPS  = 65.5f;
const float GRAVITY           = 9.80665f;

const float ALPHA = 0.98f;                   // Constante do filtro complementar
const unsigned long PRINT_PERIOD_US = 20000; // 50 Hz de saída na serial
const int GYRO_CALIB_SAMPLES = 500;

// Saídas em unidades físicas
float AcX, AcY, AcZ;   // m/s^2
float GyX, GyY, GyZ;   // °/s (bias removido)
float rollC = 0.0f;    // graus (complementar)
float pitchC = 0.0f;   // graus (complementar)

AttitudeEKF ekf;
MahonyFilter mahony;
bool ekfAccelUsed = false;
bool mahonyAccelUsed = false;

// Medição do custo de cada filtro dentro de uma janela de impressão
unsigned long usEkfSum = 0, usMahonySum = 0, loopCount = 0;

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
  Wire.requestFrom((uint8_t)MPU, (uint8_t)14, (uint8_t)true);

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
  writeRegister(0x1B, 0x08);  // GYRO_CONFIG: ±500 °/s
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
  Wire.setClock(400000);  // I2C rápido: leitura dos 14 bytes em ~0,4 ms
  init_MPU();
  delay(100);
  calibrateGyro();

  // Inicializa os filtros com o acelerômetro para não começarem em zero
  int16_t ax, ay, az, gx, gy, gz;
  readRaw(ax, ay, az, gx, gy, gz);
  rollC = atan2f((float)ay, (float)az) * RAD_TO_DEG;
  pitchC = atan2f(-(float)ax, sqrtf((float)ay * ay + (float)az * az)) * RAD_TO_DEG;
  ekf.begin(ax, ay, az);
  mahony.begin(ax, ay, az);

#ifdef IMU_OUTPUT_COMPARE
  Serial.println(F(CSV_HEADER));
#endif

  previousTime = micros();
  printTime = previousTime;
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) * 1e-6f;  // segundos
  previousTime = currentTime;
  if (dt > 0.05f) dt = 0.05f;  // proteção contra travadas longas do loop

  int16_t ax, ay, az, gx, gy, gz;
  readRaw(ax, ay, az, gx, gy, gz);

  AcX = ax / ACCEL_LSB_PER_G * GRAVITY;
  AcY = ay / ACCEL_LSB_PER_G * GRAVITY;
  AcZ = az / ACCEL_LSB_PER_G * GRAVITY;

  GyX = (gx - gyroBiasX) / GYRO_LSB_PER_DPS;
  GyY = (gy - gyroBiasY) / GYRO_LSB_PER_DPS;
  GyZ = (gz - gyroBiasZ) / GYRO_LSB_PER_DPS;

  // Filtro complementar (roll e pitch)
  float rollAccel = atan2f(AcY, AcZ) * RAD_TO_DEG;
  float pitchAccel = atan2f(-AcX, sqrtf(AcY * AcY + AcZ * AcZ)) * RAD_TO_DEG;
  rollC = (1.0f - ALPHA) * rollAccel + ALPHA * (rollC + GyX * dt);
  pitchC = (1.0f - ALPHA) * pitchAccel + ALPHA * (pitchC + GyY * dt);

  // Giroscópio em rad/s e aceleração em g, iguais para os dois filtros
  const float gxr = GyX * DEG_TO_RAD, gyr = GyY * DEG_TO_RAD, gzr = GyZ * DEG_TO_RAD;
  const float axg = AcX / GRAVITY, ayg = AcY / GRAVITY, azg = AcZ / GRAVITY;

  unsigned long t0 = micros();
  ekf.predict(gxr, gyr, gzr, dt);
  ekfAccelUsed = ekf.update(axg, ayg, azg);
  unsigned long t1 = micros();
  mahonyAccelUsed = mahony.update(gxr, gyr, gzr, axg, ayg, azg, dt);
  unsigned long t2 = micros();

  usEkfSum += t1 - t0;
  usMahonySum += t2 - t1;
  loopCount++;

  if (currentTime - printTime >= PRINT_PERIOD_US) {
#ifdef IMU_OUTPUT_COMPARE
    const float accelNorm = sqrtf(axg * axg + ayg * ayg + azg * azg);
    const float windowS = (currentTime - printTime) * 1e-6f;
    Serial.print(millis());                          Serial.print(',');
    Serial.print(mahony.roll() * RAD_TO_DEG, 3);     Serial.print(',');
    Serial.print(mahony.pitch() * RAD_TO_DEG, 3);    Serial.print(',');
    Serial.print(ekf.roll() * RAD_TO_DEG, 3);        Serial.print(',');
    Serial.print(ekf.pitch() * RAD_TO_DEG, 3);       Serial.print(',');
    Serial.print(ekf.rollStd() * RAD_TO_DEG, 3);     Serial.print(',');
    Serial.print(ekf.pitchStd() * RAD_TO_DEG, 3);    Serial.print(',');
    Serial.print(mahony.biasX() * RAD_TO_DEG, 4);    Serial.print(',');
    Serial.print(mahony.biasY() * RAD_TO_DEG, 4);    Serial.print(',');
    Serial.print(ekf.biasX() * RAD_TO_DEG, 4);       Serial.print(',');
    Serial.print(ekf.biasY() * RAD_TO_DEG, 4);       Serial.print(',');
    Serial.print(accelNorm, 3);                      Serial.print(',');
    Serial.print(mahonyAccelUsed ? 1 : 0);           Serial.print(',');
    Serial.print(ekfAccelUsed ? 1 : 0);              Serial.print(',');
    Serial.print(usMahonySum / loopCount);           Serial.print(',');
    Serial.print(usEkfSum / loopCount);              Serial.print(',');
    Serial.print(loopCount / windowS, 1);            Serial.print('\n');
#else
#if IMU_FILTER == 1
    float rollOut = ekf.roll() * RAD_TO_DEG;
#elif IMU_FILTER == 2
    float rollOut = mahony.roll() * RAD_TO_DEG;
#else
    float rollOut = rollC;
#endif
    // Formato mantido: roll|AcX|AcY|AcZ|GyX|GyY|GyZ
    // roll em graus, aceleração em m/s^2, giroscópio em °/s
    Serial.print(rollOut); Serial.print('|');
    Serial.print(AcX);     Serial.print('|');
    Serial.print(AcY);     Serial.print('|');
    Serial.print(AcZ);     Serial.print('|');
    Serial.print(GyX);     Serial.print('|');
    Serial.print(GyY);     Serial.print('|');
    Serial.print(GyZ);     Serial.print('\n');
#endif
    printTime = currentTime;
    usEkfSum = usMahonySum = loopCount = 0;
  }
}
