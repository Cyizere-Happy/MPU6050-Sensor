#include <Wire.h>

// MPU6050 registers
#define MPU_ADDR 0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43

// Sensitivities (default full-scale): ±2g, ±250°/s
const float ACCEL_SENS = 16384.0; // LSB/g
const float GYRO_SENS  = 131.0;   // LSB/(°/s)

float roll = 0.0f;   // X
float pitch = 0.0f;  // Y
float yaw = 0.0f;    // Z

// Gyro bias (calibrated at startup)
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

unsigned long lastMicros = 0;
const float alpha = 0.98f; // Complementary filter factor

int16_t read16(int reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  // Burst read accelerometer and gyroscope (6 registers each)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 12, true);
  
  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();
  int16_t rawGX = (Wire.read() << 8) | Wire.read();
  int16_t rawGY = (Wire.read() << 8) | Wire.read();
  int16_t rawGZ = (Wire.read() << 8) | Wire.read();

  ax = (float)rawAX / ACCEL_SENS; // in g
  ay = (float)rawAY / ACCEL_SENS;
  az = (float)rawAZ / ACCEL_SENS;

  gx = (float)rawGX / GYRO_SENS;  // in °/s
  gy = (float)rawGY / GYRO_SENS;
  gz = (float)rawGZ / GYRO_SENS;

  gx -= gx_bias;
  gy -= gy_bias;
  gz -= gz_bias;
}

void calibrateGyro(int samples = 500) {
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    int16_t rawGX = read16(REG_GYRO_XOUT_H);
    int16_t rawGY = read16(REG_GYRO_XOUT_H + 2);
    int16_t rawGZ = read16(REG_GYRO_XOUT_H + 4);
    sumX += rawGX;
    sumY += rawGY;
    sumZ += rawGZ;
    delay(2);
  }
  gx_bias = (sumX / (float)samples) / GYRO_SENS;
  gy_bias = (sumY / (float)samples) / GYRO_SENS;
  gz_bias = (sumZ / (float)samples) / GYRO_SENS;
}

void setup() {
  Serial.begin(230400); // Faster baud rate
  Wire.begin();
  Wire.setClock(400000); // Fast I2C mode (400 kHz)

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);

  // Calibrate gyro
  calibrateGyro(600);

  // Seed initial angles from accelerometer
  float ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);
  roll  = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  yaw   = 0.0f; // Yaw starts at 0 (relative)

  lastMicros = micros();
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6; // seconds
  lastMicros = now;

  // Accel-only angles
  float acc_roll  = atan2(ay, az) * 180.0 / PI;
  float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Integrate gyro for roll and pitch
  float roll_gyro  = roll  + gx * dt;
  float pitch_gyro = pitch + gy * dt;

  // Complementary filter for roll and pitch
  roll  = alpha * roll_gyro  + (1.0f - alpha) * acc_roll;
  pitch = alpha * pitch_gyro + (1.0f - alpha) * acc_pitch;

  // For yaw, pure integration (no correction available without magnetometer)
  yaw += gz * dt;

  // Stream CSV: pitch,roll,yaw (degrees)
  Serial.print(pitch, 3);
  Serial.print(',');
  Serial.print(roll, 3);
  Serial.print(',');
  Serial.println(yaw, 3);

  delay(5); // ~200 Hz loop
}