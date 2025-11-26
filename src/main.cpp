#include <Wire.h>
#include <Adafruit_BMP280.h>

// ================= KONFIGURASI =================
#define SDA_PIN 21
#define SCL_PIN 22
#define SERIAL_BAUD 115200
#define TELEMETRY_INTERVAL_MS 50   // Interval pengiriman data (20Hz)
#define MPU_ADDR 0x68              // Alamat I2C MPU6050

// Konstanta MPU6050
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;
const float ACCEL_SCALE = 16384.0f; // Skala Akselerometer ±2g
const float GYRO_SCALE  = 131.0f;   // Skala Giroskop ±250 °/s

// Objek Global
Adafruit_BMP280 bmp;

// Variabel Global
float alertThreshold = 40.0;          // batas sudut miring
float altitudeAlertThreshold = 0.30f;  // batas altitude relatif (meter)
unsigned long lastTelemetryTime = 0;
unsigned long lastMicros = 0;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;

// buffer serial
String serialBuf = "";

// ================= BASELINE (GROUND LEVEL) =================
float baseAltitude = 135.4f;
bool altitudeInitialized = true;
// =========================================================

// ================= STRUKTUR KALMAN FILTER =================
struct Kalman1D {
  float q, r, x, p, k;

  Kalman1D() {
    q = 0.001f;
    r = 0.03f;
    x = 0.0f;
    p = 1.0f;
    k = 0.0f;
  }

  void predict(float gyroRate, float dt) {
    x += gyroRate * dt;
    p += q;
  }

  void update(float measAngle) {
    k = p / (p + r);
    x = x + k * (measAngle - x);
    p = (1.0f - k) * p;
  }

  float getAngle() { return x; }
  void setAngle(float angle) { x = angle; }
};

Kalman1D kalmanRoll, kalmanPitch;

// ================= FUNGSI MPU6050 =================
void writeMPURegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readMPUAccelGyro(int16_t &ax, int16_t &ay, int16_t &az,
                      int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  if (Wire.available() >= 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

float calculateRoll(float ay, float az) {
  return atan2(ay, az) * 180.0f / PI;
}

float calculatePitch(float ax, float ay, float az) {
  return atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
}

void calibrateGyro(int samples = 500) {
  long sumGX = 0, sumGY = 0;
  Serial.print("Kalibrasi Gyro...");
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPUAccelGyro(ax, ay, az, gx, gy, gz);
    sumGX += gx;
    sumGY += gy;
    delay(2);
  }
  gyroBiasX = (float)sumGX / samples;
  gyroBiasY = (float)sumGY / samples;
  Serial.println(" Selesai.");
}

// ================= SETUP =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);

  if (!Wire.begin(SDA_PIN, SCL_PIN)) {
    Serial.println("Error: I2C gagal!");
    while (1);
  }

  bool bmpFound = false;
  if (bmp.begin(0x76)) { bmpFound = true; }
  else if (bmp.begin(0x77)) { bmpFound = true; }

  if (!bmpFound) Serial.println("BMP280 tidak ditemukan!");

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 tidak terdeteksi!");
    while (1);
  }

  writeMPURegister(PWR_MGMT_1, 0x00);
  delay(100);
  calibrateGyro();

  int16_t ax, ay, az, gx, gy, gz;
  readMPUAccelGyro(ax, ay, az, gx, gy, gz);
  kalmanRoll.setAngle(calculateRoll(ay, az));
  kalmanPitch.setAngle(calculatePitch(ax, ay, az));

  lastMicros = micros();

  Serial.println("Sistem Aktif.");
  Serial.printf("Threshold awal = %.1f deg\n", alertThreshold);
  Serial.printf("Altitude baseline = %.2f m\n", baseAltitude);
}

// ================= LOOP =================
void loop() {
  // Serial input
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'u') {
      alertThreshold += 5;
      Serial.printf("Threshold -> %.1f\n", alertThreshold);
    } 
    else if (c == 'd') {
      alertThreshold -= 5;
      Serial.printf("Threshold -> %.1f\n", alertThreshold);
    }
    else if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) {
        alertThreshold = serialBuf.toFloat();
        Serial.printf("Threshold set -> %.1f\n", alertThreshold);
        serialBuf = "";
      }
    }
    else if (isdigit(c) || c == '.') {
      serialBuf += c;
    }
  }

  // Delta time
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6;
  lastMicros = now;

  // Sensor read
  int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
  readMPUAccelGyro(ax_r, ay_r, az_r, gx_r, gy_r, gz_r);

  float ax = ax_r / ACCEL_SCALE;
  float ay = ay_r / ACCEL_SCALE;
  float az = az_r / ACCEL_SCALE;
  float gx = (gx_r - gyroBiasX) / GYRO_SCALE;
  float gy = (gy_r - gyroBiasY) / GYRO_SCALE;

  float rollAcc = calculateRoll(ay, az);
  float pitchAcc = calculatePitch(ax, ay, az);

  kalmanRoll.predict(gx, dt);
  kalmanRoll.update(rollAcc);

  kalmanPitch.predict(gy, dt);
  kalmanPitch.update(pitchAcc);

  float roll = kalmanRoll.getAngle();
  float pitch = kalmanPitch.getAngle();

  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();

    float rawAlt = bmp.readAltitude(1013.25);
    float adjAlt = rawAlt - baseAltitude;

    // ===== STATUS ALERT =====
    String status = "NORMAL";

    bool rollAlert = abs(roll) > alertThreshold;
    bool altAlert = adjAlt > altitudeAlertThreshold;

    if (rollAlert && altAlert) status = "!!! ALERT: MIRING + ALTITUDE !!!";
    else if (rollAlert)        status = "!!! ALERT: MIRING !!!";
    else if (altAlert)         status = "!!! ALTITUDE ALERT !!!";

    // Output
    Serial.printf(
      "Time:%5lu | Roll:%6.2f | Pitch:%6.2f | RawAlt:%6.2f | AdjAlt:%6.2f | Th:%5.1f | %s\n",
      millis(), roll, pitch, rawAlt, adjAlt, alertThreshold, status.c_str()
    );
  }
}
