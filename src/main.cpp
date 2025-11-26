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
float alertThreshold = 40.0; // Batas sudut untuk Alert
unsigned long lastTelemetryTime = 0;
unsigned long lastMicros = 0;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;

// buffer serial untuk menerima angka via serial
String serialBuf = "";

// ================= STRUKTUR KALMAN FILTER =================
struct Kalman1D {
  float q; // Kovarians noise proses
  float r; // Kovarians noise pengukuran
  float x; // Estimasi sudut
  float p; // Kovarians error estimasi
  float k; // Kalman gain

  Kalman1D() {
    q = 0.001f; 
    r = 0.03f;  
    x = 0.0f;
    p = 1.0f;
    k = 0.0f;
  }

  // Tahap Prediksi
  void predict(float gyroRate, float dt) {
    x += gyroRate * dt;
    p += q;
  }

  // Tahap Update
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

void readMPUAccelGyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
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
  Serial.print("Kalibrasi Gyro (" + String(samples) + " sampel)...");
  for (int i = 0; i < samples; ++i) {
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
  while(!Serial) delay(10); 
  
  // Inisialisasi I2C
  if(!Wire.begin(SDA_PIN, SCL_PIN)) {
    Serial.println("Error: I2C gagal diinisialisasi.");
    while(1);
  }

  // --- 1. SETUP BMP280 ---
  bool bmpFound = false;
  if (bmp.begin(0x76)) {
    Serial.println("BMP280 OK (0x76)");
    bmpFound = true;
  } else if (bmp.begin(0x77)) {
    Serial.println("BMP280 OK (0x77)");
    bmpFound = true;
  } else {
    Serial.println("PERINGATAN: BMP280 Tidak Terdeteksi!");
  }

  if (bmpFound) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_63);
  }

  // --- 2. SETUP MPU6050 ---
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: MPU6050 Tidak Terdeteksi!");
    while(1);
  }
  
  writeMPURegister(PWR_MGMT_1, 0x00);
  delay(100);
  calibrateGyro();

  // Inisialisasi sudut awal dari akselerometer
  int16_t ax, ay, az, gx, gy, gz;
  readMPUAccelGyro(ax, ay, az, gx, gy, gz);
  float rollInit = calculateRoll(ay, az);
  float pitchInit = calculatePitch(ax, ay, az);
  
  kalmanRoll.setAngle(rollInit);
  kalmanPitch.setAngle(pitchInit);
  
  lastMicros = micros();
  
  Serial.println("Sistem Siap.");
  Serial.printf("Current alertThreshold = %.1f deg\n", alertThreshold);
  Serial.println("Commands: 'u' = +5 deg, 'd' = -5 deg, or send numeric value then Enter to set directly.");
}

// ================= LOOP =================
void loop() {
  // --- Read Serial commands without blocking ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'u' || c == 'U') {
      alertThreshold += 5.0f;
      if (alertThreshold > 180.0f) alertThreshold = 180.0f;
      Serial.printf("Threshold increased -> %.1f deg\n", alertThreshold);
    } else if (c == 'd' || c == 'D') {
      alertThreshold -= 5.0f;
      if (alertThreshold < 0.0f) alertThreshold = 0.0f;
      Serial.printf("Threshold decreased -> %.1f deg\n", alertThreshold);
    } else if (c == '\r' || c == '\n') {
      // end of numeric input — parse if buffer not empty
      if (serialBuf.length() > 0) {
        float v = serialBuf.toFloat();
        if (v >= 0.0f && v <= 180.0f) {
          alertThreshold = v;
          Serial.printf("Threshold set -> %.1f deg\n", alertThreshold);
        } else {
          Serial.println("Nilai tidak valid (0-180).");
        }
        serialBuf = "";
      }
    } else {
      // accumulate numeric characters (digits, dot, minus)
      if ((c >= '0' && c <= '9') || c == '.' || c == '-' ) {
        serialBuf += c;
      } else {
        // ignore other characters (but clear if necessary)
      }
    }
  }

  // --- Hitung Delta Time (dt) ---
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastMicros) / 1000000.0f;
  lastMicros = nowMicros;

  // --- Baca Data Sensor ---
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  readMPUAccelGyro(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  // Konversi ke unit fisik
  float ax = ax_raw / ACCEL_SCALE;
  float ay = ay_raw / ACCEL_SCALE;
  float az = az_raw / ACCEL_SCALE;
  float gx = (gx_raw - gyroBiasX) / GYRO_SCALE;
  float gy = (gy_raw - gyroBiasY) / GYRO_SCALE;

  // Hitung sudut berdasarkan akselerometer
  float rollAcc = calculateRoll(ay, az);
  float pitchAcc = calculatePitch(ax, ay, az);

  // Proses Kalman Filter
  kalmanRoll.predict(gx, dt);
  kalmanRoll.update(rollAcc);
  kalmanPitch.predict(gy, dt);
  kalmanPitch.update(pitchAcc);

  float roll = kalmanRoll.getAngle();
  float pitch = kalmanPitch.getAngle();

  // --- Telemetri Serial ---
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();

    float alt = NAN;
    if (bmp.sensorID()) {
      alt = bmp.readAltitude(1013.25); 
    }

    // Logika Alert
    String status = "NORMAL";
    if (abs(roll) > alertThreshold) {
      status = "!!! ALERT: MIRING !!!";
    }

    // Output Serial Terformat
    Serial.printf("Time: %5lu ms | Roll: %6.2f | Pitch: %6.2f | Alt: %6.1f m | Th: %5.1f | Sts: %s\n", 
                  millis(), roll, pitch, alt, alertThreshold, status.c_str());
  }
}