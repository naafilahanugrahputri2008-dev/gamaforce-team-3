#include <Wire.h>
#include <Adafruit_BMP280.h>

// ================= KONFIGURASI =================
#define SDA_PIN 21
#define SCL_PIN 22
#define SERIAL_BAUD 115200
#define TELEMETRY_INTERVAL_MS 50   
#define MPU_ADDR 0x68              

// UART Telemetry (TANPA HardwareSerial.h)
#define RADIO_TX_PIN 17        // ESP32 TX2 -> RX Telemetry
#define RADIO_RX_PIN 16        // ESP32 RX2 -> TX Telemetry
#define RADIO_BAUD   57600     

// Konstanta MPU6050
const uint8_t PWR_MGMT_1   = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE  = 131.0f;

// Objek Global
Adafruit_BMP280 bmp;

// Variabel Global
float alertThreshold = 40.0;
unsigned long lastTelemetryTime = 0;
unsigned long lastMicros = 0;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
String serialBuf = "";

// ================= GROUND BASELINE =================
float baseAltitude = 133.6f;
bool altitudeInitialized = true;
// ===================================================

// ================= STRUKTUR KALMAN =================
struct Kalman1D {
  float q=0.001f,r=0.03f,x=0,p=1,k=0;
  void predict(float rate,float dt){ x+=rate*dt; p+=q;}
  void update(float angle){ k=p/(p+r); x+=k*(angle-x); p*=1-k;}
  float getAngle(){return x;}
  void setAngle(float angle){x=angle;}
};

Kalman1D kalmanRoll, kalmanPitch;

// ================= FUNGSI MPU6050 =================
void writeMPURegister(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

void readMPUAccelGyro(int16_t &ax,int16_t &ay,int16_t &az,
                      int16_t &gx,int16_t &gy,int16_t &gz){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,(uint8_t)14);

  ax=(Wire.read()<<8)|Wire.read();
  ay=(Wire.read()<<8)|Wire.read();
  az=(Wire.read()<<8)|Wire.read();
  gx=(Wire.read()<<8)|Wire.read();
  gy=(Wire.read()<<8)|Wire.read();
  gz=(Wire.read()<<8)|Wire.read();
}

float calculateRoll(float ay,float az){
  return atan2(ay,az)*180.0/PI;
}

float calculatePitch(float ax,float ay,float az){
  return atan2(-ax,sqrt(ay*ay+az*az))*180.0/PI;
}

void calibrateGyro(int sample=500){
  long sx=0,sy=0;
  for(int i=0;i<sample;i++){
    int16_t ax,ay,az,gx,gy,gz;
    readMPUAccelGyro(ax,ay,az,gx,gy,gz);
    sx+=gx; sy+=gy;
    delay(2);
  }
  gyroBiasX=sx/(float)sample;
  gyroBiasY=sy/(float)sample;
}

// ================= SETUP =================
void setup(){
  Serial.begin(SERIAL_BAUD);
  while(!Serial);

  Serial1.begin(RADIO_BAUD,SERIAL_8N1,RADIO_RX_PIN,RADIO_TX_PIN);

  Wire.begin(SDA_PIN,SCL_PIN);

  bmp.begin(0x76);

  writeMPURegister(PWR_MGMT_1,0x00);
  delay(100);
  calibrateGyro();

  int16_t ax,ay,az,gx,gy,gz;
  readMPUAccelGyro(ax,ay,az,gx,gy,gz);

  kalmanRoll.setAngle(calculateRoll(ay,az));
  kalmanPitch.setAngle(calculatePitch(ax,ay,az));

  lastMicros=micros();

  Serial.println("Sistem Siap");
  Serial1.println("Telemetry Ready");
}

// ================= LOOP =================
void loop(){

  while(Serial.available()){
    char c=Serial.read();
    if(c=='u'||c=='U'){ alertThreshold+=5; }
    else if(c=='d'||c=='D'){ alertThreshold-=5; }
    else if(c=='c'||c=='C'){
      baseAltitude=bmp.readAltitude(1013.25);
      altitudeInitialized=true;
    }
  }

  unsigned long nowMicros=micros();
  float dt=(nowMicros-lastMicros)/1000000.0f;
  lastMicros=nowMicros;

  int16_t axr,ayr,azr,gxr,gyr,gzr;
  readMPUAccelGyro(axr,ayr,azr,gxr,gyr,gzr);

  float ax=axr/ACCEL_SCALE;
  float ay=ayr/ACCEL_SCALE;
  float az=azr/ACCEL_SCALE;
  float gx=(gxr-gyroBiasX)/GYRO_SCALE;
  float gy=(gyr-gyroBiasY)/GYRO_SCALE;

  float rollAcc=calculateRoll(ay,az);
  float pitchAcc=calculatePitch(ax,ay,az);

  kalmanRoll.predict(gx,dt);
  kalmanRoll.update(rollAcc);

  kalmanPitch.predict(gy,dt);
  kalmanPitch.update(pitchAcc);

  float roll=kalmanRoll.getAngle();
  float pitch=kalmanPitch.getAngle();

  if(millis()-lastTelemetryTime>=TELEMETRY_INTERVAL_MS){
    lastTelemetryTime=millis();

    float rawAlt=bmp.readAltitude(1013.25);
    float adjAlt=rawAlt-baseAltitude;

    char line[200];
    snprintf(line,sizeof(line),
      "Roll:%.2f Pitch:%.2f RawAlt:%.2f AdjAlt:%.2f",
      roll,pitch,rawAlt,adjAlt);

    Serial.println(line);
    Serial1.println(line);
  }
}