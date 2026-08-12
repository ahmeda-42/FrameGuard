/*
  Bike Motion Trigger — ESP32-S3-WROOM-1 + BNO055 (no external libraries)
  --------------------------------------------------------------------
  Wiring:
    BNO055 SDA -> IO1
    BNO055 SCL -> IO2
    Output pin -> IO21 (HIGH for 10s when sustained motion detected)

  Talks to the BNO055 directly over I2C (register-level), reading its
  Linear Acceleration output (gravity already removed by the chip's
  onboard sensor fusion, once in NDOF mode).
*/

#include <Wire.h>

// ---------- Pin config ----------
#define SDA_PIN     1
#define SCL_PIN     2
#define OUTPUT_PIN  21

// ---------- BNO055 I2C config ----------
#define BNO055_ADDR         0x28   // default addr (COM3 pin low). Use 0x29 if pulled high.
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_LINEAR_ACCEL_DATA_ADDR 0x28

#define OPR_MODE_CONFIG 0x00
#define OPR_MODE_NDOF   0x0C
#define PWR_MODE_NORMAL 0x00

// ---------- Motion detection tuning ----------
const float ACCEL_THRESHOLD = 1.2;              // m/s^2, tune on real rides
const unsigned long SAMPLE_INTERVAL_MS = 100;   // 10 Hz sampling
const unsigned long WINDOW_MS          = 2000;  // 2s rolling window
const int WINDOW_SIZE = WINDOW_MS / SAMPLE_INTERVAL_MS;
const float REQUIRED_ACTIVE_RATIO = 0.70;       // 70% of window must be "active"
const unsigned long OUTPUT_HOLD_MS = 10000;     // 10s HIGH pulse

// ---------- State ----------
bool activeBuffer[64];
int bufIndex = 0;
int activeCount = 0;
bool bufferFilled = false;

unsigned long lastSampleTime = 0;
bool outputActive = false;
unsigned long outputStartTime = 0;

// ---------- Low-level I2C helpers ----------
void bno_write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t bno_read8(uint8_t reg) {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BNO055_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

void bno_readLen(uint8_t reg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BNO055_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}

bool bno_init() {
  delay(650); // power-on boot time

  uint8_t chipId = bno_read8(BNO055_CHIP_ID_ADDR);
  if (chipId != 0xA0) {
    Serial.print("BNO055 not found, chip ID = 0x");
    Serial.println(chipId, HEX);
    return false;
  }

  bno_write8(BNO055_OPR_MODE_ADDR, OPR_MODE_CONFIG);
  delay(25);

  bno_write8(BNO055_PAGE_ID_ADDR, 0);
  bno_write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
  bno_write8(BNO055_PWR_MODE_ADDR, PWR_MODE_NORMAL);
  delay(10);

  bno_write8(BNO055_OPR_MODE_ADDR, OPR_MODE_NDOF);
  delay(20);

  return true;
}

// Reads linear acceleration magnitude in m/s^2 (gravity already removed by fusion)
float bno_readLinearAccelMagnitude() {
  uint8_t buf[6];
  bno_readLen(BNO055_LINEAR_ACCEL_DATA_ADDR, buf, 6);

  int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

  // BNO055 linear accel scale: 1 m/s^2 = 100 LSB
  float fx = x / 100.0;
  float fy = y / 100.0;
  float fz = z / 100.0;

  return sqrt(fx * fx + fy * fy + fz * fz);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bno_init()) {
    Serial.println("BNO055 init failed — check wiring/address.");
    while (1) delay(1000);
  }

  memset(activeBuffer, 0, sizeof(activeBuffer));
  Serial.println("Bike motion trigger ready.");
}

void loop() {
  unsigned long now = millis();

  // ---- Handle output HIGH hold / release (non-blocking) ----
  if (outputActive) {
    if (now - outputStartTime >= OUTPUT_HOLD_MS) {
      digitalWrite(OUTPUT_PIN, LOW);
      outputActive = false;
      Serial.println("IO21 -> LOW (10s elapsed)");
      memset(activeBuffer, 0, sizeof(activeBuffer));
      activeCount = 0;
      bufIndex = 0;
      bufferFilled = false;
    }
    return; // skip motion evaluation while pulse is active
  }

  // ---- Sample IMU at fixed interval ----
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  float mag = bno_readLinearAccelMagnitude();
  bool isActive = (mag >= ACCEL_THRESHOLD);

  // ---- Update rolling window (circular buffer) ----
  if (bufferFilled) {
    activeCount -= activeBuffer[bufIndex] ? 1 : 0;
  }
  activeBuffer[bufIndex] = isActive;
  activeCount += isActive ? 1 : 0;

  bufIndex = (bufIndex + 1) % WINDOW_SIZE;
  if (bufIndex == 0) bufferFilled = true;

  // ---- Evaluate sustained motion ----
  int samplesToConsider = bufferFilled ? WINDOW_SIZE : bufIndex;
  if (samplesToConsider > 0) {
    float activeRatio = (float)activeCount / samplesToConsider;

    if (bufferFilled && activeRatio >= REQUIRED_ACTIVE_RATIO) {
      digitalWrite(OUTPUT_PIN, HIGH);
      outputActive = true;
      outputStartTime = now;
      Serial.print("Sustained motion detected (ratio=");
      Serial.print(activeRatio);
      Serial.println(") -> IO21 HIGH for 10s");
    }
  }

  // Optional debug:
  // Serial.print("mag="); Serial.println(mag);
}