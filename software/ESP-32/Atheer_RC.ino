/*
 * ============================================================
 *  ESP32_Atheer_RC — RC Car Controller
 * ============================================================
 *  Hardware:
 *    Servo signal  → GPIO 16
 *    L298N IN1     → GPIO 25
 *    L298N IN2     → GPIO 33
 *    L298N ENA     → GPIO 12  (PWM)
 *    MPU6050 SDA   → GPIO 21
 *    MPU6050 SCL   → GPIO 22
 *
 *  Bluetooth: Classic SPP  →  device name "ESP32_Atheer"
 *
 *  Packet format (commands FROM laptop):
 *    CMD:<steering>,<throttle>\n
 *      steering : -100 to 100  (negative=left, positive=right, 0=center)
 *      throttle : -100 to 100  (negative=reverse, positive=forward, 0=stop)
 *    Example: "CMD:-50,75\n"
 *
 *  Packet format (telemetry TO laptop, every 200 ms):
 *    STAT:<speed_ms>,<yaw_deg>\n
 *      speed_ms  : estimated speed in m/s (float, 2 dp)
 *      yaw_deg   : heading in degrees     (float, 2 dp)
 *    Example: "STAT:1.23,045.67\n"
 *
 *  Required libraries (install via Arduino Library Manager):
 *    - ESP32Servo      by Kevin Harrington
 *    - BluetoothSerial (built-in with ESP32 Arduino core)
 *    - Wire.h          (built-in — no install needed)
 * ============================================================
 */

// ── Libraries ────────────────────────────────────────────────
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include <Wire.h>

// ── MPU6050 I2C address & registers (no library needed) ──────
#define MPU6050_ADDR        0x68   // AD0 pin low (default)
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_CONFIG       0x1A  // DLPF config
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

// Sensitivity scale factors
// Gyro  ±500  °/s  → 65.5 LSB/(°/s)
// Accel ±8g         → 4096 LSB/g
#define GYRO_SCALE   65.5f
#define ACCEL_SCALE  4096.0f
#define G_TO_MS2     9.80665f

// ── Pin Definitions ──────────────────────────────────────────
#define SERVO_PIN     16
#define L298N_IN1     25
#define L298N_IN2     33
#define L298N_ENA     12   // PWM speed control

// ── Servo Configuration ───────────────────────────────────────
#define SERVO_CENTER  90   // degrees — straight ahead
#define SERVO_MIN     61   // degrees — full left
#define SERVO_MAX     117  // degrees — full right
#define SERVO_PULSE_MIN  500   // µs
#define SERVO_PULSE_MAX  2500  // µs

// ── L298N PWM (LEDC) ─────────────────────────────────────────
#define PWM_CHANNEL   0
#define PWM_FREQ      1000   // Hz
#define PWM_RESOLUTION 8     // bits → 0–255

// ── Telemetry ────────────────────────────────────────────────
#define TELEMETRY_INTERVAL_MS  200

// ── Bluetooth ────────────────────────────────────────────────
BluetoothSerial BT;

// ── Servo ────────────────────────────────────────────────────
Servo steeringServo;

// ── IMU state ────────────────────────────────────────────────
float yaw_deg       = 0.0f;   // integrated yaw (heading)
float speed_ms      = 0.0f;   // estimated speed
unsigned long lastImuTime = 0;

// ── Telemetry timer ──────────────────────────────────────────
unsigned long lastTelemetryTime = 0;

// ── Incoming BT buffer ───────────────────────────────────────
String btBuffer = "";

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("[ESP32_Atheer] Booting...");

  // ── Bluetooth ────────────────────────────────────────────
  if (!BT.begin("ESP32_Atheer")) {
    Serial.println("[BT] Failed to initialise Bluetooth!");
  } else {
    Serial.println("[BT] Ready. Waiting for connection...");
  }

  // ── Servo ────────────────────────────────────────────────
  ESP32PWM::allocateTimer(0);   // allocate hardware timer for servo
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  steeringServo.write(SERVO_CENTER);
  Serial.println("[SERVO] Initialised at centre (90°)");

  // ── L298N ────────────────────────────────────────────────
  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  
  ledcAttach(L298N_ENA, PWM_FREQ, PWM_RESOLUTION);
  stopMotor();
  Serial.println("[MOTOR] L298N initialised");

  // ── MPU6050 ──────────────────────────────────────────────
  Wire.begin(21, 22);   // SDA=GPIO21, SCL=GPIO22

  // Wake up MPU6050 (clear sleep bit in PWR_MGMT_1)
  mpu6050_writeReg(MPU6050_PWR_MGMT_1, 0x00);
  delay(100);

  // DLPF bandwidth ~21 Hz (register value 0x04)
  mpu6050_writeReg(MPU6050_CONFIG, 0x04);

  // Gyro full scale ±500 °/s  (FS_SEL=1 → bits [4:3] = 01)
  mpu6050_writeReg(MPU6050_GYRO_CONFIG, 0x08);

  // Accel full scale ±8g       (AFS_SEL=2 → bits [4:3] = 10)
  mpu6050_writeReg(MPU6050_ACCEL_CONFIG, 0x10);

  // Verify device is responding
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[IMU] MPU6050 not found! Check wiring.");
    while (1) delay(100);
  }

  lastImuTime = millis();
  Serial.println("[IMU] MPU6050 initialised (raw I2C)");

  Serial.println("[ESP32_Atheer] Ready!");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  // 1. Read & process incoming Bluetooth commands
  readBluetooth();

  // 2. Update IMU estimates
  updateIMU();

  // 3. Send telemetry every 200 ms
  unsigned long now = millis();
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = now;
    sendTelemetry();
  }
}

// ============================================================
//  BLUETOOTH — read incoming data line by line
// ============================================================
void readBluetooth() {
  while (BT.available()) {
    char c = (char)BT.read();
    if (c == '\n') {
      btBuffer.trim();
      if (btBuffer.length() > 0) {
        decodeCommand(btBuffer);
      }
      btBuffer = "";
    } else {
      btBuffer += c;
    }
  }
}

// ============================================================
//  DECODE — parse "CMD:<steering>,<throttle>"
// ============================================================
void decodeCommand(const String& packet) {
  // Must start with "CMD:"
  if (!packet.startsWith("CMD:")) {
    Serial.println("[DECODE] Unknown packet: " + packet);
    return;
  }

  String payload = packet.substring(4);  // strip "CMD:"

  int commaIdx = payload.indexOf(',');
  if (commaIdx < 0) {
    Serial.println("[DECODE] Malformed packet (no comma): " + packet);
    return;
  }

  int steering = payload.substring(0, commaIdx).toInt();   // -100 to 100
  int throttle = payload.substring(commaIdx + 1).toInt();  // -100 to 100

  // Clamp values
  steering = constrain(steering, -100, 100);
  throttle = constrain(throttle, -100, 100);

  Serial.printf("[DECODE] steering=%d  throttle=%d\n", steering, throttle);

  applySteeringCommand(steering);
  applyThrottleCommand(throttle);
}

// ============================================================
//  STEERING — map -100…100 → SERVO_MIN…SERVO_MAX
// ============================================================
void applySteeringCommand(int steering) {
  // steering=0 → SERVO_CENTER, -100 → SERVO_MIN, +100 → SERVO_MAX
  int angle = map(steering, -100, 100, SERVO_MIN, SERVO_MAX);
  angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  steeringServo.write(angle);
}

// ============================================================
//  THROTTLE — map -100…100 → L298N direction + PWM
// ============================================================
void applyThrottleCommand(int throttle) {
  if (throttle == 0) {
    stopMotor();
    return;
  }

  int pwmValue = map(abs(throttle), 0, 100, 0, 255);
  pwmValue = constrain(pwmValue, 0, 255);

  if (throttle > 0) {
    // Forward
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
  } else {
    // Reverse
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, HIGH);
  }

  ledcWrite(L298N_ENA, pwmValue);
}

// ── Stop helper ──────────────────────────────────────────────
void stopMotor() {
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, LOW);
  ledcWrite(L298N_ENA, 0);
}

// ============================================================
//  MPU6050 — raw I2C helpers
// ============================================================
void mpu6050_writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read 'len' bytes starting at 'reg' into 'buf'
void mpu6050_readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);   // repeated start
  Wire.requestFrom((uint8_t)MPU6050_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

// Combine two bytes into a signed 16-bit integer
int16_t mpu6050_combine(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

// ============================================================
//  IMU — integrate gyro for yaw, integrate accel for speed
// ============================================================
void updateIMU() {
  // Read 14 bytes: accel (6) + temp (2) + gyro (6)
  uint8_t buf[14];
  mpu6050_readRegs(MPU6050_ACCEL_XOUT_H, buf, 14);

  // Raw values
  int16_t rawAx = mpu6050_combine(buf[0],  buf[1]);
  // int16_t rawAy = mpu6050_combine(buf[2],  buf[3]);  // unused
  // int16_t rawAz = mpu6050_combine(buf[4],  buf[5]);  // unused
  // buf[6..7] = temperature — skipped
  // int16_t rawGx = mpu6050_combine(buf[8],  buf[9]);  // unused
  // int16_t rawGy = mpu6050_combine(buf[10], buf[11]); // unused
  int16_t rawGz = mpu6050_combine(buf[12], buf[13]);

  // Convert to physical units
  float accelX = (rawAx / ACCEL_SCALE) * G_TO_MS2;   // m/s²
  float gyroZ  =  rawGz / GYRO_SCALE;                 // °/s

  unsigned long now = millis();
  float dt = (now - lastImuTime) / 1000.0f;            // seconds
  lastImuTime = now;

  // ── Yaw: integrate Z-axis gyro (°/s → °) ────────────────
  yaw_deg += gyroZ * dt;

  // Keep yaw in 0–360 range
  if (yaw_deg <   0) yaw_deg += 360.0f;
  if (yaw_deg > 360) yaw_deg -= 360.0f;

  // ── Speed: integrate forward acceleration (X-axis) ───────
  // Dead-band to suppress sensor noise at rest
  if (fabsf(accelX) < 0.15f) accelX = 0.0f;

  speed_ms += accelX * dt;

  // Clamp to prevent unbounded drift
  speed_ms = constrain(speed_ms, -10.0f, 10.0f);

  // Gentle decay simulates rolling friction / coming to rest
  speed_ms *= 0.98f;
}

// ============================================================
//  ENCODE & SEND telemetry — "STAT:<speed>,<yaw>\n"
// ============================================================
void sendTelemetry() {
  if (!BT.connected()) return;

  char packet[64];
  snprintf(packet, sizeof(packet), "STAT:%.2f,%.2f\n", speed_ms, yaw_deg);
  BT.print(packet);

  Serial.print("[TELEM] Sent: ");
  Serial.print(packet);
}
