#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>

// =========================
// User configuration section
// =========================

// Motor mechanical configuration
static constexpr int MOTOR_POLE_PAIRS = 7;

// Power stage configuration
static constexpr float DRIVER_SUPPLY_VOLTAGE = 12.0f;
static constexpr float DRIVER_VOLTAGE_LIMIT = 6.0f;

// Balance loop safety
static constexpr float MAX_TILT_DEG = 25.0f;
static constexpr float CONTROL_DT_S = 0.004f; // 250 Hz

// RC receiver PWM input configuration
static constexpr int RC_THROTTLE_PIN = 35;
static constexpr int RC_STEER_PIN = 39;
static constexpr uint32_t RC_MIN_US = 1000;
static constexpr uint32_t RC_MAX_US = 2000;
static constexpr uint32_t RC_CENTER_US = 1500;
static constexpr uint32_t RC_DEADBAND_US = 30;
static constexpr uint32_t RC_TIMEOUT_US = 120000;
static constexpr float RC_MAX_TARGET_ANGLE_DEG = 6.0f;
static constexpr float RC_MAX_STEER_CMD = 1.8f;

// Battery monitor (voltage divider module -> ADC pin)
// Update divider ratio for your voltage sensor module calibration.
static constexpr int BATTERY_ADC_PIN = 34;
static constexpr float ADC_REF_VOLTAGE = 3.3f;
static constexpr float ADC_MAX = 4095.0f;
static constexpr float VOLTAGE_DIVIDER_RATIO = 5.0f; // e.g. 25V -> 5V module then scaled to 3.3V externally

// I2C pins
// Bus 0: MPU6050 + Left AS5600
static constexpr int I2C0_SDA = 21;
static constexpr int I2C0_SCL = 22;
// Bus 1: Right AS5600 only (same 0x36 address as left)
static constexpr int I2C1_SDA = 18;
static constexpr int I2C1_SCL = 19;

// Motor A (left)
static constexpr int M1_U = 25;
static constexpr int M1_V = 26;
static constexpr int M1_W = 27;
static constexpr int M1_EN = 4;

// Motor B (right)
static constexpr int M2_U = 14;
static constexpr int M2_V = 32;
static constexpr int M2_W = 33;
static constexpr int M2_EN = 16;

// =========================
// Objects
// =========================

TwoWire I2CBusRight = TwoWire(1);

MPU6050 mpu(Wire);

MagneticSensorI2C sensorLeft = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensorRight = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motorLeft = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCMotor motorRight = BLDCMotor(MOTOR_POLE_PAIRS);

BLDCDriver3PWM driverLeft = BLDCDriver3PWM(M1_U, M1_V, M1_W, M1_EN);
BLDCDriver3PWM driverRight = BLDCDriver3PWM(M2_U, M2_V, M2_W, M2_EN);

// Balance controller gains (start conservative, tune on stand)
float kp_angle = 22.0f;
float ki_angle = 0.0f;
float kd_angle = 0.7f;

float kp_rate = 0.12f;
float ki_rate = 0.8f;
float kd_rate = 0.0008f;

// Optional steering and setpoint offsets
float targetAngleDeg = 0.0f;
float steeringCmd = 0.0f;

// Internal state
float angleInt = 0.0f;
float prevAngleErr = 0.0f;
float rateInt = 0.0f;
float prevRateErr = 0.0f;

unsigned long lastControlUs = 0;
unsigned long lastPrintMs = 0;
bool balancingEnabled = false;

volatile uint32_t rcThrottleRiseUs = 0;
volatile uint32_t rcSteerRiseUs = 0;
volatile uint32_t rcThrottlePulseUs = RC_CENTER_US;
volatile uint32_t rcSteerPulseUs = RC_CENTER_US;
volatile uint32_t rcThrottleLastUpdateUs = 0;
volatile uint32_t rcSteerLastUpdateUs = 0;

float rcTargetAngleDeg = 0.0f;
float rcSteeringCmd = 0.0f;
bool rcSignalValid = false;

// =========================
// Helpers
// =========================

float readBatteryVoltage() {
  uint16_t raw = analogRead(BATTERY_ADC_PIN);
  float adcV = (raw / ADC_MAX) * ADC_REF_VOLTAGE;
  return adcV * VOLTAGE_DIVIDER_RATIO;
}

float normalizeRcUs(uint32_t pulseUs) {
  if (pulseUs < RC_MIN_US) pulseUs = RC_MIN_US;
  if (pulseUs > RC_MAX_US) pulseUs = RC_MAX_US;

  int32_t centered = static_cast<int32_t>(pulseUs) - static_cast<int32_t>(RC_CENTER_US);
  if (abs(centered) <= static_cast<int32_t>(RC_DEADBAND_US)) {
    return 0.0f;
  }

  float norm = static_cast<float>(centered) / static_cast<float>(RC_MAX_US - RC_CENTER_US);
  return constrain(norm, -1.0f, 1.0f);
}

void IRAM_ATTR rcThrottleIsr() {
  bool level = digitalRead(RC_THROTTLE_PIN);
  uint32_t nowUs = micros();
  if (level) {
    rcThrottleRiseUs = nowUs;
  } else {
    uint32_t width = nowUs - rcThrottleRiseUs;
    if (width >= 800 && width <= 2200) {
      rcThrottlePulseUs = width;
      rcThrottleLastUpdateUs = nowUs;
    }
  }
}

void IRAM_ATTR rcSteerIsr() {
  bool level = digitalRead(RC_STEER_PIN);
  uint32_t nowUs = micros();
  if (level) {
    rcSteerRiseUs = nowUs;
  } else {
    uint32_t width = nowUs - rcSteerRiseUs;
    if (width >= 800 && width <= 2200) {
      rcSteerPulseUs = width;
      rcSteerLastUpdateUs = nowUs;
    }
  }
}

void updateRcControl() {
  uint32_t throttlePulse;
  uint32_t steerPulse;
  uint32_t throttleStamp;
  uint32_t steerStamp;
  noInterrupts();
  throttlePulse = rcThrottlePulseUs;
  steerPulse = rcSteerPulseUs;
  throttleStamp = rcThrottleLastUpdateUs;
  steerStamp = rcSteerLastUpdateUs;
  interrupts();

  uint32_t nowUs = micros();
  bool throttleFresh = (nowUs - throttleStamp) <= RC_TIMEOUT_US;
  bool steerFresh = (nowUs - steerStamp) <= RC_TIMEOUT_US;
  rcSignalValid = throttleFresh && steerFresh;

  if (!rcSignalValid) {
    rcTargetAngleDeg = 0.0f;
    rcSteeringCmd = 0.0f;
    return;
  }

  float throttleNorm = normalizeRcUs(throttlePulse);
  float steerNorm = normalizeRcUs(steerPulse);

  rcTargetAngleDeg = throttleNorm * RC_MAX_TARGET_ANGLE_DEG;
  rcSteeringCmd = steerNorm * RC_MAX_STEER_CMD;
}

void stopMotors() {
  motorLeft.move(0.0f);
  motorRight.move(0.0f);
}

void parseSerialCommands() {
  // Commands:
  //  e            -> enable balancing
  //  D            -> disable balancing
  //  a<number>    -> target angle offset in deg (a1.5)
  //  s<number>    -> steering command (s0.2)
  //  p/i/d<number> -> angle PID P/I/D
  //  v/w/x<number> -> rate PID P/I/D
  if (!Serial.available()) {
    return;
  }

  char cmd = Serial.read();

  switch (cmd) {
    case 'e':
      balancingEnabled = true;
      angleInt = 0.0f;
      rateInt = 0.0f;
      Serial.println("Balancing ENABLED");
      break;
    case 'D':
      balancingEnabled = false;
      stopMotors();
      Serial.println("Balancing DISABLED");
      break;
    default:
      break;
  }

  if (!(cmd == 'a' || cmd == 's' || cmd == 'p' || cmd == 'i' || cmd == 'd' || cmd == 'v' || cmd == 'w' || cmd == 'x')) {
    return;
  }

  // Numeric commands parse value payload after command letter.
  float value = Serial.parseFloat();

  switch (cmd) {
    case 'a':
      targetAngleDeg = value;
      Serial.printf("targetAngleDeg=%.3f\n", targetAngleDeg);
      break;
    case 's':
      steeringCmd = value;
      Serial.printf("steeringCmd=%.3f\n", steeringCmd);
      break;
    case 'p':
      kp_angle = value;
      Serial.printf("kp_angle=%.4f\n", kp_angle);
      break;
    case 'i':
      ki_angle = value;
      Serial.printf("ki_angle=%.4f\n", ki_angle);
      break;
    case 'd':
      kd_angle = value;
      Serial.printf("kd_angle=%.4f\n", kd_angle);
      break;
    case 'v':
      kp_rate = value;
      Serial.printf("kp_rate=%.4f\n", kp_rate);
      break;
    case 'w':
      ki_rate = value;
      Serial.printf("ki_rate=%.4f\n", ki_rate);
      break;
    case 'x':
      kd_rate = value;
      Serial.printf("kd_rate=%.6f\n", kd_rate);
      break;
    default:
      break;
  }
}

void setupSensorsAndMotors() {
  // Initialize both I2C buses.
  Wire.begin(I2C0_SDA, I2C0_SCL, 400000);
  I2CBusRight.begin(I2C1_SDA, I2C1_SCL, 400000);

  // MPU6050 (bus 0)
  byte mpuStatus = mpu.begin();
  if (mpuStatus != 0) {
    Serial.printf("MPU6050 init error: %d\n", mpuStatus);
  } else {
    Serial.println("MPU6050 initialized");
  }
  delay(500);
  mpu.calcOffsets(true, true);
  Serial.println("MPU6050 offsets calibrated");

  // Left and right AS5600 sensors
  sensorLeft.init(&Wire);
  sensorRight.init(&I2CBusRight);

  // Link sensors to motors
  motorLeft.linkSensor(&sensorLeft);
  motorRight.linkSensor(&sensorRight);

  // Driver settings
  driverLeft.voltage_power_supply = DRIVER_SUPPLY_VOLTAGE;
  driverLeft.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driverRight.voltage_power_supply = DRIVER_SUPPLY_VOLTAGE;
  driverRight.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driverLeft.init();
  driverRight.init();

  // Link drivers
  motorLeft.linkDriver(&driverLeft);
  motorRight.linkDriver(&driverRight);

  // Motor control type for balancing torque control
  motorLeft.controller = MotionControlType::torque;
  motorRight.controller = MotionControlType::torque;
  motorLeft.torque_controller = TorqueControlType::voltage;
  motorRight.torque_controller = TorqueControlType::voltage;

  // FOC tuning defaults
  motorLeft.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  motorRight.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  motorLeft.LPF_velocity.Tf = 0.01f;
  motorRight.LPF_velocity.Tf = 0.01f;

  motorLeft.init();
  motorRight.init();
  motorLeft.initFOC();
  motorRight.initFOC();

  Serial.println("Motors + encoders initialized");
}

void runBalanceControl(float dt) {
  mpu.update();

  // Choose the axis that corresponds to robot pitch in your mechanical orientation.
  float pitchDeg = mpu.getAngleX();
  float gyroPitchRateDegS = mpu.getGyroX();

  float commandedAngleDeg = targetAngleDeg + rcTargetAngleDeg;
  float commandedSteering = steeringCmd + rcSteeringCmd;

  float angleErr = commandedAngleDeg - pitchDeg;
  float dAngle = (angleErr - prevAngleErr) / dt;
  angleInt += angleErr * dt;
  angleInt = constrain(angleInt, -15.0f, 15.0f);
  prevAngleErr = angleErr;

  // Outer loop -> desired pitch rate
  float targetRateDegS = kp_angle * angleErr + ki_angle * angleInt + kd_angle * dAngle;

  float rateErr = targetRateDegS - gyroPitchRateDegS;
  float dRate = (rateErr - prevRateErr) / dt;
  rateInt += rateErr * dt;
  rateInt = constrain(rateInt, -40.0f, 40.0f);
  prevRateErr = rateErr;

  // Inner loop -> torque/voltage command
  float baseCmd = kp_rate * rateErr + ki_rate * rateInt + kd_rate * dRate;
  baseCmd = constrain(baseCmd, -DRIVER_VOLTAGE_LIMIT, DRIVER_VOLTAGE_LIMIT);

  float leftCmd = baseCmd - commandedSteering;
  float rightCmd = baseCmd + commandedSteering;

  // Reverse one motor command if physical orientation differs.
  motorLeft.move(leftCmd);
  motorRight.move(-rightCmd);

  if (millis() - lastPrintMs > 100) {
    lastPrintMs = millis();
    float vbatt = readBatteryVoltage();
    Serial.printf(
      "pitch=%.2f rate=%.2f cmd=%.3f batt=%.2f en=%d\n",
      pitchDeg,
      gyroPitchRateDegS,
      baseCmd,
      vbatt,
      balancingEnabled ? 1 : 0
    );
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nBalanceBot boot");

  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(RC_STEER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RC_THROTTLE_PIN), rcThrottleIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_STEER_PIN), rcSteerIsr, CHANGE);

  pinMode(BATTERY_ADC_PIN, INPUT);

  setupSensorsAndMotors();

  lastControlUs = micros();
  Serial.println("Type 'e' to enable balancing, 'D' to disable.");
}

void loop() {
  // Run FOC every cycle for both motors.
  motorLeft.loopFOC();
  motorRight.loopFOC();

  parseSerialCommands();

  unsigned long nowUs = micros();
  float dt = (nowUs - lastControlUs) * 1e-6f;

  if (dt < CONTROL_DT_S) {
    return;
  }
  lastControlUs = nowUs;

  mpu.update();
  updateRcControl();
  float pitchDeg = mpu.getAngleX();

  // Hard safety cut if robot falls too far.
  if (fabsf(pitchDeg) > MAX_TILT_DEG || !balancingEnabled) {
    stopMotors();
    angleInt = 0.0f;
    rateInt = 0.0f;
    prevAngleErr = 0.0f;
    prevRateErr = 0.0f;
    return;
  }

  runBalanceControl(dt);
}
