/*
MKS DUAL FOC closed-loop speed control routine.
Test library: SimpleFOC 2.1.1
Test hardware: MKS DUAL FOC V3.1
Input in the serial window: T+speed, you can make the two motors rotate in closed loop.
For example, to make both motors rotate at a speed of 10rad/s, input: T10
When using your own motor, modify BLDCMotor(7) to your motor pole-pair count.
The default supply voltage is 16.8V; adjust voltage_power_supply and voltage_limit for your setup.
*/

#include <Arduino.h>
#include <SimpleFOC.h>

MagneticSensorI2C sensorLeft = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensorRight = MagneticSensorI2C(AS5600_I2C);
TwoWire i2cLeft = TwoWire(0);
TwoWire i2cRight = TwoWire(1);

BLDCMotor motorLeft = BLDCMotor(7);
BLDCDriver3PWM driverLeft = BLDCDriver3PWM(23, 18, 5, 17);

BLDCMotor motorRight = BLDCMotor(7);
BLDCDriver3PWM driverRight = BLDCDriver3PWM(25, 26, 27, 14);

// Enable or disable motors for testing
const bool kEnableMotorLeft = true;
const bool kEnableMotorRight = true;

float target_torque = 3; 

const float kMaxTargetTorque = 120.0f;
const unsigned long kPhaseDurationMs = 6000UL;
const float kTargetSlewRate = 40.0f;  // Max target change per second.
const float kZeroDeadband = 0.12f;    // Ignore tiny commands that cause chatter.
const unsigned long kLogIntervalMs = 100UL;

float gMotorLeftTargetSmoothed = 0.0f;
float gMotorRightTargetSmoothed = 0.0f;
unsigned long gLastApplyMs = 0;
unsigned long gLastLogMs = 0;

enum class TestPhase : uint8_t {
  MotorLeftForward,
  MotorLeftBackward,
  MotorRightForward,
  MotorRightBackward,
};

float rampFactor(unsigned long phaseElapsedMs) {
  const float halfPhase = static_cast<float>(kPhaseDurationMs) * 0.5f;
  const float t = static_cast<float>(phaseElapsedMs);

  if (t <= halfPhase) {
    return t / halfPhase;
  }

  return (static_cast<float>(kPhaseDurationMs) - t) / halfPhase;
}

TestPhase activePhase(unsigned long nowMs) {
  const unsigned long cycleMs = kPhaseDurationMs * 4UL;
  const unsigned long phaseIndex = (nowMs % cycleMs) / kPhaseDurationMs;
  TestPhase phase = static_cast<TestPhase>(phaseIndex);
  if(phase == TestPhase::MotorLeftForward && kEnableMotorLeft == false) phase = TestPhase::MotorRightForward;
  if(phase == TestPhase::MotorLeftBackward && kEnableMotorLeft == false) phase = TestPhase::MotorRightBackward;
  if(phase == TestPhase::MotorRightForward && kEnableMotorRight == false) phase = TestPhase::MotorLeftForward;
  if(phase == TestPhase::MotorRightBackward && kEnableMotorRight == false) phase = TestPhase::MotorLeftBackward;
  return phase;
}

float slewToward(float current, float target, float maxDelta) {
  const float delta = target - current;
  if (delta > maxDelta) {
    return current + maxDelta;
  }
  if (delta < -maxDelta) {
    return current - maxDelta;
  }
  return target;
}

void applyPhaseTargets(unsigned long nowMs) {
  const TestPhase phase = activePhase(nowMs);
  const unsigned long elapsed = nowMs % kPhaseDurationMs;
  const float speed = target_torque * rampFactor(elapsed);

  const unsigned long dtMs = (gLastApplyMs == 0) ? 0 : (nowMs - gLastApplyMs);
  gLastApplyMs = nowMs;
  const float maxDelta = kTargetSlewRate * (static_cast<float>(dtMs) / 1000.0f);

  float motorLeftTargetDesired = 0.0f;
  float motorRightTargetDesired = 0.0f;

  switch (phase) {
    case TestPhase::MotorLeftForward:
      motorLeftTargetDesired = speed;
      break;
    case TestPhase::MotorLeftBackward:
      motorLeftTargetDesired = -speed;
      break;
    case TestPhase::MotorRightForward:
      motorRightTargetDesired = speed;
      break;
    case TestPhase::MotorRightBackward:
      motorRightTargetDesired = -speed;
      break;
  }

  if (fabsf(motorLeftTargetDesired) < kZeroDeadband) {
    motorLeftTargetDesired = 0.0f;
  }
  if (fabsf(motorRightTargetDesired) < kZeroDeadband) {
    motorRightTargetDesired = 0.0f;
  }

  gMotorLeftTargetSmoothed = slewToward(gMotorLeftTargetSmoothed, motorLeftTargetDesired, maxDelta);
  gMotorRightTargetSmoothed = slewToward(gMotorRightTargetSmoothed, motorRightTargetDesired, maxDelta);

  if (kEnableMotorLeft) {
    motorLeft.move(gMotorLeftTargetSmoothed);
  }

  if (kEnableMotorRight) {
    motorRight.move(gMotorRightTargetSmoothed);
  }

  if (nowMs - gLastLogMs >= kLogIntervalMs) {
    gLastLogMs = nowMs;
    Serial.print("Left: ");
    Serial.print(gMotorLeftTargetSmoothed, 3);
    Serial.print(" Right: ");
    Serial.println(gMotorRightTargetSmoothed, 3);
  }
}

void handleSerialCommand() {
#ifdef ARDUINO
  if (!Serial.available()) {
    return;
  }

  const String cmd = Serial.readStringUntil('\n');
  if (cmd.length() < 2) {
    return;
  }

  if (cmd.charAt(0) == 'T') {
    target_torque = cmd.substring(1).toFloat();
    if (target_torque > kMaxTargetTorque) {
      target_torque = kMaxTargetTorque;
    }
    if (target_torque < -kMaxTargetTorque) {
      target_torque = -kMaxTargetTorque;
    }
    Serial.print("Target torque set to: ");
    Serial.println(target_torque);
  }
#endif
}

void setup() {
  i2cLeft.begin(22, 19, 400000);
  i2cRight.begin(32, 33, 400000);
  sensorLeft.init(&i2cLeft);
  sensorRight.init(&i2cRight);

  motorLeft.linkSensor(&sensorLeft);
  motorRight.linkSensor(&sensorRight);

  driverLeft.voltage_power_supply = 16.8;
  driverLeft.init();

  driverRight.voltage_power_supply = 16.8;
  driverRight.init();

  motorLeft.linkDriver(&driverLeft);
  motorRight.linkDriver(&driverRight);

  motorLeft.controller = MotionControlType::torque;
  motorRight.controller = MotionControlType::torque;

  motorLeft.voltage_limit = 8;                                 
  motorRight.voltage_limit = 8;

  Serial.begin(115200);
  motorLeft.useMonitoring(Serial);
  motorRight.useMonitoring(Serial);

  motorLeft.init();
  motorRight.init();

  motorLeft.initFOC();
  motorRight.initFOC();

  Serial.println("Motor ready.");
  Serial.println("Set target torque in serial terminal, e.g. T10");
}

void loop() {
  if (kEnableMotorLeft) {
    motorLeft.loopFOC();
  }

  if (kEnableMotorRight) {
    motorRight.loopFOC();
  }

  applyPhaseTargets(millis());

  handleSerialCommand();
}