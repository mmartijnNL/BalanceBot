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

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

float target_velocity = 30;

const float kMaxTargetVelocity = 120.0f;
const float kMinStableVelocity = 6.0f;
const unsigned long kPhaseDurationMs = 6000UL;

enum class TestPhase : uint8_t {
  kMotor0Forward,
  kMotor0Backward,
  kMotor1Forward,
  kMotor1Backward,
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
  return static_cast<TestPhase>(phaseIndex);
}

void applyPhaseTargets(unsigned long nowMs) {
  const TestPhase phase = activePhase(nowMs);
  const unsigned long elapsed = nowMs % kPhaseDurationMs;
  float speed = target_velocity * rampFactor(elapsed);

  if ((speed < kMinStableVelocity) && (speed > -kMinStableVelocity)) {
    speed = 0.0f;
  }

  float motor0Target = 0.0f;
  float motor1Target = 0.0f;

  switch (phase) {
    case TestPhase::kMotor0Forward:
      motor0Target = speed;
      break;
    case TestPhase::kMotor0Backward:
      motor0Target = -speed;
      break;
    case TestPhase::kMotor1Forward:
      motor1Target = speed;
      break;
    case TestPhase::kMotor1Backward:
      motor1Target = -speed;
      break;
  }

  motor.move(motor0Target);
  motor1.move(motor1Target);
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
    target_velocity = cmd.substring(1).toFloat();
    if (target_velocity > kMaxTargetVelocity) {
      target_velocity = kMaxTargetVelocity;
    }
    if (target_velocity < -kMaxTargetVelocity) {
      target_velocity = -kMaxTargetVelocity;
    }
    Serial.print("Target velocity set to: ");
    Serial.println(target_velocity);
  }
#endif
}

void setup() {
  I2Cone.begin(19, 18, 400000);
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);

  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  driver.voltage_power_supply = 16.8;
  driver.init();

  driver1.voltage_power_supply = 16.8;
  driver1.init();

  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;

  motor.PID_velocity.P = 0.12;
  motor1.PID_velocity.P = 0.12;
  motor.PID_velocity.I = 1.2;
  motor1.PID_velocity.I = 1.2;
  motor.PID_velocity.D = 0;
  motor1.PID_velocity.D = 0;

  motor.voltage_limit = 10;
  motor1.voltage_limit = 10;

  motor.LPF_velocity.Tf = 0.02;
  motor1.LPF_velocity.Tf = 0.02;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  motor.init();
  motor1.init();

  motor.initFOC();
  motor1.initFOC();

  Serial.println("Motor ready.");
  Serial.println("Set target velocity in serial terminal, e.g. T10");
}

void loop() {
  motor.loopFOC();
  motor1.loopFOC();

  applyPhaseTargets(millis());

  handleSerialCommand();
}