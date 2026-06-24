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

MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2C0 = TwoWire(0);
TwoWire I2C1 = TwoWire(1);

BLDCMotor motor0 = BLDCMotor(7);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(23, 18, 5, 17);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(25, 26, 27, 14);

float target_velocity = 3; 

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

  motor0.move(motor0Target);
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
  I2C0.begin(22, 19, 400000);
  I2C1.begin(32, 33, 400000);
  sensor0.init(&I2C0);
  sensor1.init(&I2C1);

  motor0.linkSensor(&sensor0);
  motor1.linkSensor(&sensor1);

  driver0.voltage_power_supply = 16.8;
  driver0.init();

  driver1.voltage_power_supply = 16.8;
  driver1.init();

  motor0.linkDriver(&driver0);
  motor1.linkDriver(&driver1);

  motor0.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;

  motor0.PID_velocity.P = 0.12;
  motor1.PID_velocity.P = 0.12;
  motor0.PID_velocity.I = 1.2;
  motor1.PID_velocity.I = 1.2;
  motor0.PID_velocity.D = 0;
  motor1.PID_velocity.D = 0;

  motor0.voltage_limit = 6;                                 
  motor1.voltage_limit = 6;

  motor0.LPF_velocity.Tf = 0.02;
  motor1.LPF_velocity.Tf = 0.02;

  Serial.begin(115200);
  motor0.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  motor0.init();
  motor1.init();

  motor0.initFOC();
  motor1.initFOC();

  Serial.println("Motor ready.");
  Serial.println("Set target velocity in serial terminal, e.g. T10");
}

void loop() {
  motor0.loopFOC();
  motor1.loopFOC();

  applyPhaseTargets(millis());

  handleSerialCommand();
}