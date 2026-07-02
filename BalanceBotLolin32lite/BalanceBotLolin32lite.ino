#ifndef BALANCE_BOT_LOLIN32LITE_INO
#define BALANCE_BOT_LOLIN32LITE_INO

#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <cmath>

namespace {

constexpr bool kEnableRcReceiver = false;

constexpr float kSupplyVoltage = 16.8f;
constexpr float kMotorVoltageLimit = 8.0f;

// --- PID tuning values ---
constexpr float kP = 3.0f;   // Proportional: stiffness, how hard the bot fights tilt
constexpr float kI = 0.4f;   // Integral:     corrects steady-state lean bias (start at 0, increase slowly)
constexpr float kD = 0.5f;  // Derivative:   damping, reduces oscillation (uses pitch rate from gyro)
// ---------------------------

constexpr float kIntegralClamp = 1.0f;       // Anti-windup: max magnitude of the integral term
constexpr float kWheelVelocityDampingGain = 0.10f;

constexpr float kRcThrottleAngleGain = 0.15f;
constexpr float kRcSteerTorqueGain = 1.6f;
constexpr unsigned long kTelemetryPeriodMs = 250UL;
constexpr float kDegreesToRadians = 0.017453292519943295f;
constexpr float kLeftMotorDirection =   -1.0f;
constexpr float kRightMotorDirection =  1.0f;

TwoWire i2cLeft = TwoWire(0);
TwoWire i2cRight = TwoWire(1);

MagneticSensorI2C leftSensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C rightSensor = MagneticSensorI2C(AS5600_I2C);
MPU6050 imu = MPU6050(i2cRight);

BLDCMotor leftMotor = BLDCMotor(7);     // 7 pole pairs
BLDCDriver3PWM leftDriver = BLDCDriver3PWM(23, 18, 5, 17);

BLDCMotor rightMotor = BLDCMotor(7);    // 7 pole pairs
BLDCDriver3PWM rightDriver = BLDCDriver3PWM(25, 26, 27, 14);

float pitchIntegral = 0.0f;
unsigned long lastTelemetryMs = 0UL;
unsigned long lastLoopMs = 0UL;

float clampAbs(float value, float limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

float readRcChannel(uint8_t pin) {
    constexpr unsigned long kRcPulseMin = 1000UL;
    constexpr unsigned long kRcPulseMax = 2000UL;
    constexpr unsigned long kRcPulseCenter = 1500UL;
    constexpr unsigned long kRcTimeout = 25000UL;
    const unsigned long pulse = pulseIn(pin, HIGH, kRcTimeout);
    if (pulse == 0) return 0.0f;
    const unsigned long clamped = (pulse < kRcPulseMin) ? kRcPulseMin
        : (pulse > kRcPulseMax ? kRcPulseMax : pulse);
    return static_cast<float>(static_cast<long>(clamped) - static_cast<long>(kRcPulseCenter)) / 500.0f;
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(250);
    Serial.println("\nBalanceBot SimpleFOC boot");

    if (kEnableRcReceiver) {
        pinMode(39, INPUT);
        pinMode(36, INPUT);
    }

    i2cLeft.begin(22, 19, 100000); 
    i2cRight.begin(32, 33, 100000);

    const byte imuStatus = imu.begin();
    if (imuStatus != 0) {
        Serial.print("MPU6050 init failed, status=");
        Serial.println(static_cast<int>(imuStatus));
    } else {
        imu.calcOffsets(true, true);
    }

    leftSensor.init(&i2cLeft);
    rightSensor.init(&i2cRight);

    leftMotor.linkSensor(&leftSensor);
    rightMotor.linkSensor(&rightSensor);

    leftDriver.voltage_power_supply = kSupplyVoltage;
    leftDriver.init();
    rightDriver.voltage_power_supply = kSupplyVoltage;
    rightDriver.init();

    leftMotor.linkDriver(&leftDriver);
    rightMotor.linkDriver(&rightDriver);

    leftMotor.controller = MotionControlType::torque;
    rightMotor.controller = MotionControlType::torque;

    leftMotor.voltage_limit = kMotorVoltageLimit;
    rightMotor.voltage_limit = kMotorVoltageLimit;

    leftMotor.useMonitoring(Serial);
    rightMotor.useMonitoring(Serial);

    leftMotor.init();
    rightMotor.init();
    leftMotor.initFOC();
    rightMotor.initFOC();

    imu.update();

    const unsigned long nowMs = millis();
    lastTelemetryMs = nowMs;
    lastLoopMs = nowMs;

    Serial.println("SimpleFOC motors ready.");
}

void loop() {
    leftMotor.loopFOC();
    rightMotor.loopFOC();

    imu.update();
    const float pitchRadians = imu.getAngleX() * kDegreesToRadians;
    const float pitchRateRadiansPerSecond = imu.getGyroX() * kDegreesToRadians;

    const unsigned long nowMs = millis();
    const float dtSeconds = static_cast<float>(nowMs - lastLoopMs) * 0.001f;
    lastLoopMs = nowMs;

    const float rcThrottle = kEnableRcReceiver ? readRcChannel(39) : 0.0f;
    const float rcSteer = kEnableRcReceiver ? readRcChannel(36) : 0.0f;
    const float targetPitchRadians = rcThrottle * kRcThrottleAngleGain;
    const float effectivePitch = pitchRadians - targetPitchRadians;

    // Integral: accumulate error over time, clamped to prevent windup
    pitchIntegral += effectivePitch * dtSeconds;
    pitchIntegral = clampAbs(pitchIntegral, kIntegralClamp);

    const float balanceTorqueTarget =
        (-kP * effectivePitch)                     // Proportional
        - (kI * pitchIntegral)                     // Integral
        - (kD * pitchRateRadiansPerSecond)          // Derivative
        - (kWheelVelocityDampingGain * 0.5f * (leftMotor.shaft_velocity + rightMotor.shaft_velocity));
    const float steerTorque = rcSteer * kRcSteerTorqueGain;

    const float leftTorque = balanceTorqueTarget - steerTorque;
    const float rightTorque = balanceTorqueTarget + steerTorque;

    leftMotor.move(kLeftMotorDirection * leftTorque);
    rightMotor.move(kRightMotorDirection * rightTorque);

    if ((nowMs - lastTelemetryMs) >= kTelemetryPeriodMs) {
        Serial.print("CTRL=ON ");
        Serial.print("pitch=");
        Serial.print(effectivePitch);
        Serial.print(" target=");
        Serial.print(targetPitchRadians);
        Serial.print(" rate=");
        Serial.print(pitchRateRadiansPerSecond);
        Serial.print(" integral=");
        Serial.print(pitchIntegral);
        Serial.print(" wheelL=");
        Serial.print(leftMotor.shaft_velocity);
        Serial.print(" wheelR=");
        Serial.print(rightMotor.shaft_velocity);
        Serial.print(" cmdL=");
        Serial.print(leftTorque);
        Serial.print(" cmdR=");
        Serial.println(rightTorque);
        lastTelemetryMs = nowMs;
    }
}

#endif // BALANCE_BOT_LOLIN32LITE_INO