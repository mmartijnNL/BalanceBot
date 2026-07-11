#ifndef BALANCE_BOT_LOLIN32LITE_INO
#define BALANCE_BOT_LOLIN32LITE_INO

#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <cmath>

namespace {

// PID tuning values
constexpr float kP = 4.0f;  // Proportional: stiffness, how hard the bot fights tilt
constexpr float kI = 0.0f;  // Integral: corrects steady-state lean / drift
constexpr float kD = 0.01f;  // Derivative: damping, reduces oscillation

constexpr float kIntegralClamp = 1.2f;       // Anti-windup: max magnitude of the integral term
constexpr float kWheelVelocityDampingGain = 0.06f;

constexpr float kSupplyVoltage = 16.8f;         // 4S LiPo: full=16.8 low=14
constexpr float kMotorVoltageLimit = 8.0f;
constexpr float kLeftMotorDirection =   1.0f;   // Reverse if needed
constexpr float kRightMotorDirection =  -1.0f;  // Reverse if needed

// Radio Control
bool kEnableRcReceiver = false;
constexpr float kRcThrottleAngleGain =  0.15f;
constexpr float kRcSteerTorqueGain =    1.6f;

constexpr float kZeroDeadband = 0.05f;        // Ignore tiny commands that cause chatter

constexpr unsigned long kTelemetryPeriodMs = 2UL;
constexpr float kDegreesToRadians =     0.017453292519943295f;
constexpr float kPitchAngleBlend =      0.90f;  // 1.0=gyro-heavy, 0.0=accel-heavy

TwoWire i2cLeft = TwoWire(0);
TwoWire i2cRight = TwoWire(1);

MagneticSensorI2C leftSensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C rightSensor = MagneticSensorI2C(AS5600_I2C);
MPU6050 imu = MPU6050(i2cLeft);

BLDCMotor leftMotor = BLDCMotor(7);     // 7 pole pairs
BLDCDriver3PWM leftDriver = BLDCDriver3PWM(23, 18, 5, 17);

BLDCMotor rightMotor = BLDCMotor(7);    // 7 pole pairs
BLDCDriver3PWM rightDriver = BLDCDriver3PWM(25, 26, 27, 14);

float startupPitchReferenceRadians = 0.0f;
float pitchIntegral = 0.0f;
unsigned long lastTelemetryMs = 0UL;
unsigned long lastLoopMs = 0UL;

float clampAbs(float value, float limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

float wrappedAngleErrorRadians(float angleRadians, float referenceRadians) {
    float error = angleRadians - referenceRadians;
    while (error > PI) error -= 2.0f * PI;
    while (error < -PI) error += 2.0f * PI;
    return error;
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


float getPitchRadians(){
    imu.update();
    const float pitchFromGyroDegrees = imu.getAngleX();
    const float pitchFromAccDegrees = imu.getAccAngleX();
    const float pitchRadians =
        (kPitchAngleBlend * pitchFromGyroDegrees + (1.0f - kPitchAngleBlend) * pitchFromAccDegrees)
        * kDegreesToRadians;
    return pitchRadians;
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(250);
    Serial.println("\nBalanceBot SimpleFOC boot");

    pinMode(39, INPUT);
    pinMode(36, INPUT);


    i2cLeft.begin(22, 19, 100000); 
    i2cRight.begin(32, 33, 100000);

    const byte imuStatus = imu.begin();
    if (imuStatus != 0) {
        Serial.print("MPU6050 init failed, status=");
        Serial.println(static_cast<int>(imuStatus));
    } else {
        // Calibrate gyro while stationary. Accel offset calibration requires a flat/level pose,
        // which is not true when the robot is held upright.
        imu.calcOffsets(true, false);
    }

    imu.update();
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

    // Capture current hold angle as startup target so control begins near zero torque.
    float startupPitchSum = 0.0f;
    constexpr int kStartupSamples = 120;
    for (int i = 0; i < kStartupSamples; ++i) {
        startupPitchSum += getPitchRadians();
        delay(2);
    }
    startupPitchReferenceRadians = startupPitchSum / static_cast<float>(kStartupSamples);

    const unsigned long nowMs = millis();
    lastTelemetryMs = nowMs;
    lastLoopMs = nowMs;

    Serial.println("SimpleFOC motors ready.");
}

void loop() {
    leftMotor.loopFOC();
    rightMotor.loopFOC();

    const float pitchRadians = getPitchRadians();
    const float pitchRateRadiansPerSecond = imu.getGyroX() * kDegreesToRadians;

    const unsigned long nowMs = millis();
    const float dtSeconds = static_cast<float>(nowMs - lastLoopMs) * 0.001f;
    lastLoopMs = nowMs;

    if(!kEnableRcReceiver && (readRcChannel(39) > 0.3f || readRcChannel(36) > 0.3f || readRcChannel(36) < 0.3f))
    {
        Serial.println("Enabling RC control");
        kEnableRcReceiver = true;
    }

    const float rcThrottle = kEnableRcReceiver ? readRcChannel(39) : 0.0f;
    const float rcSteer = kEnableRcReceiver ? readRcChannel(36) : 0.0f;
    const float targetPitchRadians = startupPitchReferenceRadians + (rcThrottle * kRcThrottleAngleGain);
    const float effectivePitch = wrappedAngleErrorRadians(pitchRadians, targetPitchRadians);

    // Integral: accumulate error over time, clamped to prevent windup
    pitchIntegral += effectivePitch * dtSeconds;
    pitchIntegral = clampAbs(pitchIntegral, kIntegralClamp);

    const float leftWheelVelocityForward = kLeftMotorDirection * leftMotor.shaft_velocity;
    const float rightWheelVelocityForward = kRightMotorDirection * rightMotor.shaft_velocity;
    const float averageWheelVelocityForward = 0.5f * (leftWheelVelocityForward + rightWheelVelocityForward);

    const float balanceTorqueTarget =
        (-kP * effectivePitch)                     // Proportional
        - (kI * pitchIntegral)                     // Integral
        - (kD * pitchRateRadiansPerSecond)          // Derivative
        - (kWheelVelocityDampingGain * averageWheelVelocityForward);
    const float steerTorque = rcSteer * kRcSteerTorqueGain;

    float leftTorque = balanceTorqueTarget - steerTorque;
    float rightTorque = balanceTorqueTarget + steerTorque;

    if (fabsf(leftTorque) < kZeroDeadband) leftTorque = 0.0f;
    if (fabsf(rightTorque) < kZeroDeadband) rightTorque = 0.0f;

    leftMotor.move(kLeftMotorDirection * leftTorque);
    rightMotor.move(kRightMotorDirection * rightTorque);

    if ((nowMs - lastTelemetryMs) >= kTelemetryPeriodMs) {
        Serial.print("pitch:");
        Serial.print(effectivePitch);
        Serial.print(",targetPitch:");
        Serial.print(targetPitchRadians);
        Serial.print(",rate:");
        Serial.print(pitchRateRadiansPerSecond);
        Serial.print(",integral:");
        Serial.print(pitchIntegral);
        Serial.print(",wheelL:");
        Serial.print(leftMotor.shaft_velocity);
        Serial.print(",wheelR:");
        Serial.print(rightMotor.shaft_velocity);
        Serial.print(",wheelFwd:");
        Serial.print(averageWheelVelocityForward);
        Serial.print(",rcThrottle:");
        Serial.print(readRcChannel(39));
        Serial.print(",rcSteer:");
        Serial.print(readRcChannel(36));

        Serial.println("");
        lastTelemetryMs = nowMs;
    }
}

#endif // BALANCE_BOT_LOLIN32LITE_INO