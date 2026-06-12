#ifndef BALANCE_BOT_LOLIN32LITE_INO
#define BALANCE_BOT_LOLIN32LITE_INO

#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <cmath>

namespace {

constexpr uint8_t kLeftI2cSdaPin = 19;
constexpr uint8_t kLeftI2cSclPin = 18;
constexpr uint8_t kRightI2cSdaPin = 23;
constexpr uint8_t kRightI2cSclPin = 5;

constexpr int kMotorPolePairs = 7;

constexpr uint8_t kLeftPwmUPin = 32;
constexpr uint8_t kLeftPwmVPin = 33;
constexpr uint8_t kLeftPwmWPin = 25;
constexpr uint8_t kLeftEnablePin = 22;

constexpr uint8_t kRightPwmUPin = 26;
constexpr uint8_t kRightPwmVPin = 27;
constexpr uint8_t kRightPwmWPin = 14;
constexpr uint8_t kRightEnablePin = 12;

constexpr uint8_t kRcThrottlePin = 34;
constexpr uint8_t kRcSteerPin = 35;

constexpr float kSupplyVoltage = 12.0f;
constexpr float kMotorVoltageLimit = 12.0f;
constexpr float kFollowerVelocityGain = 10.0f;
constexpr float kMasterTorqueGain = 5.0f;
constexpr float kPitchRateDampingGain = 0.08f;
constexpr float kRcThrottleAngleGain = 0.15f;
constexpr float kRcSteerVelocityGain = 2.0f;
constexpr float kDeadZoneRadians = 0.2f;
constexpr float kDegreesToRadians = 0.017453292519943295f;

TwoWire leftI2cBus = TwoWire(0);
TwoWire rightI2cBus = TwoWire(1);

MagneticSensorI2C leftSensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C rightSensor = MagneticSensorI2C(AS5600_I2C);
MPU6050 imu = MPU6050(leftI2cBus);

BLDCMotor leftMotor = BLDCMotor(kMotorPolePairs);
BLDCDriver3PWM leftDriver = BLDCDriver3PWM(kLeftPwmUPin, kLeftPwmVPin, kLeftPwmWPin, kLeftEnablePin);

BLDCMotor rightMotor = BLDCMotor(kMotorPolePairs);
BLDCDriver3PWM rightDriver = BLDCDriver3PWM(kRightPwmUPin, kRightPwmVPin, kRightPwmWPin, kRightEnablePin);

float deadZone(float value) {
    return (std::fabs(value) < kDeadZoneRadians) ? 0.0f : value;
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

void initializeI2cBuses() {
#if defined(ARDUINO_ARCH_ESP32)
    leftI2cBus.begin(kLeftI2cSdaPin, kLeftI2cSclPin, 400000);
    rightI2cBus.begin(kRightI2cSdaPin, kRightI2cSclPin, 400000);
#else
    leftI2cBus.setSDA(kLeftI2cSdaPin);
    leftI2cBus.setSCL(kLeftI2cSclPin);
    leftI2cBus.begin();

    rightI2cBus.setSDA(kRightI2cSdaPin);
    rightI2cBus.setSCL(kRightI2cSclPin);
    rightI2cBus.begin();
#endif
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(250);
    Serial.println("\nBalanceBot SimpleFOC boot");

    pinMode(kRcThrottlePin, INPUT);
    pinMode(kRcSteerPin, INPUT);

    initializeI2cBuses();

    const byte imuStatus = imu.begin();
    if (imuStatus != 0) {
        Serial.print("MPU6050 init failed, status=");
        Serial.println(static_cast<int>(imuStatus));
    } else {
        imu.calcOffsets(true, true);
    }

    leftSensor.init(&leftI2cBus);
    rightSensor.init(&rightI2cBus);

    leftMotor.linkSensor(&leftSensor);
    rightMotor.linkSensor(&rightSensor);

    leftDriver.voltage_power_supply = kSupplyVoltage;
    leftDriver.init();
    rightDriver.voltage_power_supply = kSupplyVoltage;
    rightDriver.init();

    leftMotor.linkDriver(&leftDriver);
    rightMotor.linkDriver(&rightDriver);

    leftMotor.controller = MotionControlType::torque;
    rightMotor.controller = MotionControlType::velocity;

    leftMotor.voltage_limit = kMotorVoltageLimit;
    rightMotor.voltage_limit = kMotorVoltageLimit;
    rightMotor.LPF_velocity.Tf = 0.01f;
    rightMotor.PID_velocity.I = 1.0f;

    leftMotor.useMonitoring(Serial);
    rightMotor.useMonitoring(Serial);

    leftMotor.init();
    rightMotor.init();
    leftMotor.initFOC();
    rightMotor.initFOC();

    Serial.println("SimpleFOC motors ready.");
}

void loop() {
    leftMotor.loopFOC();
    rightMotor.loopFOC();

    imu.update();
    const float pitchRadians = imu.getAngleX() * kDegreesToRadians;
    const float pitchRateRadiansPerSecond = imu.getGyroX() * kDegreesToRadians;

    const float rcThrottle = readRcChannel(kRcThrottlePin);
    const float rcSteer = readRcChannel(kRcSteerPin);
    const float targetPitchRadians = rcThrottle * kRcThrottleAngleGain;
    const float effectivePitch = pitchRadians - targetPitchRadians;
    const float steerBias = rcSteer * kRcSteerVelocityGain;

    const float followerVelocityTarget = kFollowerVelocityGain * deadZone(effectivePitch) + steerBias;
    rightMotor.move(followerVelocityTarget);

    const float masterTorqueTarget = (kMasterTorqueGain * ((rightMotor.shaft_velocity / 10.0f) - effectivePitch))
        - (kPitchRateDampingGain * pitchRateRadiansPerSecond);
    leftMotor.move(masterTorqueTarget);
}

#endif // BALANCE_BOT_LOLIN32LITE_INO