#ifndef BALANCE_BOT_LOLIN32LITE_INO
#define BALANCE_BOT_LOLIN32LITE_INO

#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <cmath>
#include <cstdio>

namespace {

// PID tuning values
constexpr float kP = 9.0f;  // Proportional: stiffness, how hard the bot fights tilt
constexpr float kI = 0.0f;  // Integral: corrects steady-state lean / drift
constexpr float kD = 0.02f;  // Derivative: damping, reduces oscillation

constexpr float kIntegralClamp = 1.2f;       // Anti-windup: max magnitude of the integral term
constexpr float kWheelVelocityDampingGain = 0.06f;

constexpr float kSupplyVoltage = 16.8f;         // 4S LiPo: full=16.8 low=14
constexpr float kMotorVoltageLimit = 8.0f;
constexpr float kLeftMotorDirection =   1.0f;   // Reverse if needed
constexpr float kRightMotorDirection =  -1.0f;  // Reverse if needed

// Radio Control
bool kEnableRcReceiver = false;
constexpr float kRcThrottleAngleGain =  0.4f;
constexpr float kRcSteerTorqueGain =    1.0f;

constexpr float kZeroDeadband = 0.05f;        // Ignore tiny commands that cause chatter

constexpr bool kEnableTelemetry = true;
constexpr unsigned long kTelemetryPeriodMs = 50UL;
constexpr unsigned long kSerialBaudRate = 460800UL;
constexpr float kDegreesToRadians =     0.017453292519943295f;
constexpr float kRadiansToDegrees =     57.29577951308232f;

// IMU angle reading
// kPitchAxisIndex: 0 = rotate around X (uses accY/accZ, gyroX)
//                  1 = rotate around Y (uses accX/accZ, gyroY)
// Tilt the bot slowly forward and check which axis tracks it; swap if needed.
constexpr int   kPitchAxisIndex = 1;     // 0 or 1 — change to match board mounting
constexpr float kPitchSign      = 1.0f;  // Flip to -1 if angle is inverted vs real world
constexpr float kCfAlpha        = 0.98f; // Complementary filter: 0.98 = heavy gyro trust

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

// Complementary filter state
float         cfPitchRadians  = 0.0f;
unsigned long cfLastUpdateUs  = 0UL;
bool          cfInitialized   = false;

float clampAbs(float value, float limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

struct PitchReading {
    float radians;
    float rateRadiansPerSecond;
    float angleDegrees() const { return radians * kRadiansToDegrees; }
    float rateDegreesPerSecond() const { return rateRadiansPerSecond * kRadiansToDegrees; }
};

PitchReading readPitch() {
    imu.update();

    // Raw values from MPU6050_light (offsets already applied by library)
    // Accel in g units; gyro in deg/s
    float accelTilt, gyroRateDegPerSec;
    if (kPitchAxisIndex == 0) {
        // Board tilts around X-axis: gravity component shifts between Y and Z
        accelTilt        = imu.getAccY();
        gyroRateDegPerSec = imu.getGyroX();
    } else {
        // Board tilts around Y-axis: gravity component shifts between X and Z
        accelTilt        = imu.getAccX();
        gyroRateDegPerSec = imu.getGyroY();
    }
    const float accelZ = imu.getAccZ();
    const float gyroRateRadPerSec = gyroRateDegPerSec * kDegreesToRadians;

    // Accelerometer-only angle: full -π..+π range, 0 = level
    // Positive when the tilt-axis side rises, negative when it drops.
    const float accPitchRadians = atan2f(-accelTilt, accelZ);

    const unsigned long nowUs = micros();
    if (!cfInitialized) {
        cfPitchRadians = accPitchRadians;
        cfInitialized  = true;
    } else {
        const float dt = static_cast<float>(nowUs - cfLastUpdateUs) * 1.0e-6f;
        // Complementary filter: integrate gyro (fast response) + correct drift with accel
        cfPitchRadians = kCfAlpha * (cfPitchRadians + gyroRateRadPerSec * dt)
                       + (1.0f - kCfAlpha) * accPitchRadians;
    }
    cfLastUpdateUs = nowUs;

    PitchReading reading;
    reading.radians              = kPitchSign * cfPitchRadians;
    reading.rateRadiansPerSecond = kPitchSign * gyroRateRadPerSec;
    return reading;
}

// RC receiver state — written in ISRs, read in loop()
volatile unsigned long rcThrottleRiseUs = 0UL;
volatile unsigned long rcThrottlePulseUs = 1500UL;
volatile unsigned long rcThrottleLastUpdateUs = 0UL;
volatile unsigned long rcSteerRiseUs = 0UL;
volatile unsigned long rcSteerPulseUs = 1500UL;
volatile unsigned long rcSteerLastUpdateUs = 0UL;

void IRAM_ATTR onRcThrottleChange() {
    const unsigned long now = micros();
    if (digitalRead(39) == HIGH) {
        rcThrottleRiseUs = now;
    } else if (rcThrottleRiseUs != 0UL) {
        rcThrottlePulseUs = now - rcThrottleRiseUs;
        rcThrottleLastUpdateUs = now;
    }
}

void IRAM_ATTR onRcSteerChange() {
    const unsigned long now = micros();
    if (digitalRead(36) == HIGH) {
        rcSteerRiseUs = now;
    } else if (rcSteerRiseUs != 0UL) {
        rcSteerPulseUs = now - rcSteerRiseUs;
        rcSteerLastUpdateUs = now;
    }
}

// Non-blocking RC read — returns 0 if signal is stale (>100 ms)
float getRcChannel(volatile unsigned long& pulseUs, volatile unsigned long& lastUpdateUs) {
    constexpr unsigned long kRcPulseMin = 1000UL;
    constexpr unsigned long kRcPulseMax = 2000UL;
    constexpr unsigned long kRcPulseCenter = 1500UL;
    constexpr unsigned long kRcSignalTimeout = 100000UL;  // 100 ms
    if ((micros() - lastUpdateUs) > kRcSignalTimeout) return 0.0f;
    const unsigned long pulse = pulseUs;
    const unsigned long clamped = (pulse < kRcPulseMin) ? kRcPulseMin
        : (pulse > kRcPulseMax ? kRcPulseMax : pulse);
    return static_cast<float>(static_cast<long>(clamped) - static_cast<long>(kRcPulseCenter)) / 500.0f;
}

void emitTelemetry(unsigned long nowMs,
    PitchReading pitchReading,
    float targetPitch,
    float leftWheelVelocity,
    float rightWheelVelocity) {
    if (!kEnableTelemetry) return;
    if ((nowMs - lastTelemetryMs) < kTelemetryPeriodMs) return;

    const float targetPitchDegrees = targetPitch * kRadiansToDegrees;

    char telemetryLine[256];
    const int n = snprintf(
        telemetryLine,
        sizeof(telemetryLine),
        // Keep each line compact so it fits typical UART TX buffers without blocking.
        "p:%.2f,pr:%.2f,tp:%.2f,wl:%.3f,wr:%.3f\n",
        static_cast<double>(pitchReading.angleDegrees()),
        static_cast<double>(pitchReading.rateDegreesPerSecond()),
        static_cast<double>(targetPitchDegrees),
        static_cast<double>(leftWheelVelocity),
        static_cast<double>(rightWheelVelocity));

    if (n > 0 && n < static_cast<int>(sizeof(telemetryLine))) {
        // Skip rather than block: motor loop timing is more important than telemetry.
        const int available = Serial.availableForWrite();
        if (available <= 0) return;
        if (available < n) return;
        Serial.write(reinterpret_cast<const uint8_t*>(telemetryLine), static_cast<size_t>(n));
        lastTelemetryMs = nowMs;
    }
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);
    delay(250);
    Serial.println("\nBalanceBot SimpleFOC boot");

    pinMode(39, INPUT);
    pinMode(36, INPUT);
    attachInterrupt(digitalPinToInterrupt(39), onRcThrottleChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(36), onRcSteerChange, CHANGE);

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
        startupPitchSum += readPitch().radians;
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

    const PitchReading pitch = readPitch();
    const float pitchRadians = pitch.radians;
    const float pitchRateRadiansPerSecond = pitch.rateRadiansPerSecond;

    const unsigned long nowMs = millis();
    const float dtSeconds = static_cast<float>(nowMs - lastLoopMs) * 0.001f;
    lastLoopMs = nowMs;


    const float rcThrottle = getRcChannel(rcThrottlePulseUs, rcThrottleLastUpdateUs);
    const float rcSteer    = getRcChannel(rcSteerPulseUs,    rcSteerLastUpdateUs);

    if(!kEnableRcReceiver && (rcThrottle > 0.3f || rcSteer > 0.3f || rcSteer < -0.3f))
    {
        Serial.println("Enabling RC control");
        kEnableRcReceiver = true;
    }

    const float targetPitchRadians = startupPitchReferenceRadians + ((kEnableRcReceiver ? rcThrottle : 0.0f) * kRcThrottleAngleGain);
    // Pitch is a local balance angle, not a heading; use linear subtraction to avoid wrap-around flips.
    const float effectivePitch = pitchRadians - targetPitchRadians;

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
    const float steerTorque = (kEnableRcReceiver ? rcSteer : 0.0f)  * kRcSteerTorqueGain;

    float leftTorque = balanceTorqueTarget - steerTorque;
    float rightTorque = balanceTorqueTarget + steerTorque;

    if (fabsf(leftTorque) < kZeroDeadband) leftTorque = 0.0f;
    if (fabsf(rightTorque) < kZeroDeadband) rightTorque = 0.0f;

    leftMotor.move(kLeftMotorDirection * leftTorque);
    rightMotor.move(kRightMotorDirection * rightTorque);

    emitTelemetry(
        nowMs,
        pitch,
        targetPitchRadians,
        leftWheelVelocityForward,
        rightWheelVelocityForward);
}

#endif // BALANCE_BOT_LOLIN32LITE_INO