#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <Wire.h>

namespace {

constexpr bool kRightMotorReverse =  false;  // Reverse if needed

// PID tuning values
constexpr float kP = 0.01f;  // Proportional: stiffness, how hard the bot fights tilt
constexpr float kI = 0.0f;  // Integral: disabled during initial stabilization tuning
constexpr float kD = 0.01f;  // Derivative: damping, reduces oscillation (uses pitch rate from gyro)

constexpr unsigned long kTelemetryPeriodMs = 100UL;

// Create Sensors
TwoWire i2cLeft = TwoWire(0);
MagneticSensorI2C leftSensor = MagneticSensorI2C(AS5600_I2C);
TwoWire i2cRight = TwoWire(1);
MagneticSensorI2C rightSensor = MagneticSensorI2C(AS5600_I2C);
MPU6050 imu = MPU6050(i2cLeft);

// Create Motors
BLDCMotor leftMotor = BLDCMotor(7);     // 7 pole pairs
BLDCDriver3PWM leftDriver = BLDCDriver3PWM(23, 18, 5, 17);
BLDCMotor rightMotor = BLDCMotor(7);    // 7 pole pairs
BLDCDriver3PWM rightDriver = BLDCDriver3PWM(25, 26, 27, 14);

float initialOffset = 0;


}  // namespace

void setup() {
    Serial.begin(115200);
    delay(250);
    Serial.println("\nInitializing");

    // Initialize sensors
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

    // Link sensors to motors
    leftMotor.linkSensor(&leftSensor);
    rightMotor.linkSensor(&rightSensor);

    // Initialize motors
    leftDriver.voltage_power_supply = 16.8f;         // 4S LiPo: full=16.8 low=14
    rightDriver.voltage_power_supply = 16.8f;         // 4S LiPo: full=16.8 low=14
    leftDriver.init();
    rightDriver.init();

    leftMotor.linkDriver(&leftDriver);
    rightMotor.linkDriver(&rightDriver);

    leftMotor.controller = MotionControlType::torque;
    rightMotor.controller = MotionControlType::torque;

    leftMotor.voltage_limit = 8;
    rightMotor.voltage_limit = 8;

    leftMotor.useMonitoring(Serial);
    rightMotor.useMonitoring(Serial);

    leftMotor.init();
    rightMotor.init();
    leftMotor.initFOC();
    rightMotor.initFOC();

    imu.update();

    initialOffset = imu.getAngleY();

    Serial.println("Initialized.");
}

static float integral = 0;
static float lastError = 0;

void loop() {
    static unsigned long lastLoopMs = 0;
    static unsigned long lastTelemetryMs = 0;

    leftMotor.loopFOC();
    rightMotor.loopFOC();

    // Get angle
    imu.update();
    float angle = imu.getAngleY() - initialOffset;
    // wrap to [-180, 180)
    if (angle > 180.0f) angle -= 360.0f;
    else if (angle <= -180.0f) angle += 360.0f;

    // Get time delta
    const unsigned long nowMs = millis();
    const float dtSeconds = static_cast<float>(nowMs - lastLoopMs) * 0.001f;
    lastLoopMs = nowMs;

    // PID loop
    float targetAngle = 0; // Desired pitch angle (upright position)
    float error = targetAngle - angle;
    integral += error * dtSeconds;
    float derivative = (error - lastError) / dtSeconds;
    lastError = error;
    float pidResult = (kP * error) + (kI * integral) + (kD * derivative);

    rightMotor.move(kRightMotorReverse ? -pidResult : pidResult);

    if ((nowMs - lastTelemetryMs) >= kTelemetryPeriodMs) {
        Serial.print(angle);
        Serial.print("\t");
        Serial.print(pidResult);

        Serial.print("\n");
        lastTelemetryMs = nowMs;
    }
}