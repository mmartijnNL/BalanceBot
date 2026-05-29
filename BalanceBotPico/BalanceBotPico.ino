#ifndef BALANCE_BOT_PICO_INO
#define BALANCE_BOT_PICO_INO

#include "BalanceBot.h"
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <stdarg.h>
#include <stdio.h>
#if defined(ARDUINO_ARCH_RP2040) && __has_include(<pico/bootrom.h>)
#include <pico/bootrom.h>
#endif

namespace {

constexpr uint8_t kLeftBusSdaPin = 4;
constexpr uint8_t kLeftBusSclPin = 5;
constexpr uint8_t kRightBusSdaPin = 6;
constexpr uint8_t kRightBusSclPin = 7;

constexpr uint8_t kBatteryAdcPin = 26;
constexpr uint8_t kRcThrottlePin = 20;
constexpr uint8_t kRcSteerPin = 21;

constexpr uint8_t kLeftMotorUPin = 10;
constexpr uint8_t kLeftMotorVPin = 11;
constexpr uint8_t kLeftMotorWPin = 12;
constexpr uint8_t kLeftMotorEnablePin = 13;

constexpr uint8_t kRightMotorUPin = 14;
constexpr uint8_t kRightMotorVPin = 15;
constexpr uint8_t kRightMotorWPin = 16;
constexpr uint8_t kRightMotorEnablePin = 17;

constexpr float kBatteryDividerRatio = 5.0f;
constexpr float kAdcReferenceVoltage = 3.3f;
constexpr float kControlPeriodSeconds = 0.004f;
constexpr uint32_t kPauseToggleDebounceMilliseconds = 200;

TwoWire& leftBus = Wire;
#if defined(PIN_WIRE1_SDA) && defined(PIN_WIRE1_SCL)
TwoWire& rightBus = Wire1;
#else
TwoWire& rightBus = Wire;
#endif
MPU6050 mpu(leftBus);
MagneticSensorI2C sensorLeft = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensorRight = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motorLeft = BLDCMotor(7);
BLDCMotor motorRight = BLDCMotor(7);
BLDCDriver3PWM driverLeft = BLDCDriver3PWM(kLeftMotorUPin, kLeftMotorVPin, kLeftMotorWPin, kLeftMotorEnablePin);
BLDCDriver3PWM driverRight = BLDCDriver3PWM(kRightMotorUPin, kRightMotorVPin, kRightMotorWPin, kRightMotorEnablePin);

BalanceBotConfiguration botConfiguration;
BalanceBotState botState;
uint32_t lastControlMicroseconds = 0;
bool pauseRequested = false;
bool previousPauseButtonPressed = false;
uint32_t lastPauseToggleMilliseconds = 0;

bool readPauseButtonPressed() {
#if defined(ARDUINO_ARCH_RP2040) && __has_include(<pico/bootrom.h>)
    return get_bootsel_button();
#else
    return false;
#endif
}

void handlePauseToggleButton() {
    const bool buttonPressed = readPauseButtonPressed();
    const uint32_t nowMilliseconds = millis();
    if (buttonPressed && !previousPauseButtonPressed &&
        (nowMilliseconds - lastPauseToggleMilliseconds) >= kPauseToggleDebounceMilliseconds) {
        pauseRequested = !pauseRequested;
        lastPauseToggleMilliseconds = nowMilliseconds;
        if (pauseRequested) {
            Serial.println("Paused: motors commanded to zero.");
        } else {
            lastControlMicroseconds = micros();
            Serial.println("Resumed: balancing update loop running.");
        }
    }
    previousPauseButtonPressed = buttonPressed;
}

void initializeI2cBuses() {
#if !defined(ARDUINO_ARCH_MBED)
    leftBus.setSDA(kLeftBusSdaPin);
    leftBus.setSCL(kLeftBusSclPin);
#endif
    leftBus.begin();

#if defined(PIN_WIRE1_SDA) && defined(PIN_WIRE1_SCL)
#if !defined(ARDUINO_ARCH_MBED)
    rightBus.setSDA(kRightBusSdaPin);
    rightBus.setSCL(kRightBusSclPin);
#endif
    rightBus.begin();
#endif
}

void initializeImu() {
    byte imuStatus = mpu.begin();
    if (imuStatus != 0) {
        Serial.print("MPU6050 init failed: ");
        Serial.println(static_cast<int>(imuStatus));
        return;
    }
    mpu.calcOffsets(true, true);
}

void initializeMotors() {
    sensorLeft.init(&leftBus);
    sensorRight.init(&rightBus);

    motorLeft.linkSensor(&sensorLeft);
    motorRight.linkSensor(&sensorRight);

    driverLeft.voltage_power_supply = 12.0f;
    driverRight.voltage_power_supply = 12.0f;
    driverLeft.voltage_limit = botConfiguration.driver_voltage_limit;
    driverRight.voltage_limit = botConfiguration.driver_voltage_limit;
    driverLeft.init();
    driverRight.init();

    motorLeft.linkDriver(&driverLeft);
    motorRight.linkDriver(&driverRight);
    motorLeft.voltage_limit = botConfiguration.driver_voltage_limit;
    motorRight.voltage_limit = botConfiguration.driver_voltage_limit;
    motorLeft.controller = MotionControlType::torque;
    motorRight.controller = MotionControlType::torque;
    motorLeft.init();
    motorRight.init();
    motorLeft.initFOC();
    motorRight.initFOC();
}

}  // namespace

uint32_t hal_millis(void) { return millis(); }
uint32_t hal_micros(void) { return micros(); }
void hal_serial_print(const char* text) { Serial.print(text); }
void hal_serial_printf(const char* fmt, ...) {
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    Serial.print(buffer);
}
uint16_t hal_analog_read(int pin) { return analogRead(pin); }
void hal_motor_left_move(float voltage) { motorLeft.move(voltage); }
void hal_motor_right_move(float voltage) { motorRight.move(-voltage); }
float hal_get_rc_throttle(void) { return 0.0f; }
float hal_get_rc_steer(void) { return 0.0f; }
float hal_get_pitch_deg(void) {
    mpu.update();
    return mpu.getAngleX();
}
float hal_get_gyro_pitch_rate_dps(void) {
    mpu.update();
    return mpu.getGyroX();
}
float hal_get_battery_voltage(void) {
    const float raw = static_cast<float>(analogRead(kBatteryAdcPin));
    const float adcVoltage = (raw / 4095.0f) * kAdcReferenceVoltage;
    return adcVoltage * kBatteryDividerRatio;
}

BalanceBotHardwareAbstractionLayer botHal = {
    hal_millis,
    hal_micros,
    hal_serial_print,
    hal_serial_printf,
    hal_analog_read,
    hal_motor_left_move,
    hal_motor_right_move,
    hal_get_rc_throttle,
    hal_get_rc_steer,
    hal_get_pitch_deg,
    hal_get_gyro_pitch_rate_dps,
    hal_get_battery_voltage,
};

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\nBalanceBot Pico boot");
#if !defined(PIN_WIRE1_SDA) || !defined(PIN_WIRE1_SCL)
    Serial.println("Warning: Wire1 is unavailable on this core/board; two AS5600 sensors cannot share one I2C bus.");
#endif
#if !(defined(ARDUINO_ARCH_RP2040) && __has_include(<pico/bootrom.h>))
    Serial.println("Warning: BOOTSEL button API unavailable; pause/resume button control is disabled.");
#endif

    analogReadResolution(12);
    pinMode(kBatteryAdcPin, INPUT);
    pinMode(kRcThrottlePin, INPUT_PULLDOWN);
    pinMode(kRcSteerPin, INPUT_PULLDOWN);

    BalanceBot_init(&botHal, &botConfiguration, &botState);
    botConfiguration.control_delta_time_seconds = kControlPeriodSeconds;

    initializeI2cBuses();
    initializeImu();
    initializeMotors();

    lastControlMicroseconds = micros();
    Serial.println("Pico target ready. RC input handling is still a stub and should be wired for your receiver.");
}

void loop() {
    handlePauseToggleButton();

    if (pauseRequested) {
        motorLeft.move(0.0f);
        motorRight.move(0.0f);
        motorLeft.loopFOC();
        motorRight.loopFOC();
        return;
    }

    motorLeft.loopFOC();
    motorRight.loopFOC();

    const uint32_t nowMicroseconds = micros();
    const float deltaTime = (nowMicroseconds - lastControlMicroseconds) * 1e-6f;
    if (deltaTime < botConfiguration.control_delta_time_seconds) {
        return;
    }

    lastControlMicroseconds = nowMicroseconds;
    BalanceBot_update(&botConfiguration, &botState);
}

#endif // BALANCE_BOT_PICO_INO