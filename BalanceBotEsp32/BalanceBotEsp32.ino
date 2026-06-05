#ifndef BALANCE_BOT_ESP32_INO
#define BALANCE_BOT_ESP32_INO

#include "BalanceBot.h"
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <stdarg.h>
#include <stdio.h>

namespace {

constexpr uint8_t kImuSdaPin = 21;
constexpr uint8_t kImuSclPin = 22;

constexpr uint8_t kBatteryAdcPin = 34;
constexpr uint8_t kRcThrottlePin = 36;
constexpr uint8_t kRcSteerPin = 39;

constexpr uint8_t kPauseButtonPin = 0;
constexpr uint32_t kPauseToggleDebounceMilliseconds = 200;

constexpr uint8_t kMksUartTxPin = 17;
constexpr uint8_t kMksUartRxPin = 16;
constexpr uint32_t kMksUartBaud = 115200;

constexpr float kBatteryDividerRatio = 5.0f;
constexpr float kAdcReferenceVoltage = 3.3f;
constexpr float kControlPeriodSeconds = 0.004f;

TwoWire& imuBus = Wire;
MPU6050 mpu(imuBus);

BalanceBotConfiguration botConfiguration;
BalanceBotState botState;
uint32_t lastControlMicroseconds = 0;
bool pauseRequested = false;
bool previousPauseButtonPressed = false;
uint32_t lastPauseToggleMilliseconds = 0;
float pendingLeftCommand = 0.0f;
float pendingRightCommand = 0.0f;

void sendMksCommand(float leftCommand, float rightCommand) {
    Serial1.print("M ");
    Serial1.print(leftCommand);
    Serial1.print(" ");
    Serial1.println(rightCommand);
}

bool readPauseButtonPressed() {
    return digitalRead(kPauseButtonPin) == LOW;
}

void handlePauseToggleButton() {
    const bool buttonPressed = readPauseButtonPressed();
    const uint32_t nowMilliseconds = millis();
    if (buttonPressed && !previousPauseButtonPressed &&
        (nowMilliseconds - lastPauseToggleMilliseconds) >= kPauseToggleDebounceMilliseconds) {
        pauseRequested = !pauseRequested;
        lastPauseToggleMilliseconds = nowMilliseconds;
        if (pauseRequested) {
            sendMksCommand(0.0f, 0.0f);
            Serial.println("Paused: motors commanded to zero.");
        } else {
            lastControlMicroseconds = micros();
            Serial.println("Resumed: balancing update loop running.");
        }
    }
    previousPauseButtonPressed = buttonPressed;
}

void initializeI2cBus() {
#if defined(ARDUINO_ARCH_ESP32)
    imuBus.begin(kImuSdaPin, kImuSclPin);
#else
    imuBus.setSDA(kImuSdaPin);
    imuBus.setSCL(kImuSclPin);
    imuBus.begin();
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

void initializeMksUart() {
#if defined(ARDUINO_ARCH_ESP32)
    Serial1.begin(kMksUartBaud, SERIAL_8N1, kMksUartRxPin, kMksUartTxPin);
#else
    Serial1.begin(kMksUartBaud);
#endif
    sendMksCommand(0.0f, 0.0f);
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
void hal_motor_left_move(float voltage) { pendingLeftCommand = voltage; }
void hal_motor_right_move(float voltage) {
    pendingRightCommand = -voltage;
    sendMksCommand(pendingLeftCommand, pendingRightCommand);
}
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
    Serial.println("\nBalanceBot ESP32 boot");

    analogReadResolution(12);
    pinMode(kBatteryAdcPin, INPUT);
    pinMode(kRcThrottlePin, INPUT);
    pinMode(kRcSteerPin, INPUT);
    pinMode(kPauseButtonPin, INPUT_PULLUP);

    BalanceBot_init(&botHal, &botConfiguration, &botState);
    botConfiguration.control_delta_time_seconds = kControlPeriodSeconds;

    initializeI2cBus();
    initializeImu();
    initializeMksUart();

    lastControlMicroseconds = micros();
    Serial.println("LOLIN32 Lite target ready. Sending torque commands to MKS over UART.");
}

void loop() {
    handlePauseToggleButton();

    if (pauseRequested) {
        sendMksCommand(0.0f, 0.0f);
        return;
    }

    const uint32_t nowMicroseconds = micros();
    const float deltaTime = (nowMicroseconds - lastControlMicroseconds) * 1e-6f;
    if (deltaTime < botConfiguration.control_delta_time_seconds) {
        return;
    }

    lastControlMicroseconds = nowMicroseconds;
    BalanceBot_update(&botConfiguration, &botState);
}

#endif // BALANCE_BOT_ESP32_INO