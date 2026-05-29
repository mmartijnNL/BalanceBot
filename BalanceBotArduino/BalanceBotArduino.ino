#include "../BalanceBot.h"
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <MPU6050_light.h>
#include <stdarg.h>
#include <stdio.h>

// --- Hardware objects and HAL glue ---
// (Define your hardware objects here, e.g. motors, sensors, drivers)
TwoWire I2CBusRight = TwoWire(1);
MPU6050 mpu(Wire);
MagneticSensorI2C sensorLeft = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensorRight = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motorLeft = BLDCMotor(7); // MOTOR_POLE_PAIRS
BLDCMotor motorRight = BLDCMotor(7);
BLDCDriver3PWM driverLeft = BLDCDriver3PWM(25, 26, 27, 4);
BLDCDriver3PWM driverRight = BLDCDriver3PWM(14, 32, 33, 16);

// --- HAL implementation for ESP32/Arduino ---
uint32_t hal_millis(void) { return millis(); }
uint32_t hal_micros(void) { return micros(); }
void hal_serial_print(const char* s) { Serial.print(s); }
void hal_serial_printf(const char* fmt, ...) { char buf[128]; va_list args; va_start(args, fmt); vsnprintf(buf, sizeof(buf), fmt, args); va_end(args); Serial.print(buf); }
uint16_t hal_analog_read(int pin) { return analogRead(pin); }
void hal_motor_left_move(float v) { motorLeft.move(v); }
void hal_motor_right_move(float v) { motorRight.move(-v); }
float hal_get_rc_throttle(void) { return 0.0f; /* implement RC logic */ }
float hal_get_rc_steer(void) { return 0.0f; /* implement RC logic */ }
float hal_get_pitch_deg(void) { mpu.update(); return mpu.getAngleX(); }
float hal_get_gyro_pitch_rate_dps(void) { mpu.update(); return mpu.getGyroX(); }
float hal_get_battery_voltage(void) { uint16_t raw = analogRead(34); float adcV = (raw / 4095.0f) * 3.3f; return adcV * 5.0f; }

struct BalanceBotHAL bot_hal = {
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
    hal_get_battery_voltage
};
struct BalanceBotConfig botCfg;
struct BalanceBotState botState;
unsigned long lastControlUs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\nBalanceBot boot");
    pinMode(35, INPUT); // RC_THROTTLE_PIN
    pinMode(39, INPUT); // RC_STEER_PIN
    pinMode(34, INPUT); // BATTERY_ADC_PIN
    // TODO: attachInterrupts, setupSensorsAndMotors, etc.
    lastControlUs = micros();
    BalanceBot_init(&bot_hal, &botCfg, &botState);
    Serial.println("Type 'e' to enable balancing, 'D' to disable.");
}

void loop() {
    motorLeft.loopFOC();
    motorRight.loopFOC();
    unsigned long nowUs = micros();
    float dt = (nowUs - lastControlUs) * 1e-6f;
    if (dt < 0.004f) return;
    lastControlUs = nowUs;
    BalanceBot_update(&botCfg, &botState);
}
