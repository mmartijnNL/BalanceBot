#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Main entry points for Arduino .ino
void BalanceBot_main_setup(void);
void BalanceBot_main_loop(void);

// Hardware abstraction layer (HAL) interface
struct BalanceBotHAL {
    // Time
    uint32_t (*millis)(void);
    uint32_t (*micros)(void);
    // Serial
    void (*serial_print)(const char*);
    void (*serial_printf)(const char*, ...);
    // ADC
    uint16_t (*analog_read)(int pin);
    // PWM/Motor
    void (*motor_left_move)(float voltage);
    void (*motor_right_move)(float voltage);
    // RC
    float (*get_rc_throttle)(void);
    float (*get_rc_steer)(void);
    // I2C/Sensors
    float (*get_pitch_deg)(void);
    float (*get_gyro_pitch_rate_dps)(void);
    // Battery
    float (*get_battery_voltage)(void);
};

// Configuration and state
struct BalanceBotConfig {
    float kp_angle, ki_angle, kd_angle;
    float kp_rate, ki_rate, kd_rate;
    float driver_voltage_limit;
    float max_tilt_deg;
    float control_dt_s;
};

struct BalanceBotState {
    // PID state
    float angleInt, prevAngleErr;
    float rateInt, prevRateErr;
    // RC
    float rcTargetAngleDeg, rcSteeringCmd;
    bool rcSignalValid;
    // Battery
    float batteryVoltageFiltered;
    bool lvcActive;
    unsigned long lvcBelowSinceMs;
    // Control
    bool balancingEnabled;
};

void BalanceBot_init(const struct BalanceBotHAL* hal, struct BalanceBotConfig* cfg, struct BalanceBotState* state);
void BalanceBot_update(struct BalanceBotConfig* cfg, struct BalanceBotState* state);

#ifdef __cplusplus
}
#endif
