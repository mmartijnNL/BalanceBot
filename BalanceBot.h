#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Hardware abstraction layer (HardwareAbstractionLayer) interface
struct BalanceBotHardwareAbstractionLayer {
    // Time
    uint32_t (*milliseconds)(void);
    uint32_t (*microseconds)(void);
    // Serial
    void (*serial_print)(const char*);
    void (*serial_printf)(const char*, ...);
    // AnalogToDigitalConverter
    uint16_t (*analog_read)(int pin);
    // PWM/Motor
    void (*motor_left_move)(float voltage);
    void (*motor_right_move)(float voltage);
    // RC
    float (*get_rc_throttle)(void);
    float (*get_rc_steer)(void);
    // InterIntegratedCircuit/Sensors
    float (*get_pitch_degrees)(void);
    float (*get_gyroscope_pitch_rate_degrees_per_second)(void);
    // Battery
    float (*get_battery_voltage)(void);
};

// Configuration and state
struct BalanceBotConfiguration {
    float proportional_gain_angle, integral_gain_angle, derivative_gain_angle;
    float proportional_gain_rate, integral_gain_rate, derivative_gain_rate;
    float driver_voltage_limit;
    float maximum_tilt_degrees;
    float control_delta_time_seconds;
};

struct BalanceBotState {
    // ProportionalIntegralDerivative state
    float angle_integral, previous_angle_error;
    float rate_integral, previous_rate_error;
    // RC
    float rc_target_angle_degrees, rc_steering_command;
    float rc_throttle_filtered, rc_steer_filtered;
    bool rc_signal_valid;
    // Battery
    float battery_voltage_filtered;
    bool low_voltage_cutoff_active;
    unsigned long low_voltage_cutoff_below_since_milliseconds;
    // Control
    bool balancing_enabled;
};

void BalanceBot_init(const struct BalanceBotHardwareAbstractionLayer* hardware_abstraction_layer, struct BalanceBotConfiguration* configuration, struct BalanceBotState* state);
void BalanceBot_update(struct BalanceBotConfiguration* configuration, struct BalanceBotState* state);

#ifdef __cplusplus
}
#endif
