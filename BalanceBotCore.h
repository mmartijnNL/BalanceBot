#pragma once
#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cmath>
#define constrain(x, a, b) (std::max((a), std::min((x), (b))))
#define fabsf std::fabs
#endif

struct BalanceBotConfiguration {
    float proportional_gain_angle = 22.0f;
    float integral_gain_angle = 0.0f;
    float derivative_gain_angle = 0.7f;
    float proportional_gain_rate = 0.12f;
    float integral_gain_rate = 0.8f;
    float derivative_gain_rate = 0.0008f;
    float driver_voltage_limit = 6.0f;
    float maximum_tilt_degrees = 25.0f;
};

struct BalanceBotState {
    float angle_integral = 0.0f;
    float previous_angle_error = 0.0f;
    float rate_integral = 0.0f;
    float previous_rate_error = 0.0f;
};

inline void resetBalanceBotState(BalanceBotState &state) {
    state.angle_integral = 0.0f;
    state.previous_angle_error = 0.0f;
    state.rate_integral = 0.0f;
    state.previous_rate_error = 0.0f;
}

inline float balanceBotControl(
    BalanceBotConfiguration &configuration,
    BalanceBotState &state,
    float pitch_degrees,
    float gyroscope_pitch_rate_degrees_per_second,
    float delta_time,
    float commanded_angle_degrees,
    float commanded_steering,
    float &left_command,
    float &right_command
) {
    float angle_error = commanded_angle_degrees - pitch_degrees;
    float delta_angle = (angle_error - state.previous_angle_error) / delta_time;
    state.angle_integral += angle_error * delta_time;
    state.angle_integral = constrain(state.angle_integral, -15.0f, 15.0f);
    state.previous_angle_error = angle_error;
    float target_rate_degrees_per_second = configuration.proportional_gain_angle * angle_error + configuration.integral_gain_angle * state.angle_integral + configuration.derivative_gain_angle * delta_angle;
    float rate_error = target_rate_degrees_per_second - gyroscope_pitch_rate_degrees_per_second;
    float delta_rate = (rate_error - state.previous_rate_error) / delta_time;
    state.rate_integral += rate_error * delta_time;
    state.rate_integral = constrain(state.rate_integral, -40.0f, 40.0f);
    state.previous_rate_error = rate_error;
    float base_command = configuration.proportional_gain_rate * rate_error + configuration.integral_gain_rate * state.rate_integral + configuration.derivative_gain_rate * delta_rate;
    base_command = constrain(base_command, -configuration.driver_voltage_limit, configuration.driver_voltage_limit);
    left_command = base_command - commanded_steering;
    right_command = base_command + commanded_steering;
    return base_command;
}
