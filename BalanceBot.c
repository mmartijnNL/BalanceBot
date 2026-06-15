// ...existing code...
#include "BalanceBot.h"
#include <stdarg.h>
#include <stdio.h>

static const struct BalanceBotHardwareAbstractionLayer* g_hardware_abstraction_layer = 0;

void BalanceBot_init(const struct BalanceBotHardwareAbstractionLayer* hardware_abstraction_layer, struct BalanceBotConfiguration* configuration, struct BalanceBotState* state) {
    g_hardware_abstraction_layer = hardware_abstraction_layer;
    // Set defaults if needed
    if (configuration) {
        configuration->proportional_gain_angle = 12.0f;
        configuration->integral_gain_angle = 0.0f;
        configuration->derivative_gain_angle = 0.35f;
        configuration->proportional_gain_rate = 0.09f;
        configuration->integral_gain_rate = 0.18f;
        configuration->derivative_gain_rate = 0.0004f;
        configuration->driver_voltage_limit = 6.0f;
        configuration->maximum_tilt_degrees = 25.0f;
        configuration->control_delta_time_seconds = 0.004f;
    }
    if (state) {
        state->angle_integral = 0.0f;
        state->previous_angle_error = 0.0f;
        state->rate_integral = 0.0f;
        state->previous_rate_error = 0.0f;
        state->rc_target_angle_degrees = 0.0f;
        state->rc_steering_command = 0.0f;
        state->rc_throttle_filtered = 0.0f;
        state->rc_steer_filtered = 0.0f;
        state->rc_signal_valid = false;
        state->balancing_enabled = false;
    }
}

static float constrain(float x, float a, float b) {
    return (x < a) ? a : (x > b) ? b : x;
}

void BalanceBot_update(struct BalanceBotConfiguration* configuration, struct BalanceBotState* state) {
    // RC smoothing for analog input channels.
    float throttle_normalized = g_hardware_abstraction_layer->get_rc_throttle();
    float steer_normalized = g_hardware_abstraction_layer->get_rc_steer();
    throttle_normalized = constrain(throttle_normalized, -1.0f, 1.0f);
    steer_normalized = constrain(steer_normalized, -1.0f, 1.0f);
    float delta_time = configuration->control_delta_time_seconds;
    float alpha_throttle = delta_time / (0.12f + delta_time);
    float alpha_steer = delta_time / (0.08f + delta_time);
    state->rc_throttle_filtered += alpha_throttle * (throttle_normalized - state->rc_throttle_filtered);
    state->rc_steer_filtered += alpha_steer * (steer_normalized - state->rc_steer_filtered);
    state->rc_target_angle_degrees = state->rc_throttle_filtered * 6.0f; // RC_MAX_TARGET_ANGLE_DEG
    state->rc_steering_command = state->rc_steer_filtered * 1.8f; // RC_MAX_STEER_CMD
    state->rc_signal_valid = true; // For simulation, always valid

    // Main control logic
    float pitch_degrees = g_hardware_abstraction_layer->get_pitch_degrees();
    float gyroscope_pitch_rate_degrees_per_second = g_hardware_abstraction_layer->get_gyroscope_pitch_rate_degrees_per_second();
    float commanded_angle_degrees = state->rc_target_angle_degrees;
    float commanded_steering = state->rc_steering_command;
    float angle_error = commanded_angle_degrees - pitch_degrees;
    float delta_angle = (angle_error - state->previous_angle_error) / configuration->control_delta_time_seconds;
    state->angle_integral += angle_error * configuration->control_delta_time_seconds;
    state->angle_integral = constrain(state->angle_integral, -15.0f, 15.0f);
    state->previous_angle_error = angle_error;
    float target_rate_degrees_per_second = configuration->proportional_gain_angle * angle_error + configuration->integral_gain_angle * state->angle_integral + configuration->derivative_gain_angle * delta_angle;
    float rate_error = target_rate_degrees_per_second - gyroscope_pitch_rate_degrees_per_second;
    float delta_rate = (rate_error - state->previous_rate_error) / configuration->control_delta_time_seconds;
    state->rate_integral += rate_error * configuration->control_delta_time_seconds;
    state->rate_integral = constrain(state->rate_integral, -40.0f, 40.0f);
    state->previous_rate_error = rate_error;
    float base_command = configuration->proportional_gain_rate * rate_error + configuration->integral_gain_rate * state->rate_integral + configuration->derivative_gain_rate * delta_rate;
    base_command = constrain(base_command, -configuration->driver_voltage_limit, configuration->driver_voltage_limit);
    float left_command = base_command - commanded_steering;
    float right_command = base_command + commanded_steering;
    // Output to motors
    g_hardware_abstraction_layer->motor_left_move(left_command);
    g_hardware_abstraction_layer->motor_right_move(right_command);
}
