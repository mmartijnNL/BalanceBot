#include "../BalanceBot.h"
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// --- Simulated hardware and plant state ---
static double sim_time_s = 0.0;
static double sim_time_ms = 0.0;
static double sim_pitch_degrees = 10.0;
static double sim_gyroscope_degrees_per_second = 0.0;
static double sim_pitch_noise_degrees = 0.0;
static double sim_gyroscope_noise_degrees_per_second = 0.0;
static double sim_battery_voltage = 16.8;
static double sim_left_motor_command_voltage = 0.0;
static double sim_right_motor_command_voltage = 0.0;
static double sim_left_motor_actual_voltage = 0.0;
static double sim_right_motor_actual_voltage = 0.0;
static double sim_rc_throttle = 0.0;
static double sim_rc_steer = 0.0;

// --- HAL implementation for BalanceBot.c ---
uint32_t sim_milliseconds(void) { return (uint32_t)sim_time_ms; }
uint32_t sim_microseconds(void) { return (uint32_t)(sim_time_s * 1000000.0); }
void sim_serial_print(const char* s) { printf("%s", s); }
void sim_serial_printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}
uint16_t sim_analog_read(int pin) {
    (void)pin;
    return 0;
}
void sim_motor_left_move(float voltage) { sim_left_motor_command_voltage = (double)voltage; }
void sim_motor_right_move(float voltage) { sim_right_motor_command_voltage = (double)voltage; }
float sim_get_rc_throttle(void) { return (float)sim_rc_throttle; }
float sim_get_rc_steer(void) { return (float)sim_rc_steer; }
float sim_get_pitch_degrees(void) { return (float)(sim_pitch_degrees + sim_pitch_noise_degrees); }
float sim_get_gyroscope_pitch_rate_degrees_per_second(void) { return (float)(sim_gyroscope_degrees_per_second + sim_gyroscope_noise_degrees_per_second); }
float sim_get_battery_voltage(void) { return (float)sim_battery_voltage; }

struct BalanceBotHAL sim_hal = {
    sim_milliseconds,
    sim_microseconds,
    sim_serial_print,
    sim_serial_printf,
    sim_analog_read,
    sim_motor_left_move,
    sim_motor_right_move,
    sim_get_rc_throttle,
    sim_get_rc_steer,
    sim_get_pitch_degrees,
    sim_get_gyroscope_pitch_rate_degrees_per_second,
    sim_get_battery_voltage
};

static double clampd(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static double smoothstep01(double x) {
    x = clampd(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

static double sample_smooth_profile(double time, const double* times, const double* values, int count) {
    if (time <= times[0]) return values[0];
    if (time >= times[count - 1]) return values[count - 1];

    for (int i = 0; i < count - 1; ++i) {
        if (time < times[i + 1]) {
            double segment_delta_time = times[i + 1] - times[i];
            if (segment_delta_time <= 0.0) return values[i + 1];
            double u = (time - times[i]) / segment_delta_time;
            double s = smoothstep01(u);
            return values[i] + (values[i + 1] - values[i]) * s;
        }
    }

    return values[count - 1];
}

// Time-varying analog RC profile (continuous and smooth, no jumps).
static void update_rc_profile(double time) {
    static const double times[] = {0.0, 3.0, 8.0, 12.0, 18.0, 30.0};
    static const double throttle_vals[] = {0.0, 0.0, 0.20, -0.12, 0.05, 0.0};
    static const double steer_vals[] = {0.0, 0.0, 0.0, 0.25, -0.30, 0.0};
    const int n = (int)(sizeof(times) / sizeof(times[0]));

    sim_rc_throttle = sample_smooth_profile(time, times, throttle_vals, n);
    sim_rc_steer = sample_smooth_profile(time, times, steer_vals, n);
}

static void update_battery_model(double delta_time) {
    double effort = fabs(sim_left_motor_actual_voltage) + fabs(sim_right_motor_actual_voltage);
    // Slowly decay battery with load; enough to exercise low voltage cutoff logic in a long run.
    sim_battery_voltage -= (0.002 + 0.001 * effort) * delta_time;
    if (sim_battery_voltage < 12.0) sim_battery_voltage = 12.0;
}

static void apply_motor_driver_dynamics(double delta_time) {
    const double motor_time_constant_seconds = 0.030;
    double alpha = delta_time / (motor_time_constant_seconds + delta_time);
    sim_left_motor_command_voltage = clampd(sim_left_motor_command_voltage, -6.0, 6.0);
    sim_right_motor_command_voltage = clampd(sim_right_motor_command_voltage, -6.0, 6.0);
    sim_left_motor_actual_voltage += alpha * (sim_left_motor_command_voltage - sim_left_motor_actual_voltage);
    sim_right_motor_actual_voltage += alpha * (sim_right_motor_command_voltage - sim_right_motor_actual_voltage);
}

static void update_sensor_noise(double time) {
    // Deterministic pseudo-noise so runs are reproducible.
    sim_pitch_noise_degrees = 0.08 * sin(2.0 * 3.141592653589793 * 13.0 * time);
    sim_gyroscope_noise_degrees_per_second = 0.25 * sin(2.0 * 3.141592653589793 * 17.0 * time + 0.7);
}

static void update_plant(double delta_time, double time) {
    // Basic single-axis inverted pendulum-like model in degree units.
    const double gravity_gain = 1.8;
    const double damping = 3.8;
    const double motor_gain = 18.0;

    double average_motor_voltage = 0.5 * (sim_left_motor_actual_voltage + sim_right_motor_actual_voltage);
    double disturbance = 0.0;
    if (time > 9.0 && time < 9.3) {
        disturbance = 22.0; // push disturbance pulse
    }
    double theta_double_dot = gravity_gain * sim_pitch_degrees - damping * sim_gyroscope_degrees_per_second + motor_gain * average_motor_voltage + disturbance;
    sim_gyroscope_degrees_per_second += theta_double_dot * delta_time;
    sim_pitch_degrees += sim_gyroscope_degrees_per_second * delta_time;
}

int main(void) {
    struct BalanceBotConfig cfg;
    struct BalanceBotState state;
    BalanceBot_init(&sim_hal, &cfg, &state);

    const double dt = 0.004;
    const double duration = 60.0;
    const int steps = (int)(duration / dt);

    FILE* csv = fopen("sim_stabilize.csv", "w");
    if (!csv) {
        fprintf(stderr, "Failed to open sim_stabilize.csv\n");
        return 1;
    }

        fprintf(csv,
            "t,pitch_deg,gyro_dps,pitch_meas_deg,gyro_meas_dps,left_cmd_v,right_cmd_v,left_actual_v,right_actual_v,batt_v,rc_thr,rc_str,target_angle_deg,steering_cmd,lvc_active,kp_angle,ki_angle,kd_angle,kp_rate,ki_rate,kd_rate\n");

    for (int i = 0; i <= steps; ++i) {
        double time = i * dt;
        sim_time_s = time;
        sim_time_ms = time * 1000.0;

        update_rc_profile(time);
        update_sensor_noise(time);

        BalanceBot_update(&cfg, &state);

        apply_motor_driver_dynamics(dt);
        update_plant(dt, time);
        update_battery_model(dt);

        // Optional autotune (simple gradient descent on squared error)
        float error = (state.rcTargetAngleDeg - sim_pitch_degrees) * (state.rcTargetAngleDeg - sim_pitch_degrees);
        static float prev_error = 0.0f;
        static bool autotune_enabled = true; // Set to false to disable autotune
        if (autotune_enabled) {
            float d_error = error - prev_error;
            float lr_p = 1e-4f, lr_i = 1e-6f, lr_d = 1e-6f;
            // Angle loop autotune
            cfg.kp_angle -= lr_p * d_error;
            cfg.ki_angle -= lr_i * d_error;
            cfg.kd_angle -= lr_d * d_error;
            // Clamp
            if (cfg.kp_angle < 0.0f) cfg.kp_angle = 0.0f;
            if (cfg.kp_angle > 100.0f) cfg.kp_angle = 100.0f;
            if (cfg.ki_angle < 0.0f) cfg.ki_angle = 0.0f;
            if (cfg.ki_angle > 10.0f) cfg.ki_angle = 10.0f;
            if (cfg.kd_angle < 0.0f) cfg.kd_angle = 0.0f;
            if (cfg.kd_angle > 10.0f) cfg.kd_angle = 10.0f;
            prev_error = error;
        }
        fprintf(csv,
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            time,
            sim_pitch_degrees,
            sim_gyroscope_degrees_per_second,
            sim_pitch_degrees + sim_pitch_noise_degrees,
            sim_gyroscope_degrees_per_second + sim_gyroscope_noise_degrees_per_second,
            sim_left_motor_command_voltage,
            sim_right_motor_command_voltage,
            sim_left_motor_actual_voltage,
            sim_right_motor_actual_voltage,
            sim_battery_voltage,
            sim_rc_throttle,
            sim_rc_steer,
            state.rcTargetAngleDeg,
            state.rcSteeringCmd,
            state.lvcActive ? 1 : 0,
            cfg.kp_angle, cfg.ki_angle, cfg.kd_angle, cfg.kp_rate, cfg.ki_rate, cfg.kd_rate);
    }

    fclose(csv);
    printf("Simulation complete. Results in sim_stabilize.csv\n");
    return 0;
}
