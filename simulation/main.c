#include "../BalanceBot.h"
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// --- Simulated hardware and plant state ---
static double sim_time_s = 0.0;
static double sim_time_ms = 0.0;
static double sim_pitch_deg = 10.0;
static double sim_gyro_dps = 0.0;
static double sim_pitch_noise_deg = 0.0;
static double sim_gyro_noise_dps = 0.0;
static double sim_battery_voltage = 16.8;
static double sim_left_motor_cmd_v = 0.0;
static double sim_right_motor_cmd_v = 0.0;
static double sim_left_motor_actual_v = 0.0;
static double sim_right_motor_actual_v = 0.0;
static double sim_rc_throttle = 0.0;
static double sim_rc_steer = 0.0;

// --- HAL implementation for BalanceBot.c ---
uint32_t sim_millis(void) { return (uint32_t)sim_time_ms; }
uint32_t sim_micros(void) { return (uint32_t)(sim_time_s * 1000000.0); }
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
void sim_motor_left_move(float v) { sim_left_motor_cmd_v = (double)v; }
void sim_motor_right_move(float v) { sim_right_motor_cmd_v = (double)v; }
float sim_get_rc_throttle(void) { return (float)sim_rc_throttle; }
float sim_get_rc_steer(void) { return (float)sim_rc_steer; }
float sim_get_pitch_deg(void) { return (float)(sim_pitch_deg + sim_pitch_noise_deg); }
float sim_get_gyro_pitch_rate_dps(void) { return (float)(sim_gyro_dps + sim_gyro_noise_dps); }
float sim_get_battery_voltage(void) { return (float)sim_battery_voltage; }

struct BalanceBotHAL sim_hal = {
    sim_millis,
    sim_micros,
    sim_serial_print,
    sim_serial_printf,
    sim_analog_read,
    sim_motor_left_move,
    sim_motor_right_move,
    sim_get_rc_throttle,
    sim_get_rc_steer,
    sim_get_pitch_deg,
    sim_get_gyro_pitch_rate_dps,
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

static double sample_smooth_profile(double t, const double* times, const double* values, int count) {
    if (t <= times[0]) return values[0];
    if (t >= times[count - 1]) return values[count - 1];

    for (int i = 0; i < count - 1; ++i) {
        if (t < times[i + 1]) {
            double seg_dt = times[i + 1] - times[i];
            if (seg_dt <= 0.0) return values[i + 1];
            double u = (t - times[i]) / seg_dt;
            double s = smoothstep01(u);
            return values[i] + (values[i + 1] - values[i]) * s;
        }
    }

    return values[count - 1];
}

// Time-varying analog RC profile (continuous and smooth, no jumps).
static void update_rc_profile(double t) {
    static const double times[] = {0.0, 3.0, 8.0, 12.0, 18.0, 30.0};
    static const double throttle_vals[] = {0.0, 0.0, 0.20, -0.12, 0.05, 0.0};
    static const double steer_vals[] = {0.0, 0.0, 0.0, 0.25, -0.30, 0.0};
    const int n = (int)(sizeof(times) / sizeof(times[0]));

    sim_rc_throttle = sample_smooth_profile(t, times, throttle_vals, n);
    sim_rc_steer = sample_smooth_profile(t, times, steer_vals, n);
}

static void update_battery_model(double dt) {
    double effort = fabs(sim_left_motor_actual_v) + fabs(sim_right_motor_actual_v);
    // Slowly decay battery with load; enough to exercise LVC logic in a long run.
    sim_battery_voltage -= (0.002 + 0.001 * effort) * dt;
    if (sim_battery_voltage < 12.0) sim_battery_voltage = 12.0;
}

static void apply_motor_driver_dynamics(double dt) {
    const double motor_tau_s = 0.030;
    double alpha = dt / (motor_tau_s + dt);
    sim_left_motor_cmd_v = clampd(sim_left_motor_cmd_v, -6.0, 6.0);
    sim_right_motor_cmd_v = clampd(sim_right_motor_cmd_v, -6.0, 6.0);
    sim_left_motor_actual_v += alpha * (sim_left_motor_cmd_v - sim_left_motor_actual_v);
    sim_right_motor_actual_v += alpha * (sim_right_motor_cmd_v - sim_right_motor_actual_v);
}

static void update_sensor_noise(double t) {
    // Deterministic pseudo-noise so runs are reproducible.
    sim_pitch_noise_deg = 0.08 * sin(2.0 * 3.141592653589793 * 13.0 * t);
    sim_gyro_noise_dps = 0.25 * sin(2.0 * 3.141592653589793 * 17.0 * t + 0.7);
}

static void update_plant(double dt, double t) {
    // Basic single-axis inverted pendulum-like model in degree units.
    const double gravity_gain = 1.8;
    const double damping = 3.8;
    const double motor_gain = 18.0;

    double avg_motor_v = 0.5 * (sim_left_motor_actual_v + sim_right_motor_actual_v);
    double disturbance = 0.0;
    if (t > 9.0 && t < 9.3) {
        disturbance = 22.0; // push disturbance pulse
    }
    double theta_ddot = gravity_gain * sim_pitch_deg - damping * sim_gyro_dps + motor_gain * avg_motor_v + disturbance;
    sim_gyro_dps += theta_ddot * dt;
    sim_pitch_deg += sim_gyro_dps * dt;
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
        double t = i * dt;
        sim_time_s = t;
        sim_time_ms = t * 1000.0;

        update_rc_profile(t);
        update_sensor_noise(t);

        BalanceBot_update(&cfg, &state);

        apply_motor_driver_dynamics(dt);
        update_plant(dt, t);
        update_battery_model(dt);

        // Optional autotune (simple gradient descent on squared error)
        float error = (state.rcTargetAngleDeg - sim_pitch_deg) * (state.rcTargetAngleDeg - sim_pitch_deg);
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
            t,
            sim_pitch_deg,
            sim_gyro_dps,
            sim_pitch_deg + sim_pitch_noise_deg,
            sim_gyro_dps + sim_gyro_noise_dps,
            sim_left_motor_cmd_v,
            sim_right_motor_cmd_v,
            sim_left_motor_actual_v,
            sim_right_motor_actual_v,
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
