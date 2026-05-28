#include "../BalanceBot.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// --- Simulated hardware state ---
static double sim_time_ms = 0;
static double sim_pitch_deg = 8.0;
static double sim_gyro_dps = 0.0;
static double sim_battery_voltage = 16.0;
static double sim_left_motor = 0.0, sim_right_motor = 0.0;
static double sim_rc_throttle = 0.0, sim_rc_steer = 0.0;

// --- HAL implementation ---
uint32_t sim_millis(void) { return (uint32_t)sim_time_ms; }
uint32_t sim_micros(void) { return (uint32_t)(sim_time_ms * 1000.0); }
void sim_serial_print(const char* s) { printf("%s", s); }
void sim_serial_printf(const char* fmt, ...) { va_list args; va_start(args, fmt); vprintf(fmt, args); va_end(args); }
uint16_t sim_analog_read(int pin) { return 0; }
void sim_motor_left_move(float v) { sim_left_motor = v; }
void sim_motor_right_move(float v) { sim_right_motor = v; }
float sim_get_rc_throttle(void) { return (float)sim_rc_throttle; }
float sim_get_rc_steer(void) { return (float)sim_rc_steer; }
float sim_get_pitch_deg(void) { return (float)sim_pitch_deg; }
float sim_get_gyro_pitch_rate_dps(void) { return (float)sim_gyro_dps; }
float sim_get_battery_voltage(void) { return (float)sim_battery_voltage; }

struct BalanceBotHAL sim_hal = {
    .millis = sim_millis,
    .micros = sim_micros,
    .serial_print = sim_serial_print,
    .serial_printf = sim_serial_printf,
    .analog_read = sim_analog_read,
    .motor_left_move = sim_motor_left_move,
    .motor_right_move = sim_motor_right_move,
    .get_rc_throttle = sim_get_rc_throttle,
    .get_rc_steer = sim_get_rc_steer,
    .get_pitch_deg = sim_get_pitch_deg,
    .get_gyro_pitch_rate_dps = sim_get_gyro_pitch_rate_dps,
    .get_battery_voltage = sim_get_battery_voltage
};

int main() {
    struct BalanceBotConfig cfg;
    struct BalanceBotState state;
    BalanceBot_init(&sim_hal, &cfg, &state);
    double dt = 0.004;
    double duration = 20.0;
    int steps = (int)(duration / dt);
    FILE* csv = fopen("sim_stabilize.csv", "w");
    fprintf(csv, "t,pitch_deg,gyro_dps,left_motor,right_motor\n");
    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        // Simulate plant
        double theta_ddot = 1.8 * sim_pitch_deg - 3.8 * sim_gyro_dps + 18.0 * ((sim_left_motor + sim_right_motor) / 2.0);
        sim_gyro_dps += theta_ddot * dt;
        sim_pitch_deg += sim_gyro_dps * dt;
        sim_time_ms += dt * 1000.0;
        // Run control
        BalanceBot_update(&cfg, &state);
        fprintf(csv, "%f,%f,%f,%f,%f\n", t, sim_pitch_deg, sim_gyro_dps, sim_left_motor, sim_right_motor);
    }
    fclose(csv);
    printf("Simulation complete. Results in sim_stabilize.csv\n");
    return 0;
}
