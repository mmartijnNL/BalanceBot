#include "BalanceBotController.h"
#include <algorithm>
#include <cmath>

BalanceBotController::BalanceBotController() {
    max_tilt_deg = 25.0;
    driver_voltage_limit = 6.0;
    kp_angle = 22.0; ki_angle = 0.0; kd_angle = 0.7;
    kp_rate = 0.12; ki_rate = 0.8; kd_rate = 0.0008;
    reset();
}

void BalanceBotController::reset() {
    angle_int = 0.0; prev_angle_err = 0.0;
    rate_int = 0.0; prev_rate_err = 0.0;
}

double BalanceBotController::step(double pitch_deg, double gyro_pitch_rate_dps, double dt, double rc_target_angle_deg, double rc_steering_cmd) {
    if (fabs(pitch_deg) > max_tilt_deg) {
        reset();
        return 0.0;
    }
    double angle_err = rc_target_angle_deg - pitch_deg;
    double d_angle = (angle_err - prev_angle_err) / dt;
    angle_int += angle_err * dt;
    angle_int = std::clamp(angle_int, -15.0, 15.0);
    prev_angle_err = angle_err;
    double target_rate_dps = kp_angle * angle_err + ki_angle * angle_int + kd_angle * d_angle;
    double rate_err = target_rate_dps - gyro_pitch_rate_dps;
    double d_rate = (rate_err - prev_rate_err) / dt;
    rate_int += rate_err * dt;
    rate_int = std::clamp(rate_int, -40.0, 40.0);
    prev_rate_err = rate_err;
    double base_cmd = kp_rate * rate_err + ki_rate * rate_int + kd_rate * d_rate;
    base_cmd = std::clamp(base_cmd, -driver_voltage_limit, driver_voltage_limit);
    return base_cmd;
}
