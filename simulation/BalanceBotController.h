#pragma once

class BalanceBotController {
public:
    BalanceBotController();
    void reset();
    double step(double pitch_deg, double gyro_pitch_rate_dps, double dt, double rc_target_angle_deg, double rc_steering_cmd);
    // Add controller parameters as public members for easy tuning
    double max_tilt_deg;
    double driver_voltage_limit;
    double kp_angle, ki_angle, kd_angle;
    double kp_rate, ki_rate, kd_rate;
private:
    double angle_int, prev_angle_err;
    double rate_int, prev_rate_err;
};
