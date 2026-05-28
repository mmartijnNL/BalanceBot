#include "BalanceBotController.h"
#include <fstream>
#include <iostream>
#include <cmath>

// Simple inverted pendulum plant
struct PlantConfig {
    double gravity_gain = 1.8;
    double damping = 3.8;
    double motor_gain = 18.0;
};

int main() {
    BalanceBotController ctrl;
    PlantConfig plant;
    double dt = 0.004;
    double duration = 20.0;
    int steps = static_cast<int>(duration / dt);
    double theta_deg = 8.0;
    double theta_dot_dps = 0.0;
    std::ofstream csv("sim_stabilize.csv");
    csv << "t,pitch_deg,gyro_dps,base_cmd\n";
    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        double sensed_angle = theta_deg;
        double sensed_rate = theta_dot_dps;
        double rc_target_angle_deg = 0.0;
        double rc_steering_cmd = 0.0;
        double base_cmd = ctrl.step(sensed_angle, sensed_rate, dt, rc_target_angle_deg, rc_steering_cmd);
        double theta_ddot = plant.gravity_gain * theta_deg - plant.damping * theta_dot_dps + plant.motor_gain * base_cmd;
        theta_dot_dps += theta_ddot * dt;
        theta_deg += theta_dot_dps * dt;
        csv << t << "," << theta_deg << "," << theta_dot_dps << "," << base_cmd << "\n";
    }
    csv.close();
    std::cout << "Simulation complete. Results in sim_stabilize.csv\n";
    return 0;
}
