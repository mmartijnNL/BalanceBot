#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotPico/BalanceBotPico.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_loop_updates_motor_outputs_from_inputs() {
    fake_arduino::reset();

    setup();

    // Simulate a forward-tilted robot. Controller should command non-zero torque.
    fake_arduino::set_mpu_angle_x(8.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);

    fake_arduino::advance_millis(4);
    loop();

    expect_true(std::fabs(motorLeft.last_move_command) > 0.001f, "left motor command should change after control step");
    expect_true(std::fabs(motorRight.last_move_command) > 0.001f, "right motor command should change after control step");
    expect_near(motorLeft.last_move_command, -motorRight.last_move_command, 0.05f,
                "left/right command signs should oppose due to hal right inversion");
}

}  // namespace