#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotLolin32lite/BalanceBotLolin32lite.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_low_battery_triggers_cutoff_message() {
    fake_arduino::reset();
    setup();

    fake_arduino::set_mpu_angle_x(2.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);
    rightMotor.shaft_velocity = 0.0f;

    loop();

    expect_near(rightMotor.last_move_command, 0.0f, 0.0001f, "right command should be zero inside dead-zone");
    expect_near(leftMotor.last_move_command, 0.0f, 0.25f, "left torque should remain close to zero when follower velocity is zero and angle is in dead-zone");
}

}  // namespace