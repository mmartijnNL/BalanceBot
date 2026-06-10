#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotLolin32lite/BalanceBotLolin32lite.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_loop_updates_motor_outputs_from_inputs() {
    fake_arduino::reset();

    setup();

    leftMotor.shaft_angle = 0.35f;
    rightMotor.shaft_velocity = 2.0f;

    loop();

    expect_true(leftMotor.loop_count() > 0, "left motor loopFOC should run");
    expect_true(rightMotor.loop_count() > 0, "right motor loopFOC should run");
    expect_true(std::fabs(rightMotor.last_move_command) > 0.001f, "right velocity target should be non-zero above dead-zone");

    const float expectedFollowerVelocity = 10.0f * 0.35f;
    expect_near(rightMotor.last_move_command, expectedFollowerVelocity, 0.001f, "right velocity command should track left shaft angle");

    const float expectedMasterTorque = 5.0f * ((2.0f / 10.0f) - 0.35f);
    expect_near(leftMotor.last_move_command, expectedMasterTorque, 0.001f, "left torque command should couple to right velocity and left angle");
}

}  // namespace