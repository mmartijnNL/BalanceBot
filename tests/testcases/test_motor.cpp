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

    fake_arduino::set_mpu_angle_x(20.0f);
    fake_arduino::set_mpu_gyro_x(11.5f);
    rightMotor.shaft_velocity = 2.0f;

    loop();

    expect_true(leftMotor.loop_count() > 0, "left motor loopFOC should run");
    expect_true(rightMotor.loop_count() > 0, "right motor loopFOC should run");
    expect_true(std::fabs(rightMotor.last_move_command) > 0.001f, "right velocity target should be non-zero above dead-zone");

    // RC pins default to 0 after reset -> readRcChannel returns 0.0 -> effectivePitch == pitchRadians
    const float pitchRadians = 20.0f * 0.017453292519943295f;
    const float gyroRadiansPerSecond = 11.5f * 0.017453292519943295f;
    const float effectivePitch = pitchRadians;  // no RC throttle offset
    const float expectedFollowerVelocity = 10.0f * effectivePitch;  // no steer bias
    expect_near(rightMotor.last_move_command, expectedFollowerVelocity, 0.001f, "right velocity command should track pitch angle");

    const float expectedMasterTorque = (5.0f * ((2.0f / 10.0f) - effectivePitch)) - (0.08f * gyroRadiansPerSecond);
    expect_near(leftMotor.last_move_command, expectedMasterTorque, 0.001f, "left torque command should couple to right velocity, pitch angle, and pitch rate");
}

}  // namespace