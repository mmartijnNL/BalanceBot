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

    const float normalizedLeft = leftMotor.last_move_command * kLeftMotorDirection;
    const float normalizedRight = rightMotor.last_move_command * kRightMotorDirection;

    expect_true(leftMotor.loop_count() > 0, "left motor loopFOC should run");
    expect_true(rightMotor.loop_count() > 0, "right motor loopFOC should run");
    expect_true(std::fabs(rightMotor.last_move_command) > 0.001f, "right torque should be non-zero above dead-zone");
    expect_true(normalizedRight < 0.0f, "right torque should oppose positive forward tilt");
    expect_near(normalizedLeft, normalizedRight, 0.0001f,
        "left and right torques should match with neutral steering");
    expect_true(std::fabs(rightMotor.last_move_command) <= 0.0801f,
        "first loop torque should be limited by slew-rate clamp");
}

void test_torque_slew_limit_applies_between_loops() {
    fake_arduino::reset();
    setup();

    fake_arduino::set_mpu_angle_x(25.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);

    loop();
    const float firstRight = rightMotor.last_move_command * kRightMotorDirection;
    loop();
    const float secondRight = rightMotor.last_move_command * kRightMotorDirection;

    expect_true(std::fabs(secondRight - firstRight) <= 0.0801f,
        "per-loop torque change should stay within configured slew limit");
    expect_true(std::fabs(secondRight) >= std::fabs(firstRight),
        "torque magnitude should ramp toward target over consecutive loops");
}

void test_periodic_telemetry_is_printed() {
    fake_arduino::reset();
    setup();

    fake_arduino::set_mpu_angle_x(10.0f);
    fake_arduino::set_mpu_gyro_x(2.0f);

    loop();
    fake_arduino::advance_millis(260);
    loop();

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "CTRL=ON", "telemetry should include control state");
    expect_contains(logs, "cmdL=", "telemetry should include left torque command");
    expect_contains(logs, "cmdR=", "telemetry should include right torque command");
}

void test_large_tilt_still_commands_torque_when_always_armed() {
    fake_arduino::reset();
    setup();

    fake_arduino::set_mpu_angle_x(55.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);
    loop();

    expect_true(std::fabs(rightMotor.last_move_command) > 0.001f,
        "controller should keep commanding torque even at high tilt in always-armed mode");

    const std::string logs = fake_arduino::serial_log();
    expect_true(logs.find("CTRL=OFF") == std::string::npos,
        "always-armed mode should not emit tilt safety cutoff logs");
}

}  // namespace