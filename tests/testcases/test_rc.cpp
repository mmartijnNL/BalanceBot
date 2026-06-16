#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotLolin32lite/BalanceBotLolin32lite.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

// RC pulse 1700us on throttle (+0.4), 1750us on steer (+0.5).
// With positive steer, right command should be greater than left command.
void test_rc_steer_shifts_right_motor_velocity() {
    fake_arduino::reset();
    setup();

    fake_arduino::set_mpu_angle_x(0.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);
    rightMotor.shaft_velocity = 0.0f;

    // throttle: 1700us -> (1700-1500)/500 = +0.4
    // steer:    1750us -> (1750-1500)/500 = +0.5
    fake_arduino::set_pin_pulse(34, 1700UL);
    fake_arduino::set_pin_pulse(35, 1750UL);

    loop();

    const float normalizedLeft = leftMotor.last_move_command * kLeftMotorDirection;
    const float normalizedRight = rightMotor.last_move_command * kRightMotorDirection;

    expect_true(normalizedRight > normalizedLeft,
        "positive steer should increase right torque relative to left torque");
    expect_true(std::fabs(rightMotor.last_move_command) <= 0.0801f,
        "slew limiting should constrain first-loop right torque step");
}

// RC throttle forward should bias balance torque positive to drive forward.
void test_rc_throttle_shifts_left_torque() {
    fake_arduino::reset();
    setup();

    fake_arduino::set_mpu_angle_x(0.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);
    rightMotor.shaft_velocity = 0.0f;

    // throttle full forward: 2000us -> +1.0
    fake_arduino::set_pin_pulse(34, 2000UL);
    fake_arduino::set_pin_pulse(35, 1500UL);  // steer neutral

    loop();

    const float normalizedLeft = leftMotor.last_move_command * kLeftMotorDirection;
    const float normalizedRight = rightMotor.last_move_command * kRightMotorDirection;

    expect_true(normalizedLeft > 0.0f,
        "left torque should be positive with forward throttle request");
    expect_true(normalizedRight > 0.0f,
        "right torque should be positive with forward throttle request");
    expect_near(normalizedLeft, normalizedRight, 0.0001f,
        "with neutral steer both sides should receive similar torque");
}

// No RC signal (pulseIn returns 0) should map to neutral RC channels.
void test_rc_no_signal_is_neutral() {
    fake_arduino::reset();
    setup();

    fake_arduino::set_mpu_angle_x(20.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);
    rightMotor.shaft_velocity = 0.0f;
    // pulses left at 0 (default after reset) -> readRcChannel returns 0.0

    loop();

    const float normalizedLeft = leftMotor.last_move_command * kLeftMotorDirection;
    const float normalizedRight = rightMotor.last_move_command * kRightMotorDirection;

    expect_true(normalizedRight < 0.0f,
        "with neutral RC and positive tilt, right torque should oppose tilt");
    expect_near(normalizedLeft, normalizedRight, 0.0001f,
        "neutral RC should not introduce steering asymmetry");
}

}  // namespace
