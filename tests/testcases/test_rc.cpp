#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include "../BalanceBotLolin32lite/BalanceBotLolin32lite.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

// RC pulse 1700us on throttle (+0.4), 1750us on steer (+0.5).
// Pitch is flat (0 degrees) so pitch error after throttle offset is small
// and falls inside the dead-zone; right motor output should equal steer bias only.
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

    // effectivePitch = 0 - (0.4 * 0.15) = -0.06 rad, |0.06| < kDeadZoneRadians(0.2) -> 0
    // followerVelocityTarget = 0 + 0.5 * 2.0 = 1.0
    const float expectedRight = 0.5f * 2.0f;
    expect_near(rightMotor.last_move_command, expectedRight, 0.001f,
        "right motor velocity should equal RC steer bias when pitch is in dead-zone");
}

// RC throttle forward: robot should lean (negative effectivePitch) which drives
// left torque positive to push forward.
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

    // effectivePitch = 0 - (1.0 * 0.15) = -0.15 rad
    // masterTorqueTarget = 5.0 * (0/10 - (-0.15)) - 0 = 0.75
    const float effectivePitch = -(1.0f * 0.15f);
    const float expectedLeft = 5.0f * ((0.0f / 10.0f) - effectivePitch);
    expect_near(leftMotor.last_move_command, expectedLeft, 0.001f,
        "left torque should increase when RC throttle leans the setpoint forward");
}

// No RC signal (pulseIn returns 0) -> channels default to 0.0 -> robot behaviour
// is identical to RC-off state (same as the baseline motor test).
void test_rc_no_signal_is_neutral() {
    fake_arduino::reset();
    setup();

    fake_arduino::set_mpu_angle_x(20.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);
    rightMotor.shaft_velocity = 0.0f;
    // pulses left at 0 (default after reset) -> readRcChannel returns 0.0

    loop();

    const float pitchRadians = 20.0f * 0.017453292519943295f;
    const float expectedRight = 10.0f * pitchRadians;  // no steer bias, no throttle offset
    expect_near(rightMotor.last_move_command, expectedRight, 0.001f,
        "with no RC signal right motor should track pitch angle as if RC is neutral");
}

}  // namespace
