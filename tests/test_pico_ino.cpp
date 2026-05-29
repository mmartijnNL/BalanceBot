#include "fake_arduino/Arduino.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

// Compile the sketch in a host process using the fake Arduino layer.
#include "../BalanceBotPico/BalanceBotPico.ino"

namespace {

int g_failures = 0;

void expect_true(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        g_failures++;
    }
}

void expect_contains(const std::string& haystack, const std::string& needle, const char* message) {
    expect_true(haystack.find(needle) != std::string::npos, message);
}

void expect_near(float actual, float expected, float tolerance, const char* message) {
    if (std::fabs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message << " (actual=" << actual << ", expected=" << expected << ")\n";
        g_failures++;
    }
}

void reset_harness_state() {
    fake_arduino::reset();
    fake_arduino::set_analog(26, 4095);
    fake_arduino::set_mpu_begin_status(0);
    fake_arduino::set_mpu_angle_x(0.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);
}

void test_setup_configures_pins_and_logs() {
    reset_harness_state();

    setup();

    expect_true(fake_arduino::get_analog_resolution_bits() == 12, "setup should configure 12-bit ADC");
    expect_true(fake_arduino::get_pin_mode(26) == INPUT, "battery ADC pin should be INPUT");
    expect_true(fake_arduino::get_pin_mode(20) == INPUT_PULLDOWN, "throttle pin should be INPUT_PULLDOWN");
    expect_true(fake_arduino::get_pin_mode(21) == INPUT_PULLDOWN, "steer pin should be INPUT_PULLDOWN");

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "BalanceBot Pico boot", "boot banner should be printed");
    expect_contains(logs, "Pico target ready", "ready message should be printed");
}

void test_imu_init_failure_is_logged() {
    reset_harness_state();
    fake_arduino::set_mpu_begin_status(7);

    setup();

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "MPU6050 init failed: 7", "imu init failure code should be logged");
}

void test_loop_updates_motor_outputs_from_inputs() {
    reset_harness_state();

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

void test_low_battery_triggers_cutoff_message() {
    reset_harness_state();
    setup();

    // Keep battery below cutoff long enough for trip delay.
    fake_arduino::set_analog(26, 0);
    fake_arduino::set_mpu_angle_x(0.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);

    for (int i = 0; i < 140; ++i) {
        fake_arduino::advance_millis(4);
        loop();
    }

    const std::string logs = fake_arduino::serial_log();
    expect_contains(logs, "LOW VOLTAGE CUTOFF ACTIVE", "lvc activation should be printed to Serial");
}

}  // namespace

int main() {
    test_setup_configures_pins_and_logs();
    test_imu_init_failure_is_logged();
    test_loop_updates_motor_outputs_from_inputs();
    test_low_battery_triggers_cutoff_message();

    if (g_failures == 0) {
        std::cout << "All harness tests passed.\n";
        return 0;
    }

    std::cerr << g_failures << " test(s) failed.\n";
    return 1;
}
