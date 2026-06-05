#include "fake_arduino/Arduino.h"
#include <iostream>
#include <string>
#include <cstdio>
#include "../BalanceBotEsp32/BalanceBotEsp32.ino"
#include <cmath>
#include "../test_utils.h"

namespace {

void test_loop_updates_motor_outputs_from_inputs() {
    fake_arduino::reset();

    setup();
    const std::string initialMksUart = Serial1.buffer();

    // Simulate a forward-tilted robot. Controller should command non-zero torque.
    fake_arduino::set_mpu_angle_x(8.0f);
    fake_arduino::set_mpu_gyro_x(0.0f);

    fake_arduino::advance_millis(4);
    loop();

    const std::string mksUart = Serial1.buffer();
    expect_true(mksUart.size() > initialMksUart.size(), "loop should append a new MKS UART command");
    expect_contains(mksUart, "M ", "MKS UART command frame should start with M token");

    const std::size_t framePos = mksUart.rfind("M ");
    expect_true(framePos != std::string::npos, "MKS UART log should contain at least one frame");

    float left = 0.0f;
    float right = 0.0f;
    int parsed = std::sscanf(mksUart.c_str() + framePos, "M %f %f", &left, &right);
    expect_true(parsed == 2, "MKS UART frame should contain two float torque commands");
    expect_true(std::fabs(left) > 0.001f, "left UART torque command should be non-zero after control step");
    expect_true(std::fabs(right) > 0.001f, "right UART torque command should be non-zero after control step");
    expect_near(left, -right, 0.05f, "left/right UART command signs should oppose due to hal right inversion");
}

}  // namespace