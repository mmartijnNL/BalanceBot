# INO Test Harness

This folder provides a native host test harness for Arduino sketch logic.

It compiles `BalanceBotLolin32lite.ino` as regular C++ against fake Arduino/MPU6050/Wire shims, then runs assertions against:
- simulated pin setup and reads
- motor move command outputs
- serial log text

## Run tests

From project root:

```bash
make -C tests run
```

## Add tests

Edit `tests/test_lolin32lite_ino.cpp` and add new test functions.

Useful helpers:
- `fake_arduino::set_analog(pin, value)` to inject ADC values
- `fake_arduino::set_digital_input(pin, value)` to inject digital inputs
- `fake_arduino::set_mpu_angle_x(value)` and `set_mpu_gyro_x(value)` to drive IMU values
- `fake_arduino::advance_millis(delta)` to advance virtual time and trigger periodic logic
- `fake_arduino::serial_log()` to assert on printed logs

The sketch globals are available directly in tests because the test file includes the `.ino` unit.
