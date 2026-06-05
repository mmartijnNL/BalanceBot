# BalanceBot (ESP32 + SimpleFOC)

This project now targets a direct motor-control architecture using the `SimpleFOC` library:
- ESP32 runs both FOC loops and high-level coupling logic
- Two AS5600 magnetic sensors provide shaft feedback over I2C
- Two 3PWM BLDC drivers are controlled directly from the ESP32

## Repository layout

- ESP32 firmware entry: `BalanceBotEsp32/BalanceBotEsp32.ino`
- Host simulation: `simulation/`
- Host test harness: `tests/`

## Control model

The ESP32 sketch uses a two-motor coupling loop:
- Left motor: `MotionControlType::torque`
- Right motor: `MotionControlType::velocity`
- Right target velocity follows left shaft angle with a dead-zone
- Left target torque follows right velocity and left angle feedback

## Default pin map

From `BalanceBotEsp32/BalanceBotEsp32.ino`:

- Left AS5600 I2C: SDA = GPIO19, SCL = GPIO18
- Right AS5600 I2C: SDA = GPIO23, SCL = GPIO5
- Left motor 3PWM: U = GPIO32, V = GPIO33, W = GPIO25, EN = GPIO22
- Right motor 3PWM: U = GPIO26, V = GPIO27, W = GPIO14, EN = GPIO12

## Wiring summary

- ESP32 common ground with both BLDC drivers and both AS5600 sensors
- Each motor phase connected to its matching 3PWM driver outputs
- Driver logic and sensor buses powered from stable regulator rails
- Keep I2C wires short and shielded where possible for encoder stability

## Build and upload (Arduino IDE)

1. Install an ESP32 Arduino core.
2. Install `SimpleFOC`.
3. Ensure `Wire` support for your board target is enabled.
3. Open `BalanceBotEsp32/BalanceBotEsp32.ino`.
4. Select your ESP32 board profile.
5. Select serial port and upload.

## Runtime behavior

- Both motors run `loopFOC()` continuously.
- Right motor velocity is coupled to left shaft angle.
- Left motor torque is coupled back from right shaft velocity and left angle.
- Serial monitor prints startup status.

## Host-side tests

From project root:

```bash
make -C tests run
```

Tests compile `BalanceBotEsp32.ino` against fake Arduino shims and verify:
- setup and motor/sensor initialization
- I2C bus and controller mode configuration
- control-loop coupling outputs
- dead-zone behavior around zero angle
