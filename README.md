# BalanceBot (LOLIN32 Lite + MKS DUAL FOC v3.2 Plus)

This project now targets a single controller platform:
- LOLIN32 Lite (ESP32) runs the balance controller logic
- MKS DUAL FOC v3.2 Plus handles the dual motor power stage
- MPU6050 provides robot pitch and pitch-rate feedback over I2C

## Repository layout

- Shared controller core: `BalanceBot.h`, `BalanceBot.c`
- ESP32 firmware entry: `BalanceBotEsp32/BalanceBotEsp32.ino`
- Host simulation: `simulation/`
- Host test harness: `tests/`

## Control split

- LOLIN32 Lite responsibilities:
  - Read MPU6050 over I2C
  - Read battery voltage through ADC divider
  - Run balance PID update (shared core)
  - Send left/right torque command frames to MKS over UART
- MKS responsibilities:
  - Consume UART command frames
  - Apply torque/voltage command to the motor stage

## UART command contract

The ESP32 emits one text frame per control step:

```text
M <left_cmd> <right_cmd>
```

Example:

```text
M -1.25 1.25
```

If your MKS-side parser expects a different protocol, adapt either the parser or `sendMksCommand()` in the sketch.

## Default LOLIN32 Lite pin map

From `BalanceBotEsp32/BalanceBotEsp32.ino`:

- IMU I2C:
  - SDA = GPIO21
  - SCL = GPIO22
- UART to MKS:
  - TX = GPIO17
  - RX = GPIO16
- Battery ADC:
  - GPIO34
- RC inputs (currently stubs in firmware):
  - Throttle = GPIO36
  - Steering = GPIO39
- Pause button:
  - GPIO0 (BOOT button, active-low)

## Wiring summary

- Battery -> MKS power stage
- Battery -> step-down (MP1584EN) -> LOLIN32 Lite 5V/3.3V rail
- LOLIN32 Lite UART TX/RX <-> MKS UART RX/TX
- LOLIN32 Lite I2C <-> MPU6050
- Battery divider output -> LOLIN32 Lite ADC input
- Common GND between LOLIN32 Lite, MKS, MPU6050, and battery system

## Build and upload (Arduino IDE)

1. Install an ESP32 Arduino core.
2. Install `MPU6050_light`.
3. Open `BalanceBotEsp32/BalanceBotEsp32.ino`.
4. Select LOLIN32 Lite (or equivalent ESP32 board profile).
5. Select serial port and upload.

## Runtime behavior

- Balance loop period defaults to 4 ms.
- BOOT button toggles pause/resume.
- While paused, the firmware continuously sends zero motor commands.
- Low-voltage cutoff logic runs in shared core and logs status over USB serial.

## Host-side tests

From project root:

```bash
make -C tests run
```

Tests compile `BalanceBotEsp32.ino` against fake Arduino shims and verify:
- setup and pin configuration
- MPU6050 init failure logging
- UART motor command frames
- low-voltage cutoff logging
