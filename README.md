# BalanceBot (Pico + MKS DUAL FOC v3.2 Plus over UART)

This project is now simplified to a single firmware target:
- Raspberry Pi Pico runs balance control logic
- MKS DUAL FOC v3.2 Plus runs the motor/encoder loop
- Pico sends left/right torque commands to MKS over UART

No ESP32 target is used.
No direct motor or encoder control is performed in Pico firmware.

## What stays in this repo

- Shared controller core: `BalanceBot.h`, `BalanceBot.c`
- Pico firmware entry: `BalanceBotPico/BalanceBotPico.ino`
- Host simulation and tests

## Control split

- Pico responsibilities:
  - Read IMU (MPU6050)
  - Read battery voltage
  - Run balance PID (shared core)
  - Send motor commands over UART
- MKS responsibilities:
  - Run SimpleFOC motor control
  - Read motor encoders
  - Apply torque/velocity command from UART

## UART command contract

The Pico currently emits one text frame per control step:

- `M <left_cmd> <right_cmd>`

Example:

```text
M -1.25 1.25
```

You can adapt the frame parser on the MKS side if your MKS firmware expects a different format.

## Wiring (simplified)

- Battery -> MKS power stage
- Battery -> step-down (MP1584EN) -> Pico 5V/3.3V rail
- Pico UART TX/RX <-> MKS UART RX/TX
- Pico I2C -> MPU6050
- Battery divider output -> Pico ADC input
- Common GND between Pico, MKS, sensors, and power rail

## Pico pin map in firmware

From `BalanceBotPico/BalanceBotPico.ino`:

- IMU I2C: SDA=4, SCL=5
- UART to MKS: TX=8, RX=9
- Battery ADC: GPIO26
- RC inputs (currently stubs):
  - Throttle: GPIO20
  - Steering: GPIO21

## Build and upload

1. Install an RP2040 Arduino core.
2. Install `MPU6050_light`.
3. Open `BalanceBotPico/BalanceBotPico.ino` in Arduino IDE.
4. Select Pico board and serial port.
5. Upload.

## Behavior notes

- Balance control update period defaults to 4 ms.
- BOOTSEL button (when available in the core) toggles pause/resume.
- While paused, Pico continuously sends zero commands to MKS.
- Low-voltage cutoff in shared core disables balancing and prints status to USB serial.

## Host-side tests

From project root:

```bash
make -C tests run
```

Tests compile `BalanceBotPico.ino` against fake Arduino shims and verify:
- setup behavior
- IMU error logging
- low-voltage cutoff logging
- UART motor command frames
