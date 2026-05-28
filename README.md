# BalanceBot (ESP32 + Dual BLDC + MPU6050 + AS5600)

This project contains starter firmware for a 2-wheel self-balancing robot using:
- ESP32 (on MKS Dual FOC V3.2)
- 2x 2804 BLDC motor + AS5600 magnetic encoder
- MPU6050 IMU
- SimpleFOC library
- Standard RC receiver (2 PWM channels: throttle and steering)

## Important hardware note
Each AS5600 uses fixed I2C address `0x36`.
To use two AS5600 encoders, this firmware puts them on **separate I2C buses**:
- Left encoder on `Wire` (I2C0)
- Right encoder on `Wire1` (I2C1)

## Wiring diagram

```mermaid
flowchart LR
  BATT[Battery Pack] -->|VMOT + GND| MKS[MKS Dual FOC V3.2]
  BATT -->|to Step-Down Input| STEP[MP1584 Step-Down]
  STEP -->|5V/3.3V Out| ESP[ESP32 Logic Rail]

  ESP -->|PWM U/V/W + EN (Motor L)| DRVL[FOC Driver L]
  ESP -->|PWM U/V/W + EN (Motor R)| DRVR[FOC Driver R]
  DRVL --> ML[BLDC Motor Left]
  DRVR --> MR[BLDC Motor Right]

  ESP -->|I2C0 SDA21 SCL22| MPU[MPU6050]
  ESP -->|I2C0 SDA21 SCL22| ASL[AS5600 Left]
  ESP -->|I2C1 SDA18 SCL19| ASR[AS5600 Right]

   RX[RC Receiver] -->|CH2 Throttle PWM| RCT[ESP32 GPIO35]
   RX -->|CH1 Steering PWM| RCS[ESP32 GPIO39]
   RX -->|GND| ESP
   STEP -->|5V to RX| RX

  ML --- ASL
  MR --- ASR

  BATT -->|through Voltage Sensor| VS[0-25V Voltage Sensor]
  VS -->|Scaled Analog Out| ADC[ESP32 ADC GPIO34]
```

## Pin map used by firmware

### I2C
- MPU6050 + Left AS5600: `SDA=21`, `SCL=22`
- Right AS5600 (separate bus): `SDA=18`, `SCL=19`

### Motor driver pins
- Motor Left: `U=25`, `V=26`, `W=27`, `EN=4`
- Motor Right: `U=14`, `V=32`, `W=33`, `EN=16`

### Analog
- Battery sensor output: `GPIO34`

### RC receiver PWM inputs
- Throttle channel: `GPIO35`
- Steering channel: `GPIO39`

## Software setup (Arduino IDE)
1. Install ESP32 board package by Espressif.
2. Install libraries:
   - `SimpleFOC`
   - `MPU6050_light`
3. Open `BalanceBot.ino`.
4. Select your ESP32 board and port.
5. Upload.

## First startup procedure
1. Put robot on a stand so wheels are free.
2. Power on and open serial monitor at `115200`.
3. Let IMU offset calibration complete.
4. Send command `e` to enable balancing loop.
5. If wheel direction is wrong, invert one motor command in code (`motorRight.move(-rightCmd)` line).
6. Tune PID values:
   - Angle loop: `kp_angle`, `ki_angle`, `kd_angle`
   - Rate loop: `kp_rate`, `ki_rate`, `kd_rate`

## RC control behavior
- Receiver CH2 (throttle) maps to forward/backward by changing balance angle setpoint.
- Receiver CH1 (steering) maps to left/right turn via differential wheel command.
- If RC pulses are missing for >120 ms, RC commands are forced to zero.
## Serial commands
- `e` enable balancing
- `v<number>` set rate `kp`
- `w<number>` set rate `ki`
- `x<number>` set rate `kd`

## Notes
- Keep all grounds common (battery, driver, ESP32, sensors).
- Verify voltage-sensor output never exceeds ESP32 ADC input max (3.3V).
- ESP32 GPIO inputs are not 5V tolerant. If your receiver PWM outputs are 5V, use a level shifter or divider.
- Start with low `DRIVER_VOLTAGE_LIMIT` for safe tuning.
