
This project now targets a direct motor-control architecture using the `SimpleFOC` library:
- LOLIN32LITE runs both FOC loops and high-level coupling logic
- Two AS5600 magnetic sensors provide shaft feedback over I2C
- Two 3PWM BLDC drivers are controlled directly from the LOLIN32LITE


Parts list
MPU6050 GY-521 6DOF IMU (3-axis accelerometer + gyroscope)

2x 2804 BLDC hollow-shaft motor 
2x AS5600 magnetic encoder set (FOC support)

2x Simple FOC mini

LOLIN32 Lite (ESP32)
