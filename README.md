# PHS6701 Project 1 - Balance Bot

## Dependencies

Install these in Arduino IDE Library Manager
- Adafruit MPU6050 (IMU: Gyro + Acc)
- QMC5883LCompass by MPrograms (Magnetometer)
- Adafruit AHRS (Sensor fusion for absolute yaw)
- L298N by Andrea Lombardo (Motor driver)
- AdvancedPID by Alby312

## Calibration

You can remove the wheels so that it can sit flat on the ground.

## Libraries (for v1 code)

Version 1 uses code suggested by the instruction sheet, which is not in the Library Manager.

Copy the content of the `libraries` folder into the `libraries` folder in your Arduino sketchbook location (In Arduino IDE). You cannot have other libraries of the same functions installed or it's going to create a conflict.

## Notes
InvenSense has proprietary DMP 9-axis fusion which no one has working on Arduino. Open algorithm with I2C bypass works just as well. But you need raw readings, so can't use DMP or its interrupt. (The Adafruit lib doesn't support DMP readings either.)

MPU6050 max sample rate is 1kHz.

[Alternative rates](https://github.com/kriswiner/MPU9250/issues/363#issuecomment-502226744):
1k IMU and 100 mag, update rate 5k. Adafruit used 104Hz for IMU and 1k for mag, update rate 100Hz.

RIP, embedded software's documentations are so poor...
