#include "gy-87-fusion.h"

bool GY87Fusion::init_sensors() {
  // Adapted from the example. Seems like the intention is to allow it to run degraded (no calibration).
  // However having no sensor will lock up the board.

  bool success = true;

  if (!cal.begin()) {
    Serial.println(F("ERROR: Failed to initialize calibration helper"));
    success = false;
  } else if (!cal.loadCalibration()) {
    Serial.println(F("ERROR: No calibration loaded/found"));
    success = false;
  }

  if (!mpu.begin()) {
    Serial.println(F("ERROR: Failed to find sensors"));
    while (1) delay(10);
  }
  mpu.setI2CBypass(true);

  compass.init();  // This function returns void, can't check it.
  accelerometer = mpu.getAccelerometerSensor();
  gyroscope = mpu.getGyroSensor();
  // magnetometer = &compass;

  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  //magnetometer->printSensorDetails();  // Does not have equivalent.

  return success;
}

bool GY87Fusion::setup_sensors(int filter_rate) {
  // Calibration
  compass.setCalibrationOffsets(447.00, 1895.00, -1291.00);
  compass.setCalibrationScales(1.19, 0.80, 1.10);

  // set lowest range for high precision
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // set slightly above refresh rate. 1000Hz / (1+8) > 100Hz. Divisor is how many samples to skip.
  // mpu.setSampleRateDivisor(8)

  // Continuous, 100Hz, 2G, 512 (default oversample, for less noise)
  compass.setMode(0x01, 0x08, 0x00, 0x00);

  filter.begin(filter_rate);

  return true;
}

bool GY87Fusion::read_sensors(float gyro[3], float accel[3], float mag[3]) {
  // Magnetometer values are pre-calibrated
  compass.read();
  mag[0] = (float)compass.getX();
  mag[1] = (float)compass.getY();
  mag[2] = (float)compass.getZ();

  // Read the motion sensors
  sensors_event_t e_accel, e_gyro;  // , mag
  accelerometer->getEvent(&e_accel);
  gyroscope->getEvent(&e_gyro);

  cal.calibrate(e_accel);
  cal.calibrate(e_gyro);

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gyro[0] = e_gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gyro[1] = e_gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gyro[2] = e_gyro.gyro.z * SENSORS_RADS_TO_DPS;

  accel[0] = e_accel.acceleration.x;
  accel[1] = e_accel.acceleration.y;
  accel[2] = e_accel.acceleration.z;

  return true;
}

bool GY87Fusion::get_gyro_and_pitch(float *gyro_y, float *pitch) {
  float gyro[3], accel[3], mag[3];

  read_sensors(gyro, accel, mag);

  filter.update(gyro[0], gyro[1], gyro[2],
                accel[0], accel[1], accel[2],
                mag[0], mag[1], mag[2]);

  *gyro_y = gyro[1];
  *pitch = filter.getPitch();
  return true;
}

// adafruit's event->magnetic is in micro tesla. Probably not important here.
// void compass_get_uT(int scale, double m[3]) {
//   // Read compass values
//   compass.read();

//   // Return XYZ readings
//   int x = compass.getX();
//   int y = compass.getY();
//   int z = compass.getZ();
//   m[0] = (double)x / scale * 100;  // microTesla per gauss
//   m[1] = (double)y / scale * 100;
//   m[2] = (double)z / scale * 100;
// }
