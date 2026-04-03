#include <Adafruit_AHRS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Arduino.h>
#include <QMC5883LCompass.h>

class GY87Fusion {
public:
  bool begin(int filter_rate, Adafruit_AHRS_FusionInterface *filter);
  bool read_sensors(float gyro[3], float accel[3], float mag[3]);
  bool get_gyro_and_pitch(float *gyro_y, float *pitch);

private:
  Adafruit_AHRS_FusionInterface *filter;
  Adafruit_MPU6050 mpu;
  QMC5883LCompass compass;
  Adafruit_Sensor_Calibration_EEPROM cal;
  Adafruit_Sensor *accelerometer, *gyroscope;
  bool init_sensors();
  bool setup_sensors(int filter_rate);
};
