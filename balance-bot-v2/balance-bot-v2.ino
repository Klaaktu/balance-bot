#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

const int FILTER_UPDATE_RATE_HZ = 100;  // 5000?
const int PRINT_EVERY_N_UPDATES = 10;

// pick your filter! slower == better quality output (Uno can probably only use the fastest)
//Adafruit_Madgwick filter;  // faster than NXP
const Adafruit_Mahony filter;  // fastest/smalleset

const Adafruit_MPU6050 mpu;
const QMC5883LCompass compass;

Adafruit_Sensor *accelerometer, *gyroscope;  // , *magnetometer
const Adafruit_Sensor_Calibration_EEPROM cal;

uint32_t timestamp;

bool init_sensors(void) {
  if (!mpu.begin()) {
    return false;
  }
  mpu.setI2CBypass(true);
  
  // Calibration
  compass.setCalibrationOffsets(447.00, 1895.00, -1291.00);
  compass.setCalibrationScales(1.19, 0.80, 1.10);
  compass.init();  // This function returns void, can't check it.
  accelerometer = mpu.getAccelerometerSensor();
  gyroscope = mpu.getGyroSensor();
  // magnetometer = &compass;

  return true;
}

void setup_sensors(void) {
  // set lowest range
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // set slightly above refresh rate. 1000Hz / (1+8) > 100Hz, 1kHz is the max. Divisor is how many samples to skip.
  // mpu.setSampleRateDivisor(8)

  // Continuous, 100Hz, 2G, 512 (default oversample, for less noise)
  compass.setMode(0x01, 0x08, 0x00, 0x00);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) yield();

  if (!cal.begin()) {
    Serial.println("ERROR: Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("ERROR: No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("ERROR: Failed to find sensors");
    while (1) yield();  // delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  //magnetometer->printSensorDetails();  // Does not have equivalent.

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
}

// adafruit's event->magnetic is in micro tesla. Probably not important here.
void compass_get_uT(int scale, double m[3]) {
  // Read compass values
  compass.read();

  // Return XYZ readings
  int x = compass.getX();
  int y = compass.getY();
  int z = compass.getZ();
  m[0] = (double)x / scale * 100;  // microTesla per gauss
  m[1] = (double)y / scale * 100;
  m[2] = (double)z / scale * 100;
}

bool fuse(float ypr[3]) {
  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return false;
  }
  
  timestamp = millis();
  // Read the motion sensors
  sensors_event_t accel, gyro;  // , mag
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  // magnetometer->getEvent(&mag);
  compass.read();
  // filter.update() takes floats
  float mx = compass.getX();
  float my = compass.getY();
  float mz = compass.getZ();

  // cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  float gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  float gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  float gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mx, my, mz);

  ypr[0] = filter.getYaw();
  ypr[1] = filter.getPitch();
  ypr[2] = filter.getRoll();

  return true;
}

void loop() {}


// Alternative rates: https://github.com/kriswiner/MPU9250/issues/363#issuecomment-502226744
// 1k IMU and 100 mag, update rate 5k
// Adafruit used 104Hz for IMU and 1k for mag

// InvenSense has proprietary DMP 9-axis fusion which no one has working on Arduino.
// Open algorithm with I2C bypass works just as well.