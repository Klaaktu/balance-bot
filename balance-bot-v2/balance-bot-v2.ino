#include "gy-87-fusion.h"
#include <Adafruit_AHRS.h>
#include <AdvancedPID.h>
#include <L298NX2.h>

const int FILTER_UPDATE_RATE_HZ = 100; // 5000?
const float maxAngle = 45;

// Uno can probably only use the fastest
// pick your filter! slower == better quality output
// Adafruit_NXPSensorFusion filter; // slowest
// Adafruit_Madgwick filter;  // faster than NXP
Adafruit_Mahony filter; // fastest/smalleset

GY87Fusion imu;
AdvancedPID stab_pid(2.5, 0.0, 0.15, 0.0);

uint32_t timestamp;

void setup() {
  // Standalone project, serial not necessary
  Serial.begin(115200);

  // We already prints the errors, so no need to print success bit.
  imu.begin(FILTER_UPDATE_RATE_HZ, &filter);

  timestamp = millis();
  Wire.setClock(400000); // 400KHz
}

void loop() {
  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();

  float gyro_y;
  float pitch;
  imu.get_gyro_and_pitch(&gyro_y, &pitch);
}
