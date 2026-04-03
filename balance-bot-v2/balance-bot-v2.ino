#include <AdvancedPID.h>
#include <L298NX2.h>
#include "gy-87-fusion.h"

const int FILTER_UPDATE_RATE_HZ = 100;  // 5000?
const float maxAngle = 45;

uint32_t timestamp;
AdvancedPID stab_pid(2.5, 0.0, 0.15, 0.0);

void setup() {
  // Standalone project, serial not necessary
  Serial.begin(115200);

  // We already prints the errors, so no need to print success bit.
  init_sensors();
  setup_sensors(FILTER_UPDATE_RATE_HZ);

  timestamp = millis();
  Wire.setClock(400000); // 400KHz
}

void loop() {
  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();

  float gyro_y; float pitch;
  get_gyro_and_pitch(&gyro_y, &pitch);

}


// F*, embedded libraries' documentations are so poor.