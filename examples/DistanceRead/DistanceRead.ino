/*
 * DistanceRead — millimetres from a time-of-flight sensor.
 *
 * distanceMm() returns INFINITY when nothing is in range, which is not an
 * error and not a special case you have to remember: `distanceMm(p) < 300`
 * is simply false when there is nothing there, which is what you meant.
 */
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;

const uint8_t SENSOR_PORT = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  if (!expander.begin()) {
    Serial.print(F("Expander not found: "));
    Serial.println(expander.lastErrorText());
    while (true) {
      delay(1000);
    }
  }
  if (expander.sensorType(SENSOR_PORT) != BBRSensorType::Distance) {
    Serial.println(F("That port does not have a distance sensor on it."));
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  float mm = expander.distanceMm(SENSOR_PORT);

  if (!expander.ok()) {
    Serial.println(expander.lastErrorText());
  } else if (isinf(mm)) {
    Serial.println(F("nothing in range"));
  } else {
    Serial.print(mm, 0);
    Serial.println(F(" mm"));
  }

  if (mm < 300.0f) {
    Serial.println(F("  -> something is close"));
  }
  delay(100);
}
