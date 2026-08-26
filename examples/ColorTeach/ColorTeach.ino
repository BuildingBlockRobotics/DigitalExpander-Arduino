/*
 * ColorTeach — teach the board a colour by showing it one.
 *
 * You never write RGB thresholds by hand. Hold the thing you care about in
 * front of the sensor, press a digit, and the board stores what it saw as that
 * colour number. From then on it answers "do I see colour 3?" by itself, and
 * the answer survives a power cycle because teaching saves to flash.
 *
 * Run this once, on the field, under the lighting you will actually compete in.
 * Then use ColorRead (or a trigger) at runtime.
 */
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;

const uint8_t SENSOR_PORT = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }
  Wire.begin();
  Wire.setClock(400000);

  if (!expander.begin()) {
    Serial.print(F("Expander not found: "));
    Serial.println(expander.lastErrorText());
    while (true) {
      delay(1000);
    }
  }
  if (!expander.sensorConnected(SENSOR_PORT)) {
    Serial.println(F("No sensor on that port — plug one in and reset."));
    while (true) {
      delay(1000);
    }
  }

  Serial.println(F("Hold a colour in front of the sensor, then press 1-7 to"));
  Serial.println(F("teach it as that colour number. Press 0 to see what is seen."));
}

void loop() {
  int c = Serial.read();
  if (c < '0' || c > '7') {
    delay(20);
    return;
  }

  if (c == '0') {
    Serial.print(F("currently seeing colour "));
    Serial.println(expander.colorClass(SENSOR_PORT));
    return;
  }

  uint8_t slot = (uint8_t)(c - '0');
  Serial.print(F("Teaching colour "));
  Serial.print(slot);
  Serial.println(F(" — hold still..."));

  // Takes a few seconds, and saves to flash on success. It refuses rather
  // than storing a class that would never fire (too dark) or always fire
  // (saturated) — an error here is the board protecting you from a colour
  // that would have failed silently at the worst moment.
  if (expander.teachColor(SENSOR_PORT, slot)) {
    Serial.println(F("  stored and saved."));
  } else {
    Serial.print(F("  refused: "));
    Serial.println(expander.lastErrorText());
  }
}
