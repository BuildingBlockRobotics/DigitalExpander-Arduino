/*
 * SensorDump — everything the board knows, in one bus transaction.
 *
 * The bring-up diagnostic. Every value printed here came from a single
 * firmware snapshot, so if two of them disagree, that disagreement is real
 * and not an artifact of reading them at different times.
 */
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;

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
  expander.printDeviceInfo(Serial);
}

void printSensorStatus(uint8_t bits) {
  if (bits & BBR_SSTAT_VALID) Serial.print(F("valid "));
  if (bits & BBR_SSTAT_STALE) Serial.print(F("STALE "));
  if (bits & BBR_SSTAT_QUALITY_FAIL) Serial.print(F("QUALITY-FAIL "));
  if (bits & BBR_SSTAT_OVERFLOW) Serial.print(F("SATURATED "));
  if (bits & BBR_SSTAT_BUS_ERROR) Serial.print(F("BUS-ERROR "));
  if (bits & BBR_SSTAT_INSUFFICIENT_SIGNAL) Serial.print(F("WEAK-SIGNAL "));
}

void loop() {
  BBRTelemetry t;
  if (!expander.readTelemetry(t)) {
    Serial.println(expander.lastErrorText());
    delay(500);
    return;
  }

  Serial.print(F("\n--- t="));
  Serial.print(t.timestampMicros);
  Serial.println(F(" us"));

  for (uint8_t ch = 0; ch < 4; ch++) {
    Serial.print(F("  encoder "));
    Serial.print(ch);
    Serial.print(F(": "));
    Serial.print(t.encoderCount[ch]);
    Serial.print(F(" counts, "));
    Serial.print(t.encoderVelocity[ch]);
    Serial.println(F(" counts/s"));
  }

  // A line that never toggles while you turn the shaft is the dead wire.
  Serial.print(F("  encoder pins: 0b"));
  Serial.println(expander.encoderPinState(), 2);

  for (uint8_t port = 0; port < 4; port++) {
    Serial.print(F("  port "));
    Serial.print(port);
    Serial.print(F(": "));
    if (!t.sensorPresent(port)) {
      Serial.println(F("empty"));
      continue;
    }
    if (t.sensor[port].type == BBR_STYPE_DISTANCE) {
      Serial.print(F("distance "));
      if (t.distanceValid(port)) {
        Serial.print(t.sensor[port].distance.distanceMm);
        Serial.print(F(" mm"));
      } else {
        Serial.print(F("out of range"));
      }
      Serial.print(F(", signal "));
      Serial.print(t.sensor[port].distance.signalRate);
      Serial.print(F("  "));
    } else {
      Serial.print(F("colour class "));
      Serial.print(t.sensor[port].colorClass);
      Serial.print(F(" conf "));
      Serial.print(t.sensor[port].confidence);
      Serial.print(F("  r="));
      Serial.print(t.sensor[port].color.red);
      Serial.print(F(" g="));
      Serial.print(t.sensor[port].color.green);
      Serial.print(F(" b="));
      Serial.print(t.sensor[port].color.blue);
      Serial.print(F(" ir="));
      Serial.print(t.sensor[port].color.ir);
      Serial.print(F(" prox="));
      Serial.print(t.sensor[port].color.proximity);
      Serial.print(F("  "));
    }
    printSensorStatus(t.sensor[port].status);
    Serial.println();
  }

  Serial.print(F("  digital outputs: 0b"));
  Serial.print(expander.outputState(), 2);
  Serial.print(F("  latched 0b"));
  Serial.println(expander.outputLatched(), 2);

  delay(500);
}
