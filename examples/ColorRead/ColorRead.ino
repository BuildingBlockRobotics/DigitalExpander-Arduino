/*
 * ColorRead — which taught colour is each sensor looking at?
 *
 * colorClass() answers with a number you taught (1-7), or 0 for "none of the
 * colours I know". It fails safe: a dark, saturated, stale or unplugged sensor
 * reads 0 rather than confidently naming the wrong colour.
 *
 * Teach the colours first with the ColorTeach example.
 */
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;

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
}

void loop() {
  BBRTelemetry t;
  if (!expander.readTelemetry(t)) {
    Serial.println(expander.lastErrorText());
    delay(500);
    return;
  }

  for (uint8_t port = 0; port < 4; port++) {
    if (t.sensor[port].type != BBR_STYPE_COLOR) continue;

    Serial.print(F("port "));
    Serial.print(port);
    Serial.print(F(": colour "));
    Serial.print(t.sensor[port].colorClass);
    Serial.print(F(" (confidence "));
    Serial.print(t.sensor[port].confidence);
    Serial.print(F(")  raw r="));
    Serial.print(t.sensor[port].color.red);
    Serial.print(F(" g="));
    Serial.print(t.sensor[port].color.green);
    Serial.print(F(" b="));
    Serial.print(t.sensor[port].color.blue);
    Serial.print(F("   "));
  }
  Serial.println();

  // The everyday form, for when you only care about one question:
  if (expander.seesColor(0, 1)) {
    Serial.println(F("  -> port 0 sees colour 1"));
  }
  delay(100);
}
