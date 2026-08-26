/*
 * DeviceInfo — is it there, what is it, and what is plugged into it?
 *
 * Start here. If this sketch prints a device report, your wiring, your pull-ups
 * and your address jumpers are all correct, and every other example will work.
 * If it does not, nothing else will, so fix this first.
 *
 * Wiring: expander SDA/SCL to your board's SDA/SCL, GND to GND, and 3.3 V to
 * the expander's power pin.
 *
 * Two things the board does NOT do for you, because on a REV Control Hub the
 * Hub does them: it carries no pull-ups on this bus (fit 2.2k-4.7k to 3.3 V on
 * SDA and SCL), and its I2C lines are 3.3 V and not 5 V tolerant (a 5 V
 * Arduino needs a level shifter). See the wiring guide.
 */
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;  // default address 0x38; pass 0x39-0x3B if jumpered

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  Wire.begin();
  Wire.setClock(400000);  // the expander runs happily at 100 kHz too

  if (!expander.begin()) {
    Serial.print(F("Could not start the expander: "));
    Serial.println(expander.lastErrorText());
    // Stop here rather than printing plausible-looking zeros forever.
    while (true) {
      delay(1000);
    }
  }

  expander.printDeviceInfo(Serial);

  Serial.println(F("\nSensor ports:"));
  for (uint8_t port = 0; port < 4; port++) {
    Serial.print(F("  port "));
    Serial.print(port);
    Serial.print(F(": "));
    switch (expander.sensorType(port)) {
      case BBRSensorType::Color: Serial.println(F("colour sensor")); break;
      case BBRSensorType::Distance: Serial.println(F("distance sensor")); break;
      case BBRSensorType::Empty: Serial.println(F("empty")); break;
      default: Serial.println(F("unrecognised — check the cable")); break;
    }
  }
}

void loop() {
  delay(1000);
}
