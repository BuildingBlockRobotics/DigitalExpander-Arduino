/*
 * LatchedOutput — catching an event too brief for your loop to see.
 *
 * A line of tape passing under a sensor at speed may be visible for 3 ms. A
 * loop that polls every 20 ms will miss it most times, and the times it does
 * not are the ones that make the bug look intermittent.
 *
 * A LATCHED output stays high once the condition has been true even for an
 * instant, until you explicitly clear it. So the question stops being "is it
 * true right now?" and becomes "has it been true since I last looked?", which
 * is the question you actually wanted to ask.
 *
 * Send 'c' over serial to clear the latch.
 */
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;

const uint8_t OUTPUT_INDEX = 0;
const uint8_t SENSOR_PORT = 0;
const uint8_t COLOR_SLOT = 1;

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

  // The advanced tier: full control, and it does NOT save to flash for you.
  BBROutputConfig c;
  c.source = BBROutputSource::SensorClass;
  c.srcIndex = SENSOR_PORT;
  c.classIndex = COLOR_SLOT;
  c.mode = BBROutputMode::Latched;
  c.debounceAssert = 1;  // latch on the first sample that matches
  if (!expander.configureOutput(OUTPUT_INDEX, c)) {
    Serial.print(F("Could not configure the output: "));
    Serial.println(expander.lastErrorText());
    while (true) {
      delay(1000);
    }
  }
  expander.saveConfigToFlash();

  Serial.println(F("Watching. Send 'c' to clear the latch."));
}

void loop() {
  if (Serial.read() == 'c') {
    expander.clearOutputLatch(OUTPUT_INDEX);
    Serial.println(F("cleared"));
  }

  if (expander.outputLatched() & (1 << OUTPUT_INDEX)) {
    Serial.println(F("the colour went past since the last clear"));
  }
  delay(200);
}
