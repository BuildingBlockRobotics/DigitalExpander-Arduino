/*
 * TriggerSetup — teach the board to watch a condition for you. RUN ONCE.
 *
 * This is the whole point of the product. The board can watch a distance, a
 * taught colour, an encoder threshold or a heading, and drive one of its four
 * digital outputs when the condition is met — with no I2C traffic and no code
 * in your loop. The reaction time is the board's, not your sketch's, so it
 * still works when your loop is busy doing something else.
 *
 * The configuration is saved to flash, so it survives a power cycle. Run this
 * sketch once, then load TriggerRuntime and never talk to the board again.
 */
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;

void report(const __FlashStringHelper *what, bool ok) {
  Serial.print(ok ? F("  ok    ") : F("  FAILED "));
  Serial.print(what);
  if (!ok) {
    Serial.print(F(" — "));
    Serial.print(expander.lastErrorText());
  }
  Serial.println();
}

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

  Serial.println(F("Configuring triggers (this writes to flash)..."));

  // Output 0 goes high while the distance sensor on port 0 sees something
  // within 200 mm. The board debounces it and applies hysteresis so the
  // output does not chatter at the edge of the window.
  report(F("output 0 <- within 200 mm of port 0"),
         expander.triggerWhenNear(0, 0, 200));

  // Output 1 goes high while the colour sensor on port 1 sees colour 1.
  // Teach colour 1 first with the ColorTeach example.
  report(F("output 1 <- port 1 sees colour 1"),
         expander.triggerOnColor(1, 1, 1));

  // Output 2 goes high once encoder channel 0 passes 5000 counts.
  report(F("output 2 <- encoder 0 past 5000 counts"),
         expander.triggerWhenEncoderPast(2, 0, 5000));

  // Output 3 goes high while the robot faces 90 degrees +/- 15.
  // Odometry variant only — this one is expected to fail on a base board.
  report(F("output 3 <- facing 90 +/- 15 degrees"),
         expander.triggerWhenFacing(3, 90.0f, 15.0f));

  Serial.println(F("\nDone. Load TriggerRuntime — the board does the rest."));
}

void loop() {
  delay(1000);
}
