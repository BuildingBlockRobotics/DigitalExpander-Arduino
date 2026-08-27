/*
 * EncoderRead — four quadrature encoders, counts and velocity.
 *
 * The expander counts edges for you in firmware, so nothing here depends on
 * how fast your loop runs: you can spend 50 ms rendering to a display and the
 * counts will still be exact when you come back. Velocity is computed on the
 * board too, over a window you can set with VEL_INTERVAL_MS.
 *
 * Send 'r' over the serial monitor to zero all four channels.
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
  // For a motor mounted mirrored, tell the board which way that channel
  // should count rather than negating the number everywhere you use it.
  expander.setEncoderDirection(0, BBREncoderDirection::Forward);
  expander.resetAllEncoders();
  Serial.println(F("Send 'r' to zero the encoders."));
}

void loop() {
  if (Serial.read() == 'r') {
    // Idempotent: the command carries a token, so a bus retry cannot zero twice.
    expander.resetAllEncoders();
  }

  // One block read behind the scenes, shared by all eight calls below, so the
  // counts and velocities you print are all from the same firmware sample.
  BBRTelemetry t;
  if (!expander.readTelemetry(t)) {
    Serial.print(F("read failed: "));
    Serial.println(expander.lastErrorText());
    delay(500);
    return;
  }

  for (uint8_t ch = 0; ch < 4; ch++) {
    Serial.print(F("ch"));
    Serial.print(ch);
    Serial.print(F("="));
    Serial.print(t.encoderCount[ch]);
    Serial.print(F(" ("));
    Serial.print(t.encoderVelocity[ch]);
    Serial.print(F("/s)  "));
  }
  Serial.println();
  delay(100);
}
