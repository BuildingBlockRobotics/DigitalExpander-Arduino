/*
 * PwmEncoder — absolute encoders that report position as a pulse width.
 *
 * A REV Through Bore encoder's absolute output is a PWM signal whose pulse
 * width is the shaft angle. Put a channel into PULSE_WIDTH mode and the same
 * count register reports microseconds instead of counts — absolute, correct
 * the instant you power on, with no homing move.
 *
 * A pulse width of 0 means NO SIGNAL. Test for it; do not treat it as
 * "the shaft is at zero", which is exactly what it is not.
 */
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;

const uint8_t CHANNEL = 3;

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

  expander.setChannelMode(CHANNEL, BBRChannelMode::PulseWidth);

  // The pulse widths at the start and end of one revolution. For a REV
  // Through Bore absolute output that is 1 us to 1024 us. Measure yours.
  expander.setPwmChannelParams(CHANNEL, 1, 1024);

  // With the range known, the board can count revolutions through the wrap,
  // so a multi-turn mechanism reports a continuously growing position.
  expander.setPwmWrapEnabled(CHANNEL, true);
  expander.saveConfigToFlash();

  Serial.println(F("Ready."));
}

void loop() {
  int32_t us = expander.pulseWidthUs(CHANNEL);
  if (!expander.ok()) {
    Serial.println(expander.lastErrorText());
  } else if (us == 0) {
    Serial.println(F("NO SIGNAL — check the encoder cable"));
  } else {
    Serial.print(us);
    Serial.println(F(" us"));
  }
  delay(100);
}
