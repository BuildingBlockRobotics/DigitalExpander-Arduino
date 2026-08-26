/*
 * Localizer — where is the robot on the field?
 *
 * Two dead-wheel odometry pods and the onboard IMU, fused on the board into a
 * pose you can just read. Odometry variant only.
 *
 * Setup is once per robot: tell it how many encoder ticks make a millimetre
 * (measure it — push the robot two metres and divide), where the tracking
 * point sits relative to the robot centre, and which channels the pods are on.
 *
 * Send 'r' to re-zero the pose, or 's' to teleport it — which is what you do
 * after a vision fix or a known wall touch.
 */
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;

void startLocalizer();

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

  BBRLocalizerParams p;
  p.ticksPerMmX = 19.894f;  // MEASURE these — a guess here is a wrong pose
  p.ticksPerMmY = 19.894f;
  p.tcpOffsetXMm = 0.0f;
  p.tcpOffsetYMm = 0.0f;
  p.imuScalar = 1.0f;  // from the spin-ten-turns calibration
  p.portX = 0;         // forward pod on encoder channel 0
  p.portY = 1;         // strafe pod on encoder channel 1
  if (!expander.setLocalizerParams(p)) {
    Serial.print(F("Bad localizer parameters: "));
    Serial.println(expander.lastErrorText());
    while (true) {
      delay(1000);
    }
  }
  expander.saveConfigToFlash();

  startLocalizer();
}

void startLocalizer() {
  Serial.println(F("Hold the robot completely still..."));
  // Latches the stored parameters, zeroes the pose and calibrates the gyro.
  if (!expander.resetLocalizerAndCalibrateImu()) {
    Serial.print(F("Reset failed: "));
    Serial.println(expander.lastErrorText());
    return;
  }
  if (!expander.waitForLocalizerReady(5000)) {
    Serial.print(F("Never became ready: "));
    Serial.println(expander.lastErrorText());
    return;
  }
  Serial.println(F("Localizer running. 'r' re-zeroes, 's' teleports to (0,0,0)."));
}

void loop() {
  int cmd = Serial.read();
  if (cmd == 'r') startLocalizer();
  if (cmd == 's') expander.setPose(0.0f, 0.0f, 0.0f);

  BBRLocalizerState s;
  if (!expander.readLocalizer(s)) {
    Serial.print(F("pose unavailable: "));
    Serial.println(expander.lastErrorText());
    delay(500);
    return;
  }

  Serial.print(F("x="));
  Serial.print(s.xMm, 0);
  Serial.print(F("mm y="));
  Serial.print(s.yMm, 0);
  Serial.print(F("mm heading="));
  Serial.print(s.pose().headingDeg(), 1);
  Serial.print(F("deg"));

  // Latched since the last reset: the pose has drifted by an unknown amount
  // and needs a fresh fix, not a smaller loop time.
  if (s.gyroEverSaturated()) Serial.print(F("  [COLLISION — relocalize]"));
  if (s.portConflict()) Serial.print(F("  [pod channel claimed elsewhere]"));
  if (s.poseClipped()) Serial.print(F("  [pose out of range]"));
  Serial.println();

  delay(50);
}
