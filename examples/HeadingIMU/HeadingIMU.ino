/*
 * HeadingIMU — which way is the robot facing?
 *
 * Odometry-variant boards only. On a base board heading() returns NAN and
 * says why, rather than reporting a heading of exactly 0.00 that never
 * changes — that failure looks like working software, which makes it the
 * most expensive kind of bug to find.
 *
 * Send 'z' to zero the heading, 'c' to re-calibrate the gyro (hold still).
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
  if (!expander.isOdometryVariant()) {
    Serial.println(F("This is a base board — no IMU fitted."));
    while (true) {
      delay(1000);
    }
  }

  // If the board is not mounted flat, tell it which way is up first, then
  // re-calibrate: BBR_AXIS_POS_Z is flat, and there is a value for each face.
  // expander.setImuAxisUp(BBR_AXIS_POS_Z);

  Serial.println(F("Hold the robot still — calibrating the gyro..."));
  if (!expander.calibrateGyro()) {
    Serial.print(F("Calibration failed: "));
    Serial.println(expander.lastErrorText());
  }
  expander.resetHeading();
  Serial.println(F("Ready. 'z' zeroes the heading, 'c' re-calibrates."));
}

void loop() {
  int cmd = Serial.read();
  if (cmd == 'z') expander.resetHeading();
  if (cmd == 'c') expander.calibrateGyro();

  BBRImuState imu;
  if (!expander.readImu(imu)) {
    Serial.print(F("IMU read failed: "));
    Serial.println(expander.lastErrorText());
    delay(500);
    return;
  }

  Serial.print(F("heading "));
  Serial.print(imu.yawDeg, 2);
  Serial.print(F(" deg, turning "));
  Serial.print(imu.gyroDps[2], 1);
  Serial.print(F(" deg/s, calibration grade "));
  Serial.print(imu.calibGrade);

  // Worth watching for: the gyro clipped, which means a collision hard enough
  // that the heading has quietly lost some of the turn it did not measure.
  if (imu.gyroSaturated()) Serial.print(F("  [GYRO SATURATED — heading suspect]"));
  if (!imu.biasValid()) Serial.print(F("  [bias not settled yet]"));
  Serial.println();

  delay(100);
}
