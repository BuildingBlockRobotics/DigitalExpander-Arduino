# BBR Digital Expander — Arduino library

An Arduino library for the **BBR Digital Expander**: a board that adds four
quadrature (or pulse-width) encoder inputs, four colour/distance sensor ports,
four digital outputs and — on the odometry variant — an IMU and a full pose
estimator, all through a single I2C connection.

**Documentation: <https://expander.buildingblockrobotics.com/>**

The expander was designed for FTC, but it is an ordinary I2C target: 100 or
400 kHz, one 7-bit address in `0x38`–`0x3B`, no vendor runtime, no special
host. Anything with a `Wire` implementation can drive it — Uno, Nano, Mega,
Leonardo, ESP32, ESP8266, RP2040, SAMD, STM32, Teensy.

The board can also watch a condition for you — a taught colour, a distance, an
encoder threshold, a heading — and drive a digital output when it is met, with
**no I2C traffic and no code in your loop**. Your sketch reads that output with
`digitalRead()`, the same as a limit switch.

## Wiring, before anything else

A REV Control Hub does two things for this board that your Arduino does not, and
both of them have to be handled on the bench:

- **Fit bus pull-ups.** The Expander carries none on the host-facing bus — a Hub
  provides 2.49 kΩ, and doubling up would only raise sink current. On any other
  host there are none at all until you fit them: **2.2 kΩ–4.7 kΩ from SDA and
  SCL to 3.3 V**. A microcontroller's internal pull-ups are tens of kilohms and
  are not a substitute at 400 kHz or over a robot-length cable.
- **Match the logic level.** Every I/O pin on the board is 3.3 V and **not** 5 V
  tolerant. A 5 V Arduino (Uno, Nano, Mega, Leonardo) needs a level shifter on
  SDA and SCL, with the pull-ups on the 3.3 V side. A 3.3 V board connects
  directly.

Power is 3.3 V at roughly 55 mA; there is no regulator on the board. An Uno's
onboard 3.3 V pin is marginal for that once the sensor LEDs pulse.

## Install

**Arduino IDE** — Sketch → Include Library → Add .ZIP Library, or copy this
folder into `~/Documents/Arduino/libraries/BBRDigitalExpander/`.

**PlatformIO** — add to `platformio.ini`:

```ini
lib_deps = https://github.com/BuildingBlockRobotics/DigitalExpander-Arduino.git
```

## Hello, expander

```cpp
#include <BBRDigitalExpander.h>
#include <Wire.h>

BBRDigitalExpander expander;  // default address 0x38

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  if (!expander.begin()) {
    Serial.println(expander.lastErrorText());
    while (true) { delay(1000); }
  }
}

void loop() {
  int32_t counts  = expander.encoderCount(0);
  bool    isRed   = expander.seesColor(1, 1);
  float   mm      = expander.distanceMm(2);
  float   heading = expander.heading();
  ...
}
```

Run the **DeviceInfo** example first. If it prints a device report, your wiring,
pull-ups and address jumpers are all correct and everything else will work.

## Examples

Start with `ColorTeach` if you are new to the board, and read `TriggerSetup`
and `TriggerRuntime` together — that pair is the whole point of the product.

| Example | Shows |
|---|---|
| `DeviceInfo` | Identity, variant, capabilities, what is plugged into each port |
| `EncoderRead` | Counts and firmware-computed velocities, idempotent reset |
| `ColorTeach` | Teach-by-example colour calibration, saved to flash |
| `ColorRead` | Reading colour classes and confidence — the primary interface |
| `DistanceRead` | Millimetres, and what "nothing in range" looks like |
| `TriggerSetup` | The canonical condition-to-digital-output setup, run once |
| `TriggerRuntime` | The runtime half: one `digitalRead()`, zero I2C |
| `LatchedOutput` | Catching an event too brief for your loop to poll |
| `HeadingIMU` | Heading, and failing loud when the gyro cannot be trusted |
| `PwmEncoder` | Absolute pulse-width encoders and multi-turn wrap tracking |
| `Localizer` | Full pose tracking: parameters, calibration, live pose, re-zeroing |
| `SensorDump` | One-transaction dump of every live value — the bring-up diagnostic |

## Two tiers

The **everyday tier** is one call per question — no bitmasks, no register
numbers, and the tuning values that matter preset to ones that work:

```cpp
expander.teachColor(0, 1);           // this is colour 1
expander.triggerWhenNear(0, 0, 200); // output 0 high inside 200 mm
expander.seesColor(0, 1);
expander.distanceMm(0);
expander.heading();
```

The **advanced tier** underneath exposes the full register map —
`readTelemetry()`, `configureOutput()`, `writeColorClass()`, `runCommand()`,
and raw `readRegisters()`/`writeRegisters()`. You can mix them freely.

The setup helpers (`teachColor`, `triggerOnColor`, `triggerWhenNear`,
`triggerWhenEncoderPast`, `triggerWhenFacing`) **save to flash automatically**,
so run them once from a setup sketch — never in a loop, and never faster than
once a second. The advanced tier never auto-saves; call `saveConfigToFlash()`
yourself.

## Errors, without exceptions

Nothing here throws and nothing allocates. Calls that can fail return `false`,
or a documented sentinel (`NAN` for a heading, `INFINITY` for a distance that
is out of range), and leave the reason behind:

```cpp
float mm = expander.distanceMm(0);
if (!expander.ok()) {
  Serial.println(expander.lastErrorText());   // a sentence, and often the fix
  Serial.println((int)expander.lastStatus()); // a BBRStatus you can branch on
}
```

Error text lives in flash (`__FlashStringHelper`), so carrying it costs an Uno
no RAM.

## Behaviour worth knowing

- **`begin()` refuses loudly** on a wrong `DEVICE_ID`, an unsupported protocol
  major, or an unrecognised hardware variant. A clear failure at setup beats
  corrupt data an hour into a match.
- **Commands are idempotent.** Every command carries a token the firmware will
  not honour twice, so a bus-level retry cannot execute one command twice —
  `resetEncoder()` cannot double-zero.
- **`readTelemetry()` is one snapshot.** The firmware latches the whole
  telemetry region when the register pointer enters it, so every value in a
  `BBRTelemetry` was sampled by the same pass. On a core with a 32-byte `Wire`
  buffer the 96-byte block arrives as three transactions, but the pointer is
  written only once, so it is still **one** snapshot — nothing tears across the
  seam.
- **The everyday getters share that snapshot.** They re-read only when the
  cached block is older than 10 ms (`setTelemetryMaxAgeMs()`), so a loop asking
  four questions costs one bus transaction, not four, and the four answers are
  mutually consistent.
- **IMU access fails rather than returning zeros.** A heading of exactly 0.00
  that never changes looks like working software; `heading()` returns `NAN` and
  says whether the board has no IMU or has a broken one.
- **The localizer block is checked with CRC16.** I2C acknowledges bytes without
  verifying them, so a noise-flipped bit would otherwise arrive looking like a
  perfectly valid pose. A mismatch is re-read, then reported.
- **Config writes are verified by readback.** The expander never NAKs for
  addressing reasons — an out-of-range write is silently discarded — so the
  library writes, reads back and compares.

## Wire buffer sizes

Block reads are chunked to fit whatever `Wire` buffer your core has, so nothing
needs configuring. Block *writes* are not chunked, because the largest one the
library performs is 21 bytes (a 20-byte config window plus the register
pointer) — comfortably inside the 32-byte buffer of even an Uno.

## Licence

MIT — see [LICENSE](LICENSE). Copy it into your sketch, change it, ship it.
