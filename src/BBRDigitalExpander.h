// BBRDigitalExpander.h — Arduino driver for the BBR Digital Expander.
//
// The expander is an I2C target that adds four encoder inputs, four
// colour/distance sensor ports, four digital outputs and (on the odometry
// variant) an IMU and a pose estimator to whatever board you plug it into.
// It was built for FTC, but nothing about it is FTC-specific: it speaks
// plain I2C at 100 or 400 kHz and answers to a 7-bit address in 0x38-0x3B.
//
// The API has two tiers, the same two the FTC driver has. The EVERYDAY tier
// is one call per question ("how far?", "do I see red?", "how many
// counts?") with no bitmasks and no register knowledge. The ADVANCED tier
// underneath exposes the full register map for when you outgrow it.
//
// Nothing here throws or allocates. Calls that can fail return false (or a
// documented sentinel: NAN for a heading, INFINITY for a distance) and
// leave the reason in lastStatus()/lastErrorText().
//
// Protocol contract 1.1. Documentation: https://expander.buildingblockrobotics.com/
#ifndef BBR_DIGITAL_EXPANDER_H
#define BBR_DIGITAL_EXPANDER_H

#include <Arduino.h>
#include <Wire.h>

#include "BBRRegMap.h"

// How many bytes of one I2C transaction this core's Wire library can hold.
// Only affects how many chunks a block read is split into, never what the
// caller sees: the register pointer is written once and the chunks continue
// from it, so the whole block still comes out of one firmware snapshot.
#if defined(I2C_BUFFER_LENGTH)
#define BBR_I2C_CHUNK I2C_BUFFER_LENGTH
#elif defined(BUFFER_LENGTH)
#define BBR_I2C_CHUNK BUFFER_LENGTH
#else
#define BBR_I2C_CHUNK 32
#endif

/** Why the last call failed. Ok means it did not. */
enum class BBRStatus : uint8_t {
    Ok = 0,
    NotInitialized,       ///< begin() has not run, or it failed
    Transport,            ///< the device did not answer, or answered short
    WrongDevice,          ///< DEVICE_ID is not a BBR Digital Expander
    ProtocolMismatch,     ///< PROTOCOL_MAJOR is not one this driver speaks
    UnknownVariant,       ///< HW_VARIANT is not a build this driver knows
    BadArgument,          ///< a port, channel, slot or value was out of range
    WriteRejected,        ///< the device kept its old value (out of range?)
    CommandFailed,        ///< the firmware ran the command and reported an error
    CommandTimeout,       ///< the command never left BUSY
    NoImu,                ///< base variant: there is no IMU on this board
    ImuFault,             ///< an IMU is fitted (or strapped for) but not answering
    WrongSensorType,      ///< that port holds the other kind of sensor
    WrongChannelMode,     ///< quadrature call on a pulse-width channel, or vice versa
    CrcMismatch,          ///< the localizer block failed CRC16 three times
    LocalizerNotRunning,  ///< pose read before the localizer was ready
};

enum class BBRSensorType : uint8_t {
    Empty = BBR_STYPE_EMPTY,
    Color = BBR_STYPE_COLOR,
    Distance = BBR_STYPE_DISTANCE,
    Unknown = BBR_STYPE_UNKNOWN,
};

enum class BBRChannelMode : uint8_t { Quadrature = 0, PulseWidth = 1 };

/** Which way an encoder channel counts. */
enum class BBREncoderDirection : uint8_t { Forward = 0, Reverse = 1 };

/** What drives a digital output. */
enum class BBROutputSource : uint8_t {
    Disabled = BBR_DOUT_SRC_DISABLED,
    SensorClass = BBR_DOUT_SRC_SENSOR_CLASS,
    Encoder = BBR_DOUT_SRC_ENCODER,
    ImuHeading = BBR_DOUT_SRC_IMU_HEADING,
};

/** How a digital output behaves once its source matches. */
enum class BBROutputMode : uint8_t {
    Level = BBR_DOUT_MODE_LEVEL,
    Latched = BBR_DOUT_MODE_LATCHED,
    Pulse = BBR_DOUT_MODE_PULSE,
};

enum class BBRLocalizerStatus : uint8_t {
    NotReady = BBR_LOC_NOT_READY,
    WarmingUpImu = BBR_LOC_WARMING_UP_IMU,
    CalibratingImu = BBR_LOC_CALIBRATING_IMU,
    Running = BBR_LOC_RUNNING,
    FaultNoImu = BBR_LOC_FAULT_NO_IMU,
    Unknown = 0xFF,
};

/**
 * One sensor port's reading. Which half of the union is live depends on
 * `type`: read `color` on a COLOR port and `distance` on a DISTANCE port —
 * the firmware overlays them on the same registers, so the other half is
 * the same bytes reinterpreted, not a second measurement.
 */
struct BBRSensorReading {
    uint8_t type;         ///< a BBRSensorType value
    uint8_t status;       ///< BBR_SSTAT_* bits
    uint8_t colorClass;   ///< taught colour 1-7 currently seen, 0 = no match
    uint8_t confidence;   ///< 0-255
    union {
        struct {
            uint16_t red, green, blue, ir, proximity;  ///< rescaled 16-bit channels
        } color;
        struct {
            uint16_t distanceMm;   ///< BBR_DISTANCE_INVALID when out of range
            uint16_t signalRate;
            uint16_t ambientRate;
            uint8_t rangeStatus;
        } distance;
    };
};

/**
 * One snapshot of the whole telemetry block. Every value in it was sampled
 * by the same firmware pass, so they are mutually consistent by
 * construction — you can compare an encoder count against a colour class
 * without wondering whether they are from the same instant.
 *
 * On a channel in PULSE_WIDTH mode, encoderCount is the pulse width in
 * microseconds (0 means NO SIGNAL) and encoderVelocity is us per second.
 */
struct BBRTelemetry {
    int32_t encoderCount[BBR_NUM_ENCODERS];
    int32_t encoderVelocity[BBR_NUM_ENCODERS];  ///< signed counts/s (or us/s)
    BBRSensorReading sensor[BBR_NUM_SENSOR_PORTS];
    uint8_t presentMask;
    uint32_t timestampMicros;  ///< firmware clock, shared with the IMU block

    bool sensorPresent(uint8_t port) const {
        return port < BBR_NUM_SENSOR_PORTS && (presentMask & (1u << port)) != 0;
    }
    bool distanceValid(uint8_t port) const {
        return port < BBR_NUM_SENSOR_PORTS
               && sensor[port].type == BBR_STYPE_DISTANCE
               && sensor[port].distance.distanceMm != BBR_DISTANCE_INVALID;
    }
};

/** One snapshot of the IMU block. Angles in degrees, rates in deg/s. */
struct BBRImuState {
    float quatW, quatX, quatY, quatZ;
    float yawDeg, pitchDeg, rollDeg;  ///< robot frame, yaw CCW-positive, [-180, 180)
    float gyroDps[3];                 ///< robot frame, x y z
    float accelMps2[3];               ///< body frame, gravity removed
    uint8_t calibGrade;               ///< 0-3
    uint8_t status;                   ///< BBR_ISTAT_* bits
    uint32_t timestampMicros;

    /** True while the turn rate exceeds what the gyro can measure — usually
     *  a hard collision. The heading is suspect from that moment on. */
    bool gyroSaturated() const { return (status & BBR_ISTAT_SATURATED) != 0; }
    /** True once the robot has been still long enough to trust the bias. */
    bool biasValid() const { return (status & BBR_ISTAT_BIAS_VALID) != 0; }
    /** True while yaw/pitch/roll/quat are usable. */
    bool fusionValid() const { return (status & BBR_ISTAT_FUSION_VALID) != 0; }
};

/** Robot pose: millimetres and radians, +X forward, +Y left, CCW positive. */
struct BBRPose {
    float xMm;
    float yMm;
    float headingRad;

    float headingDeg() const { return headingRad * 57.29577951308232f; }
};

/** One snapshot of the localizer block. */
struct BBRLocalizerState {
    BBRLocalizerStatus status;
    uint8_t flags;  ///< BBR_LOCF_* bits
    float xMm, yMm, headingRad;
    float velXMmPerSec, velYMmPerSec;  ///< world frame
    float headingVelRadPerSec;
    uint32_t timestampMicros;

    BBRPose pose() const {
        BBRPose p;
        p.xMm = xMm;
        p.yMm = yMm;
        p.headingRad = headingRad;
        return p;
    }
    /** True if the gyro has clipped at any point since the last localizer
     *  reset (latched) — relocalize, the integrated pose has drifted. */
    bool gyroEverSaturated() const { return (flags & BBR_LOCF_GYRO_SATURATED_EVER) != 0; }
    /** True while the pose sits outside +/-32.7 m and reads clamped. */
    bool poseClipped() const { return (flags & BBR_LOCF_POSE_CLIPPED) != 0; }
    /** True if a pod channel's invert mask, mode or PWM parameters changed
     *  while the localizer was running. One tick delta was swallowed rather
     *  than integrated as a jump, so the pose is missing it; latched. Set
     *  direction and mode at setup, before starting the localizer. */
    bool portConflict() const { return (flags & BBR_LOCF_PORT_CONFLICT) != 0; }
};

/** Localizer setup. Stored values take effect on the next resetLocalizer(). */
struct BBRLocalizerParams {
    /** Pod resolution, encoder ticks per millimetre of travel. Measure it:
     *  push the robot a couple of metres along one axis and divide. */
    float ticksPerMmX = 0.0f;
    float ticksPerMmY = 0.0f;
    /** Tracking-point offset from the robot centre, mm, robot frame. */
    float tcpOffsetXMm = 0.0f;
    float tcpOffsetYMm = 0.0f;
    /** Gyro scale correction from the spin-ten-turns calibration, near 1.0. */
    float imuScalar = 1.0f;
    /** Encoder channels the X and Y pods are wired to, 0-3, and different. */
    uint8_t portX = 0;
    uint8_t portY = 1;
    /** Velocity averaging window, ms. */
    uint8_t velocityIntervalMs = BBR_DEFAULT_LOC_VEL_INTERVAL_MS;
};

/** A colour class window, in normalized chromaticity (0-1000 per channel). */
struct BBRColorClass {
    uint16_t rMin = 0, rMax = 0;
    uint16_t gMin = 0, gMax = 0;
    uint16_t bMin = 0, bMax = 0;
    bool proximityGate = false;
    uint16_t proxMin = 0;
    uint16_t proxMax = BBR_PROX_MAX_VALUE;
};

/** A distance class window in millimetres, inclusive bounds. */
struct BBRDistanceClass {
    uint16_t distMin = 0;
    uint16_t distMax = 0;
    uint16_t hysteresisMm = 10;
    uint16_t minSignalRate = 0;
};

/** Full digital-output configuration; see the triggers guide. */
struct BBROutputConfig {
    BBROutputSource source = BBROutputSource::Disabled;
    uint8_t srcIndex = 0;    ///< sensor port or encoder channel
    uint8_t classIndex = 0;  ///< class slot, for SensorClass sources
    bool activeLow = false;
    BBROutputMode mode = BBROutputMode::Level;
    uint8_t debounceAssert = BBR_DEFAULT_DEBOUNCE_ASSERT;
    uint8_t debounceRelease = BBR_DEFAULT_DEBOUNCE_RELEASE;
    uint8_t pulseMs = 0;
    int32_t threshMin = 0;  ///< encoder counts, or yaw in centidegrees
    int32_t threshMax = 0;
};

class BBRDigitalExpander {
   public:
    /**
     * @param i2cAddress 7-bit address, 0x38-0x3B by the address jumpers.
     * @param wire       which I2C bus; defaults to Wire.
     */
    explicit BBRDigitalExpander(uint8_t i2cAddress = BBR_I2C_ADDR_DEFAULT,
                                TwoWire &wire = Wire);

    /**
     * Verify identity and protocol. Call after Wire.begin(). Returns false —
     * and refuses to operate — on a wrong DEVICE_ID, an unsupported
     * PROTOCOL_MAJOR or an unrecognised HW_VARIANT: a clear failure at setup
     * beats corrupt data an hour later.
     */
    bool begin();

    /** True if the most recent call succeeded. */
    bool ok() const { return _status == BBRStatus::Ok; }
    /** Why the most recent call failed. */
    BBRStatus lastStatus() const { return _status; }
    /** A sentence naming the failure and, where there is one, the fix. */
    const __FlashStringHelper *lastErrorText() const;
    /** The firmware's BBR_ERR_* code from the last command that reported one. */
    uint8_t lastCommandResult() const { return _commandResult; }

    uint8_t address() const { return _address; }
    uint8_t capabilities() const { return _capabilities; }
    uint8_t protocolMinor() const { return _protocolMinor; }
    uint8_t hardwareVariant() const { return _hwVariant; }
    bool isOdometryVariant() const { return _hwVariant == BBR_VARIANT_ODOMETRY; }
    bool hasCapability(uint8_t capBit) const { return (_capabilities & capBit) != 0; }

    /** Identity, variant, capabilities and STATUS bits, one line each. */
    void printDeviceInfo(Print &out);

    // ==================================================================
    // EVERYDAY TIER — one call per question, no bitmasks, no register map.
    // ==================================================================

    /**
     * Encoder position of `channel` (0-3) in counts, or 0 if the read
     * failed — check ok(). On a channel in PULSE_WIDTH mode this is the
     * pulse width instead; use pulseWidthUs() there so the name tells the
     * truth.
     */
    int32_t encoderCount(uint8_t channel);
    /** Encoder velocity of `channel` (0-3), signed counts per second. */
    int32_t encoderVelocity(uint8_t channel);
    /**
     * Pulse width on `channel` (0-3) in microseconds — the absolute
     * position of a PWM encoder, multi-turn accumulated when wrap tracking
     * is on. 0 means NO SIGNAL: test for it rather than treating it as a
     * position. Fails with WrongChannelMode if the channel is still
     * quadrature.
     */
    int32_t pulseWidthUs(uint8_t channel);
    /** Zero one encoder channel (0-3). */
    bool resetEncoder(uint8_t channel);
    /** Zero all four encoder channels at once. */
    bool resetAllEncoders();

    /** True while a sensor is detected on `port` (0-3). */
    bool sensorConnected(uint8_t port);
    /** What kind of sensor is on `port` (0-3); Empty when none. */
    BBRSensorType sensorType(uint8_t port);
    /**
     * Which taught colour the sensor on `port` (0-3) currently sees: a slot
     * number 1-7, or 0 for "nothing I was taught". Fails safe — a dark,
     * saturated, stale or unplugged sensor reads 0, never a confident wrong
     * answer. Teach colours with teachColor().
     */
    uint8_t colorClass(uint8_t port);
    /** True while the sensor on `port` sees the colour taught into `classSlot` (1-7). */
    bool seesColor(uint8_t port, uint8_t classSlot);
    /**
     * Distance measured on `port` (0-3), in millimetres, or INFINITY when
     * nothing is in range — so `distanceMm(p) < 300` is always safe to
     * write. Fails with WrongSensorType if that port holds a colour sensor.
     */
    float distanceMm(uint8_t port);

    /**
     * Teach a colour by example: hold the target in front of the sensor on
     * `sensorPort` (0-3) and call this — the reading becomes colour number
     * `classSlot` (1-7). Takes up to a few seconds, and saves to flash
     * automatically. Refuses rather than storing a class that would never —
     * or always — fire: too dark, saturated, or no sensor on the port.
     */
    bool teachColor(uint8_t sensorPort, uint8_t classSlot);

    // The trigger helpers below configure the board to watch a condition and
    // drive a digital output when it is met, with no I2C traffic and no code
    // in your loop. Run them ONCE from a setup sketch: they save to flash, so
    // the board keeps doing it after a power cycle. Your runtime sketch then
    // just reads the output pin — see the TriggerSetup/TriggerRuntime pair.

    /** Output `output` (0-3) goes high while the sensor on `sensorPort` sees
     *  the colour taught into `classSlot` (1-7). Saved to flash. */
    bool triggerOnColor(uint8_t output, uint8_t sensorPort, uint8_t classSlot);
    /** Output `output` (0-3) goes high while the distance sensor on
     *  `sensorPort` sees something within `maxMm`. Saved to flash. */
    bool triggerWhenNear(uint8_t output, uint8_t sensorPort, uint16_t maxMm);
    /** Output `output` (0-3) goes high while encoder `channel` reads
     *  `counts` or more (signed). Saved to flash. */
    bool triggerWhenEncoderPast(uint8_t output, uint8_t channel, int32_t counts);
    /**
     * Output `output` (0-3) goes high while the robot faces `headingDeg`
     * give or take `toleranceDeg` — so (0, 90, 15) holds output 0 high
     * between 75 and 105 degrees. The window may straddle +/-180; the board
     * handles that. Needs the odometry variant. Saved to flash.
     */
    bool triggerWhenFacing(uint8_t output, float headingDeg, float toleranceDeg);
    /** Turn one output off entirely, so nothing drives it. Saved to flash. */
    bool disableOutput(uint8_t output);

    /** Live state of the four digital outputs, bit n = output n. */
    uint8_t outputState();
    /** Which LATCHED outputs are currently latched, bit n = output n. */
    uint8_t outputLatched();
    /** Un-latch one digital output (0-3) configured in LATCHED mode. */
    bool clearOutputLatch(uint8_t output);
    /** Un-latch all four digital outputs. */
    bool clearAllOutputLatches();

    /**
     * Heading in degrees, [-180, 180), positive = turning left, or NAN if
     * the IMU cannot be trusted — check ok(). Zero it with resetHeading().
     * Odometry variant only: a base board fails with NoImu rather than
     * reporting a plausible heading of 0.00 forever.
     */
    float heading();
    /** Zero yaw at the current orientation. */
    bool resetHeading();
    /** Re-estimate gyro bias (about a second). The robot must be still. */
    bool calibrateGyro();

    // ==================================================================
    // ADVANCED TIER — the full register map. Nothing below auto-saves;
    // persist explicitly with saveConfigToFlash().
    // ==================================================================

    /** One snapshot of the whole telemetry block. */
    bool readTelemetry(BBRTelemetry &out);
    /** One snapshot of the IMU block. Fails on a board whose IMU cannot be
     *  trusted rather than reporting zeros. */
    bool readImu(BBRImuState &out);

    /** Whole-block read backing the everyday getters, refreshed when older
     *  than this many milliseconds. Default 10; 0 disables caching. */
    void setTelemetryMaxAgeMs(uint8_t ms) { _telemetryMaxAgeMs = ms; }
    /** Drop the cached telemetry so the next getter reads the device. */
    void invalidateTelemetry() { _telemetryValid = false; }

    /** Device STATUS bits (BBR_STATUS_*), or 0 if the read failed. */
    uint8_t deviceStatus();
    /** True when configuration changed in RAM has not been saved to flash. */
    bool isConfigDirty();
    /** Persist all configuration to flash. Setup-time only (up to 500 ms). */
    bool saveConfigToFlash();
    bool loadFactoryDefaults();
    bool clearFaults();

    /**
     * Live encoder input levels: bit 2n = channel n's A line, bit 2n+1 its
     * B line. Idle inputs read 1 (pull-ups). A bring-up diagnostic — a bit
     * that never toggles while the shaft turns is the dead line. Point
     * sample per firmware pass, so poll fast and turn slowly.
     */
    uint8_t encoderPinState();

    /** Zero the encoder channels whose bits are set; prefer resetEncoder(). */
    bool resetEncoders(uint8_t channelMask);

    /** Set all four channel modes at once; bit n set = channel n PULSE_WIDTH. */
    bool setChannelModes(uint8_t pwmMask);
    bool setChannelMode(uint8_t channel, BBRChannelMode mode);
    bool getChannelMode(uint8_t channel, BBRChannelMode &out);
    /** Multi-turn wrap tracking for one pulse-width channel; needs PWM params. */
    bool setPwmWrapEnabled(uint8_t channel, bool enabled);
    bool setPwmWrapMask(uint8_t mask);
    /**
     * Calibrate one pulse-width channel: the pulse widths at the start and
     * end of a revolution (e.g. 1 and 1024 for a REV Through Bore absolute
     * output). Preserves the other channels. Persist with saveConfigToFlash().
     */
    bool setPwmChannelParams(uint8_t channel, uint16_t minUs, uint16_t maxUs);
    bool getPwmChannelParams(uint8_t channel, uint16_t &minUs, uint16_t &maxUs);

    /**
     * Make `channel` (0-3) count UP when the thing it measures moves the way
     * you consider forward. `Reverse` flips both the count and the velocity.
     *
     * This is the fix for an encoder that counts backwards, and it belongs
     * here rather than in your sketch: the odometry localizer reads the
     * channels straight off the board, so flipping a sign in your own code
     * never reaches it. Push the robot and check — driving forward should
     * raise the X pod's count, strafing left should raise the Y pod's.
     *
     * Written to RAM: call saveConfigToFlash() afterwards, or the direction
     * is back to front again after a power cycle.
     */
    bool setEncoderDirection(uint8_t channel, BBREncoderDirection direction);
    /** Which way `channel` (0-3) currently counts. */
    bool getEncoderDirection(uint8_t channel, BBREncoderDirection &out);

    /** All four channels at once as a bitmask: bit n set inverts channel n.
     *  setEncoderDirection() says the same thing one channel at a time and
     *  reads better; this sets several in one transaction. */
    bool setEncoderInvertMask(uint8_t mask);
    int16_t getEncoderInvertMask();

    /**
     * Which robot axis points UP, so yaw is integrated about the true
     * vertical when the board is not mounted flat. Pass a BBR_AXIS_* value;
     * flat is BBR_AXIS_POS_Z. Re-calibrate the IMU after changing it.
     */
    bool setImuAxisUp(uint8_t axisUp);
    int16_t getImuAxisUp();

    /** Write a colour class window directly. Does not save to flash. */
    bool writeColorClass(uint8_t port, uint8_t slot, const BBRColorClass &c);
    /** Write a distance class window directly. Does not save to flash. */
    bool writeDistanceClass(uint8_t port, uint8_t slot, const BBRDistanceClass &c);
    /** Configure a digital output in full. Does not save to flash. */
    bool configureOutput(uint8_t output, const BBROutputConfig &c);
    /** Un-latch the outputs whose bits are set; prefer clearOutputLatch(). */
    bool clearOutputLatches(uint8_t mask);

    /** Localizer snapshot in any state; check `status` yourself. */
    bool readLocalizerRaw(BBRLocalizerState &out);
    /** Localizer snapshot, failing unless it is RUNNING. */
    bool readLocalizer(BBRLocalizerState &out);
    /** Current pose. Fails unless the localizer is RUNNING. */
    bool getPose(BBRPose &out);
    bool setLocalizerParams(const BBRLocalizerParams &p);
    bool getLocalizerParams(BBRLocalizerParams &out);
    /**
     * Latch stored parameters, zero the pose and calibrate the gyro (about
     * a second). The robot must be completely still. Follow with
     * waitForLocalizerReady() before reading a pose.
     */
    bool resetLocalizerAndCalibrateImu();
    /** Block until the localizer reaches RUNNING; keep the robot still.
     *  False on timeout, or on a state waiting cannot fix. */
    bool waitForLocalizerReady(uint32_t timeoutMs);
    /** Teleport the reported pose — the relocalization primitive after a
     *  vision fix or a wall touch. The heading is normalized for you. */
    bool setPose(float xMm, float yMm, float headingRad);

    /**
     * Issue a command the way the protocol requires: one 3-byte block write
     * (arg, token, opcode), then poll COMMAND_STATUS bounded by the
     * command's maximum duration. Every command carries a token, so a
     * transport retry can never execute one twice.
     */
    bool runCommand(uint8_t opcode, uint8_t arg, uint16_t maxMs);

    // ------------------------------------------------ raw register access

    /** Read `len` bytes from `reg`. Split into chunks for small Wire
     *  buffers, but the pointer is written once so the whole block still
     *  comes out of a single firmware snapshot. */
    bool readRegisters(uint8_t reg, uint8_t *buf, uint8_t len);
    bool writeRegisters(uint8_t reg, const uint8_t *data, uint8_t len);
    bool writeRegister(uint8_t reg, uint8_t value);
    /** One register's value, or -1 if the read failed. */
    int16_t readRegister(uint8_t reg);

   private:
    TwoWire *_wire;
    uint8_t _address;
    BBRStatus _status = BBRStatus::NotInitialized;
    const __FlashStringHelper *_errorText = nullptr;

    bool _begun = false;
    uint8_t _capabilities = 0;
    uint8_t _protocolMinor = 0;
    uint8_t _hwVariant = 0;
    uint8_t _token = 0;
    uint8_t _commandResult = BBR_OK;
    bool _imuVerified = false;

    BBRTelemetry _telemetry;
    bool _telemetryValid = false;
    uint32_t _telemetryMillis = 0;
    uint8_t _telemetryMaxAgeMs = 10;
    int16_t _channelModeMask = -1;

    bool fail(BBRStatus status, const __FlashStringHelper *text);
    bool succeed();
    bool requireBegun();
    bool checkRange(const __FlashStringHelper *text, int32_t v, int32_t lo, int32_t hi);
    bool checkPort(uint8_t port);
    bool checkChannel(uint8_t channel);
    bool checkOutput(uint8_t output);
    bool checkSlot(uint8_t slot);

    bool setPointer(uint8_t reg);
    bool writeVerified(uint8_t reg, uint8_t value, const __FlashStringHelper *name);
    bool selectSensorClass(uint8_t port, uint8_t slot);
    bool writeConfigWindow(const uint8_t *data, uint8_t len);
    bool autoSaveIfDirty();
    bool attachOutputToClass(uint8_t output, uint8_t sensorPort, uint8_t classSlot);

    const BBRTelemetry *cachedTelemetry();
    bool rejectWrongSensorType(uint8_t port, const BBRTelemetry &t, uint8_t wrongType,
                               const __FlashStringHelper *why);
    int16_t channelModeMaskCached();
    bool requireImu();
};

#endif  // BBR_DIGITAL_EXPANDER_H
