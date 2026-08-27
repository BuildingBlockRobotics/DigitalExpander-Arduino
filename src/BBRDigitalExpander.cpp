// BBRDigitalExpander.cpp — Arduino driver for the BBR Digital Expander.
//
// See BBRDigitalExpander.h for the API and https://expander.buildingblockrobotics.com/
// for what each register means.

#include "BBRDigitalExpander.h"

#include <math.h>
#include <string.h>

namespace {

// The protocol is little-endian on the wire regardless of what the host is,
// so every multi-byte value is assembled byte by byte rather than memcpy'd.
uint16_t le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

int16_t sle16(const uint8_t *p) {
    return (int16_t)le16(p);
}

int32_t sle32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16)
                     | ((uint32_t)p[3] << 24));
}

uint32_t ule32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16)
           | ((uint32_t)p[3] << 24);
}

// avr-libc does not carry every float variant of the libm rounders that
// larger cores do, so rounding is done here and depends on no libm symbol.
int32_t roundToI32(float v) {
    return (int32_t)(v < 0.0f ? v - 0.5f : v + 0.5f);
}

void put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

// A block that is entirely 0x00 or entirely 0xFF never came from the device:
// the blocks this guards all contain a timestamp, a CRC or a status byte with
// reserved bits, none of which are ever uniform while the board is alive.
// 0x00 is a transfer that returned nothing, 0xFF a bus released mid-read.
bool looksLikeFailedRead(const uint8_t *d, uint8_t len) {
    if (len < 2) return false;
    if (d[0] != 0x00 && d[0] != 0xFF) return false;
    for (uint8_t i = 1; i < len; i++) {
        if (d[i] != d[0]) return false;
    }
    return true;
}

/** Fold a heading in centidegrees into the firmware's -18000..17999. */
int32_t wrapCdeg(int32_t cdeg) {
    return ((cdeg + 18000) % 36000 + 36000) % 36000 - 18000;
}

/**
 * PROFIBUS CRC16 (poly 0x1DCF, init 0xFFFF, no reflection, xor-out 0xFFFF) —
 * the same algorithm the OctoQuad uses, so ported verification code works
 * unchanged.
 */
uint16_t crc16Profibus(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1DCF) : (uint16_t)(crc << 1);
        }
    }
    return (uint16_t)(crc ^ 0xFFFF);
}

const uint8_t kAllChannels = (1u << BBR_NUM_ENCODERS) - 1;
const uint8_t kAllOutputs = (1u << BBR_NUM_DOUTS) - 1;

}  // namespace

BBRDigitalExpander::BBRDigitalExpander(uint8_t i2cAddress, TwoWire &wire)
    : _wire(&wire), _address(i2cAddress) {}

// --------------------------------------------------------------- bookkeeping

bool BBRDigitalExpander::fail(BBRStatus status, const __FlashStringHelper *text) {
    _status = status;
    _errorText = text;
    return false;
}

bool BBRDigitalExpander::succeed() {
    _status = BBRStatus::Ok;
    _errorText = nullptr;
    return true;
}

const __FlashStringHelper *BBRDigitalExpander::lastErrorText() const {
    if (_status == BBRStatus::Ok) return F("OK");
    if (_errorText != nullptr) return _errorText;
    return F("unknown failure");
}

bool BBRDigitalExpander::requireBegun() {
    if (_begun) return true;
    return fail(BBRStatus::NotInitialized,
                F("begin() has not succeeded — call Wire.begin() then expander.begin()"));
}

bool BBRDigitalExpander::checkRange(const __FlashStringHelper *text, int32_t v, int32_t lo,
                                    int32_t hi) {
    if (v >= lo && v <= hi) return true;
    return fail(BBRStatus::BadArgument, text);
}

bool BBRDigitalExpander::checkPort(uint8_t port) {
    return checkRange(F("sensor port must be 0-3"), port, 0, BBR_NUM_SENSOR_PORTS - 1);
}

bool BBRDigitalExpander::checkChannel(uint8_t channel) {
    return checkRange(F("encoder channel must be 0-3"), channel, 0, BBR_NUM_ENCODERS - 1);
}

bool BBRDigitalExpander::checkOutput(uint8_t output) {
    return checkRange(F("digital output must be 0-3"), output, 0, BBR_NUM_DOUTS - 1);
}

bool BBRDigitalExpander::checkSlot(uint8_t slot) {
    return checkRange(F("class slot must be 1-7"), slot, BBR_CLASS_SLOT_FIRST,
                      BBR_CLASS_SLOT_LAST);
}

// ------------------------------------------------------------------ transport

bool BBRDigitalExpander::setPointer(uint8_t reg) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission() != 0) {
        return fail(BBRStatus::Transport,
                    F("the expander did not acknowledge — check wiring, power and the "
                      "address jumpers"));
    }
    return true;
}

bool BBRDigitalExpander::readRegisters(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (len == 0) return succeed();
    if (!setPointer(reg)) return false;

    // The firmware latches a snapshot region when the pointer ENTERS it, and
    // the pointer survives a STOP. So the pointer is written once above and
    // the chunks below simply continue from it: a 96-byte telemetry read on a
    // 32-byte Wire buffer is three transactions but still one snapshot, with
    // no chance of a 32-bit count tearing across the seam.
    uint8_t got = 0;
    while (got < len) {
        uint8_t want = len - got;
        if (want > BBR_I2C_CHUNK) want = BBR_I2C_CHUNK;
        uint8_t n = _wire->requestFrom(_address, want);
        if (n != want) {
            return fail(BBRStatus::Transport,
                        F("short I2C read — the expander stopped answering mid-block"));
        }
        for (uint8_t i = 0; i < want; i++) {
            buf[got + i] = (uint8_t)_wire->read();
        }
        got += want;
    }
    return succeed();
}

bool BBRDigitalExpander::writeRegisters(uint8_t reg, const uint8_t *data, uint8_t len) {
    if ((uint16_t)len + 1 > BBR_I2C_CHUNK) {
        return fail(BBRStatus::BadArgument,
                    F("write is larger than this board's I2C buffer"));
    }
    _wire->beginTransmission(_address);
    _wire->write(reg);
    for (uint8_t i = 0; i < len; i++) {
        _wire->write(data[i]);
    }
    if (_wire->endTransmission() != 0) {
        return fail(BBRStatus::Transport,
                    F("the expander did not acknowledge a write — check wiring and power"));
    }
    return succeed();
}

bool BBRDigitalExpander::writeRegister(uint8_t reg, uint8_t value) {
    return writeRegisters(reg, &value, 1);
}

int16_t BBRDigitalExpander::readRegister(uint8_t reg) {
    uint8_t v;
    if (!readRegisters(reg, &v, 1)) return -1;
    return v;
}

bool BBRDigitalExpander::writeVerified(uint8_t reg, uint8_t value,
                                       const __FlashStringHelper *name) {
    // The bus layer never NAKs: an out-of-range write is silently discarded
    // and the old value kept. Write, read back, compare.
    if (!writeRegister(reg, value)) return false;
    int16_t back = readRegister(reg);
    if (back < 0) return false;
    if ((uint8_t)back != value) return fail(BBRStatus::WriteRejected, name);
    return succeed();
}

// ----------------------------------------------------------------- identity

bool BBRDigitalExpander::begin() {
    _begun = false;
    _imuVerified = false;
    _telemetryValid = false;
    _channelModeMask = -1;

    // DEVICE_ID .. STATUS in one read. A glitched or cut-short transaction
    // reads as all zeros or all 0xFF, both transient, so a malformed identity
    // block is retried before concluding the device is the wrong one. Only a
    // stable, well-formed-but-mismatched answer is a refusal.
    uint8_t id[9];
    bool sane = false;
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        if (readRegisters(BBR_REG_DEVICE_ID, id, sizeof(id))
            && id[BBR_REG_DEVICE_ID] == BBR_DEVICE_ID_VALUE
            && id[BBR_REG_PROTOCOL_MAJOR] != 0xFF) {
            sane = true;
            break;
        }
        delay(20);
    }
    if (!sane) {
        if (_status == BBRStatus::Transport) return false;
        return fail(BBRStatus::WrongDevice,
                    F("not a BBR Digital Expander — DEVICE_ID did not read back as 0xB2"));
    }

    if (id[BBR_REG_PROTOCOL_MAJOR] != BBR_PROTOCOL_MAJOR_VALUE) {
        // Registers may have moved or changed meaning: refuse to operate.
        return fail(BBRStatus::ProtocolMismatch,
                    F("firmware speaks a protocol major this driver does not — update "
                      "the driver or the firmware"));
    }
    _protocolMinor = id[BBR_REG_PROTOCOL_MINOR];
    _hwVariant = id[BBR_REG_HW_VARIANT];
    if (_hwVariant != BBR_VARIANT_BASE && _hwVariant != BBR_VARIANT_ODOMETRY) {
        // Guessing here risks driving hardware that is not fitted.
        return fail(BBRStatus::UnknownVariant,
                    F("unknown hardware variant — firmware older than the board, or a "
                      "mis-assembled board"));
    }
    _capabilities = id[BBR_REG_CAPABILITIES];
    _begun = true;
    return succeed();
}

void BBRDigitalExpander::printDeviceInfo(Print &out) {
    uint8_t id[9];
    if (!readRegisters(BBR_REG_DEVICE_ID, id, sizeof(id))) {
        out.print(F("BBR Digital Expander: no answer at 0x"));
        out.println(_address, HEX);
        return;
    }
    out.print(F("BBR Digital Expander at 0x"));
    out.println(_address, HEX);
    out.print(F("  device id   0x"));
    out.println(id[BBR_REG_DEVICE_ID], HEX);
    out.print(F("  firmware    "));
    out.print(id[BBR_REG_FW_VERSION_MAJOR]);
    out.print('.');
    out.print(id[BBR_REG_FW_VERSION_MINOR]);
    out.print('.');
    out.println(id[BBR_REG_FW_VERSION_PATCH]);
    out.print(F("  protocol    "));
    out.print(id[BBR_REG_PROTOCOL_MAJOR]);
    out.print('.');
    out.println(id[BBR_REG_PROTOCOL_MINOR]);
    out.print(F("  variant     "));
    out.println(id[BBR_REG_HW_VARIANT] == BBR_VARIANT_ODOMETRY ? F("odometry (IMU fitted)")
                                                               : F("base"));
    uint8_t cap = id[BBR_REG_CAPABILITIES];
    out.print(F("  can do      "));
    if (cap & BBR_CAP_ENCODERS) out.print(F("encoders "));
    if (cap & BBR_CAP_COLOR) out.print(F("colour "));
    if (cap & BBR_CAP_IMU) out.print(F("imu "));
    if (cap & BBR_CAP_DOUT) out.print(F("outputs "));
    out.println();
    uint8_t st = id[BBR_REG_STATUS];
    out.print(F("  status      "));
    if (st & BBR_STATUS_READY) out.print(F("ready "));
    if (st & BBR_STATUS_CFG_DIRTY) out.print(F("unsaved-config "));
    if (st & BBR_STATUS_FLASH_ERROR) out.print(F("FLASH-ERROR "));
    if (st & BBR_STATUS_SENSOR_BUS_ERROR) out.print(F("SENSOR-BUS-ERROR "));
    if (st & BBR_STATUS_IMU_PRESENT) out.print(F("imu-present "));
    if (st & BBR_STATUS_IMU_CALIBRATED) out.print(F("imu-calibrated "));
    if (st & BBR_STATUS_CMD_OVERRUN) out.print(F("command-overrun "));
    if (st & BBR_STATUS_CFG_DEFAULTED) out.print(F("config-defaulted "));
    out.println();
}

// ----------------------------------------------------------------- commands

bool BBRDigitalExpander::runCommand(uint8_t opcode, uint8_t arg, uint16_t maxMs) {
    if (!requireBegun()) return false;

    // Any command can move counts, classes or modes: drop the caches.
    _telemetryValid = false;
    _channelModeMask = -1;

    // Every command carries a token the firmware refuses to honour twice, so
    // a transport-level retry cannot execute one command two times — a reset
    // cannot double-zero.
    _token++;
    if (_token == BBR_TOKEN_NONE) _token = 1;
    uint8_t block[3] = {arg, _token, opcode};
    if (!writeRegisters(BBR_REG_COMMAND_ARG, block, sizeof(block))) return false;

    // COMMAND_RESULT and COMMAND_ECHO are written before COMMAND_STATUS
    // leaves BUSY, so a status other than BUSY already has its result beside
    // it — one read is enough, no confirming re-read.
    const uint32_t start = millis();
    const uint32_t limit = (uint32_t)maxMs + 50;
    for (;;) {
        uint8_t out[3];
        if (!readRegisters(BBR_REG_COMMAND_STATUS, out, sizeof(out))) return false;
        if (out[0] == BBR_CMDSTAT_DONE) {
            _commandResult = out[1];
            return succeed();
        }
        if (out[0] == BBR_CMDSTAT_ERROR) {
            _commandResult = out[1];
            return fail(BBRStatus::CommandFailed, F("the expander rejected a command — "
                                                    "see lastCommandResult()"));
        }
        if (millis() - start > limit) {
            // Overrunning the documented maximum is a fault a retry will not
            // fix, so this is an error rather than a longer wait.
            return fail(BBRStatus::CommandTimeout,
                        F("command never finished — the expander may have reset"));
        }
        delay(2);
    }
}

bool BBRDigitalExpander::clearFaults() {
    return runCommand(BBR_CMD_CLEAR_FAULTS, 0, 10);
}

bool BBRDigitalExpander::saveConfigToFlash() {
    return runCommand(BBR_CMD_CFG_SAVE_FLASH, 0, BBR_CMD_CFG_SAVE_FLASH_MAX_MS);
}

bool BBRDigitalExpander::loadFactoryDefaults() {
    return runCommand(BBR_CMD_CFG_LOAD_DEFAULTS, 0, 10);
}

uint8_t BBRDigitalExpander::deviceStatus() {
    int16_t v = readRegister(BBR_REG_STATUS);
    return v < 0 ? 0 : (uint8_t)v;
}

bool BBRDigitalExpander::isConfigDirty() {
    int16_t v = readRegister(BBR_REG_STATUS);
    return v > 0 && ((uint8_t)v & BBR_STATUS_CFG_DIRTY) != 0;
}

/**
 * Save if anything changed, riding out the firmware's one-save-per-second
 * rate limit. The setup helpers call this so "it worked yesterday, gone
 * today" cannot happen to someone who has never heard of flash wear.
 */
bool BBRDigitalExpander::autoSaveIfDirty() {
    // Read STATUS directly rather than through isConfigDirty(): that returns
    // false both for "nothing to save" and for "the read failed", and those
    // two must not lead to the same answer here.
    int16_t st = readRegister(BBR_REG_STATUS);
    if (st < 0) return false;
    if (((uint8_t)st & BBR_STATUS_CFG_DIRTY) == 0) return succeed();
    if (saveConfigToFlash()) return true;
    if (_status != BBRStatus::CommandFailed
        || _commandResult != BBR_ERR_FLASH_RATE_LIMITED) {
        return false;
    }
    delay(BBR_FLASH_SAVE_RATE_LIMIT_MS);
    return saveConfigToFlash();
}

// ---------------------------------------------------------------- telemetry

bool BBRDigitalExpander::readTelemetry(BBRTelemetry &out) {
    if (!requireBegun()) return false;
    uint8_t d[96];
    if (!readRegisters(BBR_SNAP_TELEMETRY_START, d, sizeof(d))) return false;
    // The timestamp is nonzero from the first millisecond of boot, so a
    // uniform block can only be a transfer that never happened.
    if (looksLikeFailedRead(d, sizeof(d))) {
        return fail(BBRStatus::Transport,
                    F("telemetry read came back uniform — the transfer never reached "
                      "the expander"));
    }

    const uint8_t base = BBR_SNAP_TELEMETRY_START;
    for (uint8_t i = 0; i < BBR_NUM_ENCODERS; i++) {
        out.encoderCount[i] = sle32(&d[BBR_REG_ENC0_COUNT - base + 4 * i]);
        out.encoderVelocity[i] = sle32(&d[BBR_REG_ENC0_VELOCITY - base + 4 * i]);
    }
    for (uint8_t i = 0; i < BBR_NUM_SENSOR_PORTS; i++) {
        BBRSensorReading &s = out.sensor[i];
        s.type = d[BBR_REG_SENSOR_TYPE - base + i];
        s.status = d[BBR_REG_SENSOR_STATUS - base + i];
        s.colorClass = d[BBR_REG_S0_CLASS - base + i];
        s.confidence = d[BBR_REG_CLASS_CONFIDENCE - base + i];

        const uint8_t *raw = &d[BBR_RAW_BASE - base + BBR_RAW_STRIDE * i];
        if (s.type == BBR_STYPE_DISTANCE) {
            s.distance.distanceMm = le16(&raw[BBR_RAW_DISTANCE_MM]);
            s.distance.signalRate = le16(&raw[BBR_RAW_SIGNAL_RATE]);
            s.distance.ambientRate = le16(&raw[BBR_RAW_AMBIENT_RATE]);
            s.distance.rangeStatus = raw[BBR_RAW_RANGE_STATUS];
        } else {
            s.color.red = le16(&raw[BBR_RAW_RED]);
            s.color.green = le16(&raw[BBR_RAW_GREEN]);
            s.color.blue = le16(&raw[BBR_RAW_BLUE]);
            s.color.ir = le16(&raw[BBR_RAW_IR]);
            s.color.proximity = le16(&raw[BBR_RAW_PROXIMITY]);
        }
    }
    out.presentMask = d[BBR_REG_SENSOR_PRESENT_MASK - base];
    out.timestampMicros = ule32(&d[BBR_REG_TELEMETRY_TIMESTAMP - base]);
    return succeed();
}

/**
 * The snapshot behind the everyday getters. A loop that asks four one-value
 * questions costs one bus transaction, not four, and every answer within a
 * pass is self-consistent.
 */
const BBRTelemetry *BBRDigitalExpander::cachedTelemetry() {
    if (_telemetryValid && (uint32_t)(millis() - _telemetryMillis) <= _telemetryMaxAgeMs) {
        succeed();
        return &_telemetry;
    }
    if (!readTelemetry(_telemetry)) {
        _telemetryValid = false;
        return nullptr;
    }
    _telemetryValid = true;
    _telemetryMillis = millis();
    return &_telemetry;
}

uint8_t BBRDigitalExpander::encoderPinState() {
    int16_t v = readRegister(BBR_REG_ENC_PIN_STATE);
    return v < 0 ? 0 : (uint8_t)v;
}

// ------------------------------------------------------------------ encoders

int32_t BBRDigitalExpander::encoderCount(uint8_t channel) {
    if (!checkChannel(channel)) return 0;
    const BBRTelemetry *t = cachedTelemetry();
    return t ? t->encoderCount[channel] : 0;
}

int32_t BBRDigitalExpander::encoderVelocity(uint8_t channel) {
    if (!checkChannel(channel)) return 0;
    const BBRTelemetry *t = cachedTelemetry();
    return t ? t->encoderVelocity[channel] : 0;
}

int16_t BBRDigitalExpander::channelModeMaskCached() {
    if (_channelModeMask < 0) {
        int16_t v = readRegister(BBR_REG_CHANNEL_MODE);
        if (v < 0) return -1;
        _channelModeMask = v;
    }
    succeed();
    return _channelModeMask;
}

int32_t BBRDigitalExpander::pulseWidthUs(uint8_t channel) {
    if (!checkChannel(channel)) return 0;
    int16_t mask = channelModeMaskCached();
    if (mask < 0) return 0;
    if ((mask & (1 << channel)) == 0) {
        fail(BBRStatus::WrongChannelMode,
             F("that channel is still in quadrature mode — call setChannelMode(ch, "
               "BBRChannelMode::PulseWidth) first"));
        return 0;
    }
    const BBRTelemetry *t = cachedTelemetry();
    return t ? t->encoderCount[channel] : 0;
}

bool BBRDigitalExpander::resetEncoders(uint8_t channelMask) {
    if (!checkRange(F("channel mask must be 0-15"), channelMask, 0, kAllChannels)) return false;
    return runCommand(BBR_CMD_RESET_ENCODERS, channelMask, 10);
}

bool BBRDigitalExpander::resetEncoder(uint8_t channel) {
    if (!checkChannel(channel)) return false;
    return resetEncoders((uint8_t)(1u << channel));
}

bool BBRDigitalExpander::resetAllEncoders() {
    return resetEncoders(kAllChannels);
}

bool BBRDigitalExpander::setChannelModes(uint8_t pwmMask) {
    if (!checkRange(F("channel mode mask must be 0-15"), pwmMask, 0, kAllChannels)) return false;
    if (!writeVerified(BBR_REG_CHANNEL_MODE, pwmMask, F("the expander refused the channel "
                                                        "mode mask"))) {
        return false;
    }
    _channelModeMask = pwmMask;
    _telemetryValid = false;
    return true;
}

bool BBRDigitalExpander::setChannelMode(uint8_t channel, BBRChannelMode mode) {
    if (!checkChannel(channel)) return false;
    int16_t mask = readRegister(BBR_REG_CHANNEL_MODE);
    if (mask < 0) return false;
    uint8_t bit = (uint8_t)(1u << channel);
    return setChannelModes(mode == BBRChannelMode::PulseWidth ? (uint8_t)(mask | bit)
                                                             : (uint8_t)(mask & ~bit));
}

bool BBRDigitalExpander::getChannelMode(uint8_t channel, BBRChannelMode &out) {
    if (!checkChannel(channel)) return false;
    int16_t mask = readRegister(BBR_REG_CHANNEL_MODE);
    if (mask < 0) return false;
    _channelModeMask = mask;
    out = (mask & (1 << channel)) ? BBRChannelMode::PulseWidth : BBRChannelMode::Quadrature;
    return succeed();
}

bool BBRDigitalExpander::setPwmWrapMask(uint8_t mask) {
    if (!checkRange(F("wrap mask must be 0-15"), mask, 0, kAllChannels)) return false;
    return writeVerified(BBR_REG_PWM_WRAP_MASK, mask,
                         F("the expander refused the wrap mask — are the PWM parameters "
                           "set for those channels?"));
}

bool BBRDigitalExpander::setPwmWrapEnabled(uint8_t channel, bool enabled) {
    if (!checkChannel(channel)) return false;
    int16_t mask = readRegister(BBR_REG_PWM_WRAP_MASK);
    if (mask < 0) return false;
    uint8_t bit = (uint8_t)(1u << channel);
    return setPwmWrapMask(enabled ? (uint8_t)(mask | bit) : (uint8_t)(mask & ~bit));
}

bool BBRDigitalExpander::setPwmChannelParams(uint8_t channel, uint16_t minUs, uint16_t maxUs) {
    if (!checkChannel(channel)) return false;
    if (!checkRange(F("minUs must be 1 or more"), minUs, 1, BBR_PWM_WIDTH_MAX_US - 1)) {
        return false;
    }
    if (!checkRange(F("maxUs must be greater than minUs"), maxUs, (int32_t)minUs + 1,
                    BBR_PWM_WIDTH_MAX_US)) {
        return false;
    }
    // Read-modify-write: the window holds all four channels' calibration.
    if (!runCommand(BBR_CMD_PWM_PARAMS_LOAD, 0, 10)) return false;
    uint8_t w[BBR_CFGWIN_SIZE];
    if (!readRegisters(BBR_CFGWIN_BASE, w, sizeof(w))) return false;
    put16(&w[BBR_PWMWIN_PWM0_MIN_US - BBR_CFGWIN_BASE + 4 * channel], minUs);
    put16(&w[BBR_PWMWIN_PWM0_MAX_US - BBR_CFGWIN_BASE + 4 * channel], maxUs);
    if (!writeRegisters(BBR_CFGWIN_BASE, w, sizeof(w))) return false;
    return runCommand(BBR_CMD_PWM_PARAMS_STORE, 0, 10);
}

bool BBRDigitalExpander::getPwmChannelParams(uint8_t channel, uint16_t &minUs,
                                             uint16_t &maxUs) {
    if (!checkChannel(channel)) return false;
    if (!runCommand(BBR_CMD_PWM_PARAMS_LOAD, 0, 10)) return false;
    uint8_t w[BBR_CFGWIN_SIZE];
    if (!readRegisters(BBR_CFGWIN_BASE, w, sizeof(w))) return false;
    minUs = le16(&w[BBR_PWMWIN_PWM0_MIN_US - BBR_CFGWIN_BASE + 4 * channel]);
    maxUs = le16(&w[BBR_PWMWIN_PWM0_MAX_US - BBR_CFGWIN_BASE + 4 * channel]);
    return succeed();
}

bool BBRDigitalExpander::setEncoderDirection(uint8_t channel,
                                            BBREncoderDirection direction) {
    if (!checkChannel(channel)) return false;
    int16_t mask = getEncoderInvertMask();
    if (mask < 0) return false;
    uint8_t bit = (uint8_t)(1u << channel);
    if (!setEncoderInvertMask(direction == BBREncoderDirection::Reverse
                                  ? (uint8_t)(mask | bit)
                                  : (uint8_t)(mask & ~bit))) {
        return false;
    }
    return autoSaveIfDirty();
}

bool BBRDigitalExpander::getEncoderDirection(uint8_t channel,
                                            BBREncoderDirection &out) {
    if (!checkChannel(channel)) return false;
    int16_t mask = getEncoderInvertMask();
    if (mask < 0) return false;
    out = (mask & (1 << channel)) ? BBREncoderDirection::Reverse
                                  : BBREncoderDirection::Forward;
    return succeed();
}

bool BBRDigitalExpander::setEncoderInvertMask(uint8_t mask) {
    if (!checkRange(F("invert mask must be 0-15"), mask, 0, kAllChannels)) return false;
    return writeVerified(BBR_REG_ENC_INVERT_MASK, mask,
                         F("the expander refused the encoder invert mask"));
}

int16_t BBRDigitalExpander::getEncoderInvertMask() {
    return readRegister(BBR_REG_ENC_INVERT_MASK);
}

// ------------------------------------------------------------------- sensors

bool BBRDigitalExpander::sensorConnected(uint8_t port) {
    if (!checkPort(port)) return false;
    const BBRTelemetry *t = cachedTelemetry();
    return t && t->sensorPresent(port);
}

BBRSensorType BBRDigitalExpander::sensorType(uint8_t port) {
    if (!checkPort(port)) return BBRSensorType::Unknown;
    const BBRTelemetry *t = cachedTelemetry();
    if (!t) return BBRSensorType::Unknown;
    switch (t->sensor[port].type) {
        case BBR_STYPE_EMPTY: return BBRSensorType::Empty;
        case BBR_STYPE_COLOR: return BBRSensorType::Color;
        case BBR_STYPE_DISTANCE: return BBRSensorType::Distance;
        default: return BBRSensorType::Unknown;
    }
}

/** Fail when the port definitely holds the wrong kind of sensor — that is a
 *  wiring mix-up, not a reading, and silently answering would hide it. */
bool BBRDigitalExpander::rejectWrongSensorType(uint8_t port, const BBRTelemetry &t,
                                               uint8_t wrongType,
                                               const __FlashStringHelper *why) {
    if (t.sensorPresent(port) && t.sensor[port].type == wrongType) {
        return fail(BBRStatus::WrongSensorType, why);
    }
    return true;
}

uint8_t BBRDigitalExpander::colorClass(uint8_t port) {
    if (!checkPort(port)) return BBR_CLASS_NO_MATCH;
    const BBRTelemetry *t = cachedTelemetry();
    if (!t) return BBR_CLASS_NO_MATCH;
    if (!rejectWrongSensorType(port, *t, BBR_STYPE_DISTANCE,
                               F("that port has a distance sensor but a colour was asked "
                                 "for — check which port the sensor is plugged into"))) {
        return BBR_CLASS_NO_MATCH;
    }
    return t->sensor[port].colorClass;
}

bool BBRDigitalExpander::seesColor(uint8_t port, uint8_t classSlot) {
    if (!checkSlot(classSlot)) return false;
    return colorClass(port) == classSlot;
}

float BBRDigitalExpander::distanceMm(uint8_t port) {
    if (!checkPort(port)) return INFINITY;
    const BBRTelemetry *t = cachedTelemetry();
    if (!t) return INFINITY;
    if (!rejectWrongSensorType(port, *t, BBR_STYPE_COLOR,
                               F("that port has a colour sensor but a distance was asked "
                                 "for — check which port the sensor is plugged into"))) {
        return INFINITY;
    }
    if (!t->distanceValid(port)) return INFINITY;
    return (float)t->sensor[port].distance.distanceMm;
}

bool BBRDigitalExpander::selectSensorClass(uint8_t port, uint8_t slot) {
    if (!writeVerified(BBR_REG_CFG_SENSOR_SELECT, port,
                       F("the expander refused the sensor port selection"))) {
        return false;
    }
    return writeVerified(BBR_REG_CFG_CLASS_SELECT, slot,
                         F("the expander refused the class slot selection"));
}

bool BBRDigitalExpander::writeConfigWindow(const uint8_t *data, uint8_t len) {
    return writeRegisters(BBR_CFGWIN_BASE, data, len);
}

bool BBRDigitalExpander::teachColor(uint8_t sensorPort, uint8_t classSlot) {
    if (!checkPort(sensorPort) || !checkSlot(classSlot)) return false;
    if (!selectSensorClass(sensorPort, classSlot)) return false;
    if (!runCommand(BBR_CMD_SENSOR_CAPTURE, 0, BBR_CMD_SENSOR_CAPTURE_MAX_MS)) return false;
    if (!runCommand(BBR_CMD_CFG_STORE, 0, 10)) return false;
    return autoSaveIfDirty();
}

bool BBRDigitalExpander::writeColorClass(uint8_t port, uint8_t slot, const BBRColorClass &c) {
    if (!checkPort(port) || !checkSlot(slot)) return false;
    // Chromaticity is normalized to 0-1000 per channel, so a window outside
    // that can never match and an inverted one can never be entered.
    if (c.rMax > BBR_CHROMA_SCALE || c.gMax > BBR_CHROMA_SCALE || c.bMax > BBR_CHROMA_SCALE
        || c.rMin > c.rMax || c.gMin > c.gMax || c.bMin > c.bMax) {
        return fail(BBRStatus::BadArgument,
                    F("colour window bounds must be 0-1000 with min no greater than max"));
    }
    if (c.proxMax > BBR_PROX_MAX_VALUE || c.proxMin > c.proxMax) {
        return fail(BBRStatus::BadArgument,
                    F("proximity window must be 0-2047 with min no greater than max"));
    }
    if (!selectSensorClass(port, slot)) return false;
    uint8_t w[BBR_CFGWIN_SIZE];
    memset(w, 0, sizeof(w));
    w[BBR_CFGWIN_ENABLED - BBR_CFGWIN_BASE] = 1;
    w[BBR_CFGWIN_FLAGS - BBR_CFGWIN_BASE] = c.proximityGate ? BBR_CLASSF_PROX_GATE : 0;
    put16(&w[BBR_CFGWIN_R_MIN - BBR_CFGWIN_BASE], c.rMin);
    put16(&w[BBR_CFGWIN_R_MAX - BBR_CFGWIN_BASE], c.rMax);
    put16(&w[BBR_CFGWIN_G_MIN - BBR_CFGWIN_BASE], c.gMin);
    put16(&w[BBR_CFGWIN_G_MAX - BBR_CFGWIN_BASE], c.gMax);
    put16(&w[BBR_CFGWIN_B_MIN - BBR_CFGWIN_BASE], c.bMin);
    put16(&w[BBR_CFGWIN_B_MAX - BBR_CFGWIN_BASE], c.bMax);
    put16(&w[BBR_CFGWIN_PROX_MIN - BBR_CFGWIN_BASE], c.proxMin);
    put16(&w[BBR_CFGWIN_PROX_MAX - BBR_CFGWIN_BASE], c.proxMax);
    if (!writeConfigWindow(w, sizeof(w))) return false;
    return runCommand(BBR_CMD_CFG_STORE, 0, 10);
}

bool BBRDigitalExpander::writeDistanceClass(uint8_t port, uint8_t slot,
                                            const BBRDistanceClass &c) {
    if (!checkPort(port) || !checkSlot(slot)) return false;
    if (!checkRange(F("distMax must be at least distMin and below 65535"), c.distMax,
                    c.distMin, BBR_DISTANCE_INVALID - 1)) {
        return false;
    }
    if (!selectSensorClass(port, slot)) return false;
    uint8_t w[BBR_CFGWIN_SIZE];
    memset(w, 0, sizeof(w));
    w[BBR_CFGWIN_ENABLED - BBR_CFGWIN_BASE] = 1;
    w[BBR_CFGWIN_FLAGS - BBR_CFGWIN_BASE] = 1;  // require a valid RANGE_STATUS
    put16(&w[BBR_CFGWIN_DIST_MIN - BBR_CFGWIN_BASE], c.distMin);
    put16(&w[BBR_CFGWIN_DIST_MAX - BBR_CFGWIN_BASE], c.distMax);
    put16(&w[BBR_CFGWIN_HYSTERESIS - BBR_CFGWIN_BASE], c.hysteresisMm);
    put16(&w[BBR_CFGWIN_MIN_SIGNAL_RATE - BBR_CFGWIN_BASE], c.minSignalRate);
    if (!writeConfigWindow(w, sizeof(w))) return false;
    return runCommand(BBR_CMD_CFG_STORE, 0, 10);
}

// ----------------------------------------------------------- digital outputs

bool BBRDigitalExpander::configureOutput(uint8_t output, const BBROutputConfig &c) {
    if (!checkOutput(output)) return false;
    switch (c.source) {
        case BBROutputSource::SensorClass:
            if (!checkPort(c.srcIndex) || !checkSlot(c.classIndex)) return false;
            break;
        case BBROutputSource::Encoder:
            if (!checkChannel(c.srcIndex)) return false;
            break;
        case BBROutputSource::ImuHeading:
            // threshMin > threshMax is legal here: that is how the firmware is
            // told the window crosses +/-180. Only the ends are checked.
            if (!checkRange(F("heading window must be within +/-180 degrees"), c.threshMin,
                            BBR_YAW_MIN_CDEG, BBR_YAW_MAX_CDEG)
                || !checkRange(F("heading window must be within +/-180 degrees"), c.threshMax,
                               BBR_YAW_MIN_CDEG, BBR_YAW_MAX_CDEG)) {
                return false;
            }
            break;
        default:
            break;
    }
    if (!writeVerified(BBR_REG_DOUT_SELECT, output,
                       F("the expander refused the output selection"))) {
        return false;
    }
    uint8_t w[BBR_DOUTWIN_SIZE];
    memset(w, 0, sizeof(w));
    w[BBR_DOUTWIN_SRC_TYPE - BBR_DOUTWIN_BASE] = (uint8_t)c.source;
    w[BBR_DOUTWIN_SRC_INDEX - BBR_DOUTWIN_BASE] = c.srcIndex;
    w[BBR_DOUTWIN_CLASS_INDEX - BBR_DOUTWIN_BASE] = c.classIndex;
    w[BBR_DOUTWIN_MODE - BBR_DOUTWIN_BASE] =
        (uint8_t)((c.activeLow ? BBR_DOUT_MODE_ACTIVE_LOW : 0) | ((uint8_t)c.mode << 1));
    w[BBR_DOUTWIN_DEBOUNCE_ASSERT - BBR_DOUTWIN_BASE] = c.debounceAssert;
    w[BBR_DOUTWIN_DEBOUNCE_RELEASE - BBR_DOUTWIN_BASE] = c.debounceRelease;
    w[BBR_DOUTWIN_PULSE_MS - BBR_DOUTWIN_BASE] = c.pulseMs;
    put32(&w[BBR_DOUTWIN_THRESH_MIN - BBR_DOUTWIN_BASE], (uint32_t)c.threshMin);
    put32(&w[BBR_DOUTWIN_THRESH_MAX - BBR_DOUTWIN_BASE], (uint32_t)c.threshMax);
    if (!writeRegisters(BBR_DOUTWIN_BASE, w, sizeof(w))) return false;
    return runCommand(BBR_CMD_DOUT_STORE, 0, 10);
}

bool BBRDigitalExpander::attachOutputToClass(uint8_t output, uint8_t sensorPort,
                                             uint8_t classSlot) {
    BBROutputConfig c;
    c.source = BBROutputSource::SensorClass;
    c.srcIndex = sensorPort;
    c.classIndex = classSlot;
    return configureOutput(output, c);
}

bool BBRDigitalExpander::triggerOnColor(uint8_t output, uint8_t sensorPort,
                                        uint8_t classSlot) {
    if (!checkOutput(output) || !checkPort(sensorPort) || !checkSlot(classSlot)) return false;
    if (!attachOutputToClass(output, sensorPort, classSlot)) return false;
    return autoSaveIfDirty();
}

bool BBRDigitalExpander::triggerWhenNear(uint8_t output, uint8_t sensorPort, uint16_t maxMm) {
    if (!checkOutput(output) || !checkPort(sensorPort)) return false;
    if (!checkRange(F("maxMm must be 1 or more"), maxMm, 1, BBR_DISTANCE_INVALID - 1)) {
        return false;
    }
    BBRDistanceClass cls;
    cls.distMax = maxMm;
    cls.minSignalRate = 100;  // reject weak edge-of-range returns
    // Slot 7 keeps the range window clear of slots 1-6, so it cannot collide
    // with colours someone taught on the same port.
    if (!writeDistanceClass(sensorPort, BBR_CLASS_SLOT_LAST, cls)) return false;
    if (!attachOutputToClass(output, sensorPort, BBR_CLASS_SLOT_LAST)) return false;
    return autoSaveIfDirty();
}

bool BBRDigitalExpander::triggerWhenEncoderPast(uint8_t output, uint8_t channel,
                                                int32_t counts) {
    if (!checkOutput(output) || !checkChannel(channel)) return false;
    BBROutputConfig c;
    c.source = BBROutputSource::Encoder;
    c.srcIndex = channel;
    c.threshMin = counts;
    c.threshMax = INT32_MAX;
    if (!configureOutput(output, c)) return false;
    return autoSaveIfDirty();
}

bool BBRDigitalExpander::triggerWhenFacing(uint8_t output, float headingDeg,
                                           float toleranceDeg) {
    if (!checkOutput(output)) return false;
    if (isnan(headingDeg) || isinf(headingDeg)) {
        return fail(BBRStatus::BadArgument, F("headingDeg must be a real number"));
    }
    // Written as a positive test so NaN falls through to the failure.
    if (!(toleranceDeg > 0.0f && toleranceDeg <= 180.0f)) {
        return fail(BBRStatus::BadArgument,
                    F("toleranceDeg must be more than 0 and at most 180"));
    }
    // A heading trigger on a board with no IMU would never fire.
    if (!requireImu()) return false;

    BBROutputConfig c;
    c.source = BBROutputSource::ImuHeading;
    int32_t tolCdeg = roundToI32(toleranceDeg * 100.0f);
    if (tolCdeg >= 18000) {
        // The whole circle. Written as a non-wrapping full-range window so it
        // reads back as obviously always-true rather than as a wrap window one
        // centidegree wide.
        c.threshMin = BBR_YAW_MIN_CDEG;
        c.threshMax = BBR_YAW_MAX_CDEG;
    } else {
        // Fold into (-360, 360) in float space first: a wild heading would
        // otherwise overflow the integer cast into a valid-looking window
        // pointing somewhere else entirely.
        int32_t centre = wrapCdeg(roundToI32((float)fmod((double)headingDeg, 360.0) * 100.0f));
        c.threshMin = wrapCdeg(centre - tolCdeg);
        c.threshMax = wrapCdeg(centre + tolCdeg);
    }
    if (!configureOutput(output, c)) return false;
    return autoSaveIfDirty();
}

bool BBRDigitalExpander::disableOutput(uint8_t output) {
    if (!checkOutput(output)) return false;
    BBROutputConfig c;  // defaults to Disabled
    if (!configureOutput(output, c)) return false;
    return autoSaveIfDirty();
}

uint8_t BBRDigitalExpander::outputState() {
    int16_t v = readRegister(BBR_REG_DOUT_STATE);
    return v < 0 ? 0 : (uint8_t)v;
}

uint8_t BBRDigitalExpander::outputLatched() {
    int16_t v = readRegister(BBR_REG_DOUT_LATCHED);
    return v < 0 ? 0 : (uint8_t)v;
}

bool BBRDigitalExpander::clearOutputLatches(uint8_t mask) {
    if (!checkRange(F("output mask must be 0-15"), mask, 0, kAllOutputs)) return false;
    return runCommand(BBR_CMD_DOUT_CLEAR_LATCH, mask, 10);
}

bool BBRDigitalExpander::clearOutputLatch(uint8_t output) {
    if (!checkOutput(output)) return false;
    return clearOutputLatches((uint8_t)(1u << output));
}

bool BBRDigitalExpander::clearAllOutputLatches() {
    return clearOutputLatches(kAllOutputs);
}

// ----------------------------------------------------------------------- IMU

/**
 * All-zero IMU registers would decode to a heading of exactly 0.00 that never
 * changes — the worst possible failure, because it looks like working
 * software. So every IMU path checks first and fails with a message naming
 * which of "no IMU on this board" and "IMU fitted but broken" it is.
 */
bool BBRDigitalExpander::requireImu() {
    if (!requireBegun()) return false;
    if (_imuVerified) return succeed();

    int16_t istat = readRegister(BBR_REG_IMU_STATUS);
    if (istat < 0) return false;
    if (istat == 0xFF) {
        // A released bus reads 0xFF, which would even set PRESENT and let
        // garbage through. Never treat it as a fitted IMU.
        return fail(BBRStatus::Transport,
                    F("IMU status read as 0xFF — a failed transfer, not a fitted IMU"));
    }
    bool fitted = (_capabilities & BBR_CAP_IMU) != 0;
    bool responding = ((uint8_t)istat & BBR_ISTAT_PRESENT) != 0;
    if (!fitted && !responding) {
        return fail(BBRStatus::NoImu,
                    F("this board has no IMU (base variant) — heading and odometry need "
                      "the odometry variant"));
    }
    if (fitted && !responding) {
        return fail(BBRStatus::ImuFault,
                    F("IMU fitted but not responding — a hardware fault, not a wrong "
                      "purchase"));
    }
    if (!fitted) {
        return fail(BBRStatus::ImuFault,
                    F("IMU responding on a board not strapped for one — check the variant "
                      "straps"));
    }
    _imuVerified = true;
    return succeed();
}

bool BBRDigitalExpander::readImu(BBRImuState &out) {
    // The full check runs until it passes once; after that the fitted/absent
    // question cannot change, and the status byte inside the snapshot below
    // still guards every read — so heading() in a loop is one transaction.
    if (!requireImu()) return false;

    uint8_t d[32];
    if (!readRegisters(BBR_SNAP_IMU_START, d, sizeof(d))) return false;
    if (looksLikeFailedRead(d, sizeof(d))) {
        return fail(BBRStatus::Transport,
                    F("IMU read came back uniform — the transfer never reached the "
                      "expander"));
    }
    const uint8_t base = BBR_SNAP_IMU_START;
    out.status = d[BBR_REG_IMU_STATUS - base];
    if ((out.status & BBR_ISTAT_PRESENT) == 0) {
        // The IMU vanished after verification.
        _imuVerified = false;
        return fail(BBRStatus::ImuFault,
                    F("the IMU stopped answering mid-run — check power and cable routing"));
    }
    out.quatW = sle16(&d[BBR_REG_QUAT_W - base]) / (float)BBR_QUAT_SCALE;
    out.quatX = sle16(&d[BBR_REG_QUAT_X - base]) / (float)BBR_QUAT_SCALE;
    out.quatY = sle16(&d[BBR_REG_QUAT_Y - base]) / (float)BBR_QUAT_SCALE;
    out.quatZ = sle16(&d[BBR_REG_QUAT_Z - base]) / (float)BBR_QUAT_SCALE;
    out.yawDeg = sle16(&d[BBR_REG_YAW - base]) / (float)BBR_ANGLE_SCALE_CDEG;
    out.pitchDeg = sle16(&d[BBR_REG_PITCH - base]) / (float)BBR_ANGLE_SCALE_CDEG;
    out.rollDeg = sle16(&d[BBR_REG_ROLL - base]) / (float)BBR_ANGLE_SCALE_CDEG;
    for (uint8_t i = 0; i < 3; i++) {
        out.gyroDps[i] =
            sle16(&d[BBR_REG_GYRO_X - base + 2 * i]) / (float)BBR_GYRO_SCALE_LSB_PER_DPS;
        out.accelMps2[i] =
            sle16(&d[BBR_REG_ACCEL_X - base + 2 * i]) / (float)BBR_ACCEL_SCALE_LSB_PER_MPS2;
    }
    out.calibGrade = d[BBR_REG_IMU_CALIB - base];
    out.timestampMicros = ule32(&d[BBR_REG_IMU_TIMESTAMP - base]);
    return succeed();
}

float BBRDigitalExpander::heading() {
    BBRImuState s;
    if (!readImu(s)) return NAN;
    return s.yawDeg;
}

bool BBRDigitalExpander::resetHeading() {
    if (!requireImu()) return false;
    return runCommand(BBR_CMD_IMU_RESET_HEADING, 0, 10);
}

bool BBRDigitalExpander::calibrateGyro() {
    if (!requireImu()) return false;
    return runCommand(BBR_CMD_CALIBRATE_GYRO, 0, BBR_CMD_CALIBRATE_GYRO_MAX_MS);
}

bool BBRDigitalExpander::setImuAxisUp(uint8_t axisUp) {
    if (!checkRange(F("axisUp must be a BBR_AXIS_* value"), axisUp, BBR_AXIS_POS_X,
                    BBR_AXIS_NEG_Z)) {
        return false;
    }
    return writeVerified(BBR_REG_IMU_AXIS_UP, axisUp,
                         F("the expander refused the IMU axis-up setting"));
}

int16_t BBRDigitalExpander::getImuAxisUp() {
    return readRegister(BBR_REG_IMU_AXIS_UP);
}

// ----------------------------------------------------------------- localizer

bool BBRDigitalExpander::readLocalizerRaw(BBRLocalizerState &out) {
    if (!requireBegun()) return false;
    uint8_t d[22];
    const uint8_t base = BBR_SNAP_LOCALIZER_START;
    const uint8_t crcOff = BBR_REG_LOC_CRC16 - base;

    // I2C acknowledges bytes without verifying them, so a noise-flipped bit
    // would otherwise arrive looking like a perfectly valid pose. Re-read on
    // mismatch; give up rather than return a suspect pose.
    for (uint8_t attempt = 1;; attempt++) {
        if (!readRegisters(base, d, sizeof(d))) return false;
        if (looksLikeFailedRead(d, sizeof(d))) {
            return fail(BBRStatus::Transport,
                        F("localizer read came back uniform — the transfer never reached "
                          "the expander"));
        }
        if (crc16Profibus(d, 20) == le16(&d[crcOff])) break;
        if (attempt == 3) {
            return fail(BBRStatus::CrcMismatch,
                        F("localizer block failed its checksum three times — I2C "
                          "corruption; check wiring, cable routing and grounding"));
        }
    }

    switch (d[BBR_REG_LOC_STATUS - base]) {
        case BBR_LOC_NOT_READY: out.status = BBRLocalizerStatus::NotReady; break;
        case BBR_LOC_WARMING_UP_IMU: out.status = BBRLocalizerStatus::WarmingUpImu; break;
        case BBR_LOC_CALIBRATING_IMU: out.status = BBRLocalizerStatus::CalibratingImu; break;
        case BBR_LOC_RUNNING: out.status = BBRLocalizerStatus::Running; break;
        case BBR_LOC_FAULT_NO_IMU: out.status = BBRLocalizerStatus::FaultNoImu; break;
        default: out.status = BBRLocalizerStatus::Unknown; break;
    }
    out.flags = d[BBR_REG_LOC_FLAGS - base];
    out.velXMmPerSec = sle16(&d[BBR_REG_LOC_VEL_X - base]);
    out.velYMmPerSec = sle16(&d[BBR_REG_LOC_VEL_Y - base]);
    out.headingVelRadPerSec =
        sle16(&d[BBR_REG_LOC_VEL_H - base]) / (float)BBR_LOC_HEADING_VEL_SCALE;
    out.xMm = sle16(&d[BBR_REG_LOC_X - base]);
    out.yMm = sle16(&d[BBR_REG_LOC_Y - base]);
    out.headingRad = sle16(&d[BBR_REG_LOC_H - base]) / (float)BBR_LOC_HEADING_SCALE;
    out.timestampMicros = ule32(&d[BBR_REG_LOC_TIMESTAMP - base]);
    return succeed();
}

bool BBRDigitalExpander::readLocalizer(BBRLocalizerState &out) {
    if (!readLocalizerRaw(out)) return false;
    // Same fail-loud contract as the IMU: a localizer stuck at (0, 0, 0) looks
    // exactly like working software, so anything short of RUNNING fails.
    switch (out.status) {
        case BBRLocalizerStatus::Running:
            return succeed();
        case BBRLocalizerStatus::FaultNoImu:
            requireImu();  // produces the specific base-variant/fault message
            if (_status == BBRStatus::Ok) {
                fail(BBRStatus::ImuFault, F("localizer faulted: the IMU is unavailable"));
            }
            return false;
        case BBRLocalizerStatus::NotReady:
            return fail(BBRStatus::LocalizerNotRunning,
                        F("localizer not started — call resetLocalizerAndCalibrateImu()"));
        case BBRLocalizerStatus::WarmingUpImu:
        case BBRLocalizerStatus::CalibratingImu:
            return fail(BBRStatus::LocalizerNotRunning,
                        F("localizer still starting up — call waitForLocalizerReady() "
                          "before reading a pose"));
        default:
            return fail(BBRStatus::LocalizerNotRunning,
                        F("localizer in an unknown state — firmware newer than this "
                          "driver?"));
    }
}

bool BBRDigitalExpander::getPose(BBRPose &out) {
    BBRLocalizerState s;
    if (!readLocalizer(s)) return false;
    out = s.pose();
    return true;
}

bool BBRDigitalExpander::setLocalizerParams(const BBRLocalizerParams &p) {
    if (!checkChannel(p.portX) || !checkChannel(p.portY)) return false;
    if (p.portX == p.portY) {
        return fail(BBRStatus::BadArgument,
                    F("portX and portY are the same channel — the X and Y pods need "
                      "different encoder channels"));
    }
    // Wire unit is ticks per metre, so a millimetre figure scales by 1000.
    int32_t tpmX = roundToI32(p.ticksPerMmX * 1000.0f);
    int32_t tpmY = roundToI32(p.ticksPerMmY * 1000.0f);
    if (!checkRange(F("ticksPerMmX is out of range — measure it, do not guess"), tpmX,
                    BBR_LOC_TPM_MIN, BBR_LOC_TPM_MAX)
        || !checkRange(F("ticksPerMmY is out of range — measure it, do not guess"), tpmY,
                       BBR_LOC_TPM_MIN, BBR_LOC_TPM_MAX)) {
        return false;
    }
    int32_t scalar = roundToI32(p.imuScalar * (float)BBR_LOC_IMU_SCALAR_SCALE);
    if (!checkRange(F("imuScalar must be between 0.8 and 1.2"), scalar,
                    BBR_LOC_IMU_SCALAR_MIN, BBR_LOC_IMU_SCALAR_MAX)) {
        return false;
    }
    if (!checkRange(F("velocityIntervalMs must be 1-255"), p.velocityIntervalMs, 1, 255)) {
        return false;
    }
    uint8_t w[BBR_CFGWIN_SIZE];
    memset(w, 0, sizeof(w));
    put32(&w[BBR_LOCWIN_TPM_X - BBR_CFGWIN_BASE], (uint32_t)tpmX);
    put32(&w[BBR_LOCWIN_TPM_Y - BBR_CFGWIN_BASE], (uint32_t)tpmY);
    put16(&w[BBR_LOCWIN_TCP_X - BBR_CFGWIN_BASE], (uint16_t)(int16_t)roundToI32(p.tcpOffsetXMm));
    put16(&w[BBR_LOCWIN_TCP_Y - BBR_CFGWIN_BASE], (uint16_t)(int16_t)roundToI32(p.tcpOffsetYMm));
    put16(&w[BBR_LOCWIN_IMU_SCALAR - BBR_CFGWIN_BASE], (uint16_t)scalar);
    w[BBR_LOCWIN_PORT_X - BBR_CFGWIN_BASE] = p.portX;
    w[BBR_LOCWIN_PORT_Y - BBR_CFGWIN_BASE] = p.portY;
    w[BBR_LOCWIN_VEL_INTERVAL_MS - BBR_CFGWIN_BASE] = p.velocityIntervalMs;
    if (!writeConfigWindow(w, sizeof(w))) return false;
    return runCommand(BBR_CMD_LOC_PARAMS_STORE, 0, 10);
}

bool BBRDigitalExpander::getLocalizerParams(BBRLocalizerParams &out) {
    if (!runCommand(BBR_CMD_LOC_PARAMS_LOAD, 0, 10)) return false;
    uint8_t w[BBR_CFGWIN_SIZE];
    if (!readRegisters(BBR_CFGWIN_BASE, w, sizeof(w))) return false;
    out.ticksPerMmX = (float)(int32_t)ule32(&w[BBR_LOCWIN_TPM_X - BBR_CFGWIN_BASE]) / 1000.0f;
    out.ticksPerMmY = (float)(int32_t)ule32(&w[BBR_LOCWIN_TPM_Y - BBR_CFGWIN_BASE]) / 1000.0f;
    out.tcpOffsetXMm = sle16(&w[BBR_LOCWIN_TCP_X - BBR_CFGWIN_BASE]);
    out.tcpOffsetYMm = sle16(&w[BBR_LOCWIN_TCP_Y - BBR_CFGWIN_BASE]);
    out.imuScalar =
        le16(&w[BBR_LOCWIN_IMU_SCALAR - BBR_CFGWIN_BASE]) / (float)BBR_LOC_IMU_SCALAR_SCALE;
    out.portX = w[BBR_LOCWIN_PORT_X - BBR_CFGWIN_BASE];
    out.portY = w[BBR_LOCWIN_PORT_Y - BBR_CFGWIN_BASE];
    out.velocityIntervalMs = w[BBR_LOCWIN_VEL_INTERVAL_MS - BBR_CFGWIN_BASE];
    return succeed();
}

bool BBRDigitalExpander::resetLocalizerAndCalibrateImu() {
    if (!requireImu()) return false;
    return runCommand(BBR_CMD_LOC_RESET, 0, BBR_CMD_LOC_RESET_MAX_MS);
}

bool BBRDigitalExpander::waitForLocalizerReady(uint32_t timeoutMs) {
    const uint32_t start = millis();
    for (;;) {
        BBRLocalizerState s;
        if (!readLocalizerRaw(s)) return false;
        switch (s.status) {
            case BBRLocalizerStatus::Running:
                return succeed();
            case BBRLocalizerStatus::FaultNoImu:
                requireImu();
                if (_status == BBRStatus::Ok) {
                    fail(BBRStatus::ImuFault, F("localizer faulted: the IMU is unavailable"));
                }
                return false;
            case BBRLocalizerStatus::NotReady:
                return fail(BBRStatus::LocalizerNotRunning,
                            F("localizer not started — call "
                              "resetLocalizerAndCalibrateImu() first"));
            default:
                break;  // warming up or calibrating: keep waiting
        }
        if (millis() - start > timeoutMs) {
            return fail(BBRStatus::CommandTimeout,
                        F("the localizer did not become ready in time — was the robot "
                          "moving?"));
        }
        delay(50);
    }
}

bool BBRDigitalExpander::setPose(float xMm, float yMm, float headingRad) {
    if (!checkRange(F("pose X is outside +/-32767 mm"), roundToI32(xMm), INT16_MIN,
                    INT16_MAX)
        || !checkRange(F("pose Y is outside +/-32767 mm"), roundToI32(yMm), INT16_MIN,
                       INT16_MAX)) {
        return false;
    }
    // Normalize into [-pi, pi): fmodf lands in (-2pi, 2pi), then fold the ends
    // so exactly +pi from caller code is accepted as -pi.
    const float twoPi = 6.283185307179586f;
    float h = (float)fmod((double)headingRad, (double)twoPi);
    if (h >= (float)M_PI) h -= twoPi;
    if (h < -(float)M_PI) h += twoPi;

    uint8_t w[6];
    put16(&w[BBR_LOCWIN_POSE_X - BBR_CFGWIN_BASE], (uint16_t)(int16_t)roundToI32(xMm));
    put16(&w[BBR_LOCWIN_POSE_Y - BBR_CFGWIN_BASE], (uint16_t)(int16_t)roundToI32(yMm));
    put16(&w[BBR_LOCWIN_POSE_H - BBR_CFGWIN_BASE],
          (uint16_t)(int16_t)roundToI32(h * (float)BBR_LOC_HEADING_SCALE));
    // Written immediately before the command on purpose: the config window is
    // shared between overlays and LOC_SET_POSE reads whatever is there now.
    if (!writeConfigWindow(w, sizeof(w))) return false;
    return runCommand(BBR_CMD_LOC_SET_POSE, 0, 10);
}
