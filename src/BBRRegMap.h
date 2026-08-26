/* BBRRegMap.h — BBR Digital Expander I2C register map constants.
 *
 * GENERATED FILE — DO NOT EDIT. Regenerated from the protocol definition
 * whenever the register map changes.
 *
 * Protocol contract 1.1. Multi-byte values are little-endian.
 * What each register means: https://expander.buildingblockrobotics.com/
 */
#ifndef BBR_REGMAP_H
#define BBR_REGMAP_H

/* ------------------------------------------------------------------
 * Protocol and device identity
 * ------------------------------------------------------------------ */

#define BBR_PROTOCOL_MAJOR_VALUE 1
#define BBR_PROTOCOL_MINOR_VALUE 1
#define BBR_DEVICE_ID_VALUE      0xB2
#define BBR_I2C_ADDR_DEFAULT     0x38  /* 7-bit */
#define BBR_I2C_ADDR_MIN         0x38
#define BBR_I2C_ADDR_MAX         0x3B

/* ------------------------------------------------------------------
 * Registers
 * ------------------------------------------------------------------ */

#define BBR_REG_DEVICE_ID              0x00  /* u8, R */
#define BBR_REG_FW_VERSION_MAJOR       0x01  /* u8, R */
#define BBR_REG_FW_VERSION_MINOR       0x02  /* u8, R */
#define BBR_REG_FW_VERSION_PATCH       0x03  /* u8, R */
#define BBR_REG_PROTOCOL_MAJOR         0x04  /* u8, R */
#define BBR_REG_PROTOCOL_MINOR         0x05  /* u8, R */
#define BBR_REG_HW_VARIANT             0x06  /* u8, R */
#define BBR_REG_CAPABILITIES           0x07  /* u8, R */
#define BBR_REG_STATUS                 0x08  /* u8, R */
#define BBR_REG_LAST_INVALID_REG       0x09  /* u8, R */
#define BBR_REG_COMMAND_ARG            0x0A  /* u8, W */
#define BBR_REG_COMMAND_TOKEN          0x0B  /* u8, RW */
#define BBR_REG_COMMAND                0x0C  /* u8, W */
#define BBR_REG_COMMAND_STATUS         0x0D  /* u8, R */
#define BBR_REG_COMMAND_RESULT         0x0E  /* u8, R */
#define BBR_REG_COMMAND_ECHO           0x0F  /* u8, R */
#define BBR_REG_ENC0_COUNT             0x10  /* i32, R */
#define BBR_REG_ENC1_COUNT             0x14  /* i32, R */
#define BBR_REG_ENC2_COUNT             0x18  /* i32, R */
#define BBR_REG_ENC3_COUNT             0x1C  /* i32, R */
#define BBR_REG_ENC0_VELOCITY          0x20  /* i32, R */
#define BBR_REG_ENC1_VELOCITY          0x24  /* i32, R */
#define BBR_REG_ENC2_VELOCITY          0x28  /* i32, R */
#define BBR_REG_ENC3_VELOCITY          0x2C  /* i32, R */
#define BBR_REG_S0_CLASS               0x30  /* u8, R */
#define BBR_REG_S1_CLASS               0x31  /* u8, R */
#define BBR_REG_S2_CLASS               0x32  /* u8, R */
#define BBR_REG_S3_CLASS               0x33  /* u8, R */
#define BBR_REG_SENSOR_PRESENT_MASK    0x34  /* u8, R */
#define BBR_REG_CLASS_CONFIDENCE       0x35  /* u8[4], R */
#define BBR_REG_CLASS_CONFIDENCE_COUNT 4
#define BBR_REG_SENSOR_STATUS          0x39  /* u8[4], R */
#define BBR_REG_SENSOR_STATUS_COUNT    4
#define BBR_REG_TELEMETRY_TIMESTAMP    0x68  /* u32, R */
#define BBR_REG_SENSOR_TYPE            0x6C  /* u8[4], R */
#define BBR_REG_SENSOR_TYPE_COUNT      4
#define BBR_REG_CFG_SENSOR_SELECT      0x70  /* u8, RW */
#define BBR_REG_CFG_CLASS_SELECT       0x71  /* u8, RW */
#define BBR_REG_CFG_GAIN               0x72  /* u8, RW */
#define BBR_REG_CFG_INTEGRATION        0x73  /* u8, RW */
#define BBR_REG_CFG_LED_CURRENT        0x74  /* u8, RW */
#define BBR_REG_ENC_INVERT_MASK        0x75  /* u8, RW */
#define BBR_REG_VEL_INTERVAL_MS        0x76  /* u8, RW */
#define BBR_REG_IMU_AXIS_UP            0x77  /* u8, RW */
#define BBR_REG_IMU_YAW_OFFSET         0x78  /* i16, RW */
#define BBR_REG_IR_COEFF_R             0x7A  /* u8, RW */
#define BBR_REG_IR_COEFF_G             0x7B  /* u8, RW */
#define BBR_REG_IR_COEFF_B             0x7C  /* u8, RW */
#define BBR_REG_CFG_MIN_SUM            0x7D  /* u16, RW */
#define BBR_REG_CFG_MEAS_RATE          0x7F  /* u8, RW */
#define BBR_REG_CHANNEL_MODE           0x9A  /* u8, RW */
#define BBR_REG_PWM_WRAP_MASK          0x9B  /* u8, RW */
#define BBR_REG_ENC_PIN_STATE          0x9C  /* u8, R */
#define BBR_REG_CFG_GENERATION         0x94  /* u32, R */
#define BBR_REG_CFG_SCHEMA_VER         0x98  /* u8, R */
#define BBR_REG_CFG_ACTIVE_SLOT        0x99  /* u8, R */
#define BBR_REG_QUAT_W                 0xA0  /* i16, R */
#define BBR_REG_QUAT_X                 0xA2  /* i16, R */
#define BBR_REG_QUAT_Y                 0xA4  /* i16, R */
#define BBR_REG_QUAT_Z                 0xA6  /* i16, R */
#define BBR_REG_YAW                    0xA8  /* i16, R */
#define BBR_REG_PITCH                  0xAA  /* i16, R */
#define BBR_REG_ROLL                   0xAC  /* i16, R */
#define BBR_REG_GYRO_X                 0xAE  /* i16, R */
#define BBR_REG_GYRO_Y                 0xB0  /* i16, R */
#define BBR_REG_GYRO_Z                 0xB2  /* i16, R */
#define BBR_REG_ACCEL_X                0xB4  /* i16, R */
#define BBR_REG_ACCEL_Y                0xB6  /* i16, R */
#define BBR_REG_ACCEL_Z                0xB8  /* i16, R */
#define BBR_REG_IMU_CALIB              0xBA  /* u8, R */
#define BBR_REG_IMU_STATUS             0xBB  /* u8, R */
#define BBR_REG_IMU_TIMESTAMP          0xBC  /* u32, R */
#define BBR_REG_DOUT_SELECT            0xC0  /* u8, RW */
#define BBR_REG_DOUT_STATE             0xC1  /* u8, R */
#define BBR_REG_DOUT_LATCHED           0xC2  /* u8, R */
#define BBR_REG_LOC_STATUS             0xE0  /* u8, R */
#define BBR_REG_LOC_FLAGS              0xE1  /* u8, R */
#define BBR_REG_LOC_VEL_X              0xE2  /* i16, R */
#define BBR_REG_LOC_VEL_Y              0xE4  /* i16, R */
#define BBR_REG_LOC_VEL_H              0xE6  /* i16, R */
#define BBR_REG_LOC_X                  0xE8  /* i16, R */
#define BBR_REG_LOC_Y                  0xEA  /* i16, R */
#define BBR_REG_LOC_H                  0xEC  /* i16, R */
#define BBR_REG_LOC_TIMESTAMP          0xF0  /* u32, R */
#define BBR_REG_LOC_CRC16              0xF4  /* u16, R */

/* ------------------------------------------------------------------
 * Raw sensor channel block: base + stride*port + field offset
 * ------------------------------------------------------------------ */

#define BBR_RAW_BASE         0x40
#define BBR_RAW_STRIDE       10
#define BBR_RAW_COUNT        4
#define BBR_RAW_RED          0  /* u16 (color overlay) */
#define BBR_RAW_GREEN        2  /* u16 (color overlay) */
#define BBR_RAW_BLUE         4  /* u16 (color overlay) */
#define BBR_RAW_IR           6  /* u16 (color overlay) */
#define BBR_RAW_PROXIMITY    8  /* u16 (color overlay) */
#define BBR_RAW_DISTANCE_MM  0  /* u16 (distance overlay) */
#define BBR_RAW_SIGNAL_RATE  2  /* u16 (distance overlay) */
#define BBR_RAW_AMBIENT_RATE 4  /* u16 (distance overlay) */
#define BBR_RAW_RANGE_STATUS 6  /* u8 (distance overlay) */

/* ------------------------------------------------------------------
 * Config data window (color / distance / localizer / pose overlays)
 * ------------------------------------------------------------------ */

#define BBR_CFGWIN_BASE            0x80
#define BBR_CFGWIN_SIZE            20
#define BBR_CFGWIN_ENABLED         0x80  /* u8 */
#define BBR_CFGWIN_FLAGS           0x81  /* u8 */
#define BBR_CFGWIN_R_MIN           0x82  /* u16 */
#define BBR_CFGWIN_R_MAX           0x84  /* u16 */
#define BBR_CFGWIN_G_MIN           0x86  /* u16 */
#define BBR_CFGWIN_G_MAX           0x88  /* u16 */
#define BBR_CFGWIN_B_MIN           0x8A  /* u16 */
#define BBR_CFGWIN_B_MAX           0x8C  /* u16 */
#define BBR_CFGWIN_PROX_MIN        0x8E  /* u16 */
#define BBR_CFGWIN_PROX_MAX        0x90  /* u16 */
#define BBR_CFGWIN_LABEL           0x92  /* u8 */
#define BBR_CFGWIN_DIST_MIN        0x82  /* u16 (distance overlay) */
#define BBR_CFGWIN_DIST_MAX        0x84  /* u16 (distance overlay) */
#define BBR_CFGWIN_HYSTERESIS      0x86  /* u16 (distance overlay) */
#define BBR_CFGWIN_MIN_SIGNAL_RATE 0x88  /* u16 (distance overlay) */
#define BBR_LOCWIN_TPM_X           0x80  /* u32 (localizer overlay) */
#define BBR_LOCWIN_TPM_Y           0x84  /* u32 (localizer overlay) */
#define BBR_LOCWIN_TCP_X           0x88  /* i16 (localizer overlay) */
#define BBR_LOCWIN_TCP_Y           0x8A  /* i16 (localizer overlay) */
#define BBR_LOCWIN_IMU_SCALAR      0x8C  /* u16 (localizer overlay) */
#define BBR_LOCWIN_PORT_X          0x8E  /* u8 (localizer overlay) */
#define BBR_LOCWIN_PORT_Y          0x8F  /* u8 (localizer overlay) */
#define BBR_LOCWIN_VEL_INTERVAL_MS 0x90  /* u8 (localizer overlay) */
#define BBR_LOCWIN_POSE_X          0x80  /* i16 (pose overlay) */
#define BBR_LOCWIN_POSE_Y          0x82  /* i16 (pose overlay) */
#define BBR_LOCWIN_POSE_H          0x84  /* i16 (pose overlay) */
#define BBR_PWMWIN_PWM0_MIN_US     0x80  /* u16 (pwm overlay) */
#define BBR_PWMWIN_PWM0_MAX_US     0x82  /* u16 (pwm overlay) */
#define BBR_PWMWIN_PWM1_MIN_US     0x84  /* u16 (pwm overlay) */
#define BBR_PWMWIN_PWM1_MAX_US     0x86  /* u16 (pwm overlay) */
#define BBR_PWMWIN_PWM2_MIN_US     0x88  /* u16 (pwm overlay) */
#define BBR_PWMWIN_PWM2_MAX_US     0x8A  /* u16 (pwm overlay) */
#define BBR_PWMWIN_PWM3_MIN_US     0x8C  /* u16 (pwm overlay) */
#define BBR_PWMWIN_PWM3_MAX_US     0x8E  /* u16 (pwm overlay) */

/* ------------------------------------------------------------------
 * Digital output config window
 * ------------------------------------------------------------------ */

#define BBR_DOUTWIN_BASE             0xD0
#define BBR_DOUTWIN_SIZE             16
#define BBR_DOUTWIN_SRC_TYPE         0xD0  /* u8 */
#define BBR_DOUTWIN_SRC_INDEX        0xD1  /* u8 */
#define BBR_DOUTWIN_CLASS_INDEX      0xD2  /* u8 */
#define BBR_DOUTWIN_MODE             0xD3  /* u8 */
#define BBR_DOUTWIN_DEBOUNCE_ASSERT  0xD4  /* u8 */
#define BBR_DOUTWIN_DEBOUNCE_RELEASE 0xD5  /* u8 */
#define BBR_DOUTWIN_PULSE_MS         0xD6  /* u8 */
#define BBR_DOUTWIN_THRESH_MIN       0xD8  /* i32 */
#define BBR_DOUTWIN_THRESH_MAX       0xDC  /* i32 */

/* ------------------------------------------------------------------
 * Commands (write opcode to REG_COMMAND; see driver sequence in the md)
 * ------------------------------------------------------------------ */

#define BBR_CMD_RESET_ENCODERS        0x01  /* sync, arg: channel mask */
#define BBR_CMD_CLEAR_FAULTS          0x02  /* sync */
#define BBR_CMD_CFG_LOAD              0x10  /* sync */
#define BBR_CMD_CFG_STORE             0x11  /* sync */
#define BBR_CMD_CFG_SAVE_FLASH        0x12  /* async */
#define BBR_CMD_CFG_LOAD_DEFAULTS     0x13  /* sync */
#define BBR_CMD_SENSOR_CAPTURE        0x20  /* async */
#define BBR_CMD_IMU_RESET_HEADING     0x30  /* sync */
#define BBR_CMD_CALIBRATE_GYRO        0x31  /* async */
#define BBR_CMD_LOC_RESET             0x32  /* async */
#define BBR_CMD_LOC_SET_POSE          0x33  /* sync */
#define BBR_CMD_LOC_PARAMS_LOAD       0x34  /* sync */
#define BBR_CMD_LOC_PARAMS_STORE      0x35  /* sync */
#define BBR_CMD_PWM_PARAMS_LOAD       0x36  /* sync */
#define BBR_CMD_PWM_PARAMS_STORE      0x37  /* sync */
#define BBR_CMD_DOUT_LOAD             0x40  /* sync */
#define BBR_CMD_DOUT_STORE            0x41  /* sync */
#define BBR_CMD_DOUT_CLEAR_LATCH      0x42  /* sync, arg: output mask */
#define BBR_CMD_REBOOT_BOOTLOADER     0xF0  /* noreturn, arg: 0x5A required */
#define BBR_CMD_CFG_SAVE_FLASH_MAX_MS 500
#define BBR_CMD_SENSOR_CAPTURE_MAX_MS 5000
#define BBR_CMD_CALIBRATE_GYRO_MAX_MS 2000
#define BBR_CMD_LOC_RESET_MAX_MS      3000
#define BBR_CMD_SYNC_MAX_US           100  /* all sync commands complete in under this */

/* ------------------------------------------------------------------
 * Command status (REG_COMMAND_STATUS)
 * ------------------------------------------------------------------ */

#define BBR_CMDSTAT_IDLE  0
#define BBR_CMDSTAT_BUSY  1
#define BBR_CMDSTAT_DONE  2
#define BBR_CMDSTAT_ERROR 3

/* ------------------------------------------------------------------
 * Error codes (REG_COMMAND_RESULT)
 * ------------------------------------------------------------------ */

#define BBR_OK                      0x00
#define BBR_ERR_UNKNOWN_COMMAND     0x01
#define BBR_ERR_BAD_ARG             0x02
#define BBR_ERR_NOT_SUPPORTED       0x03
#define BBR_ERR_TOKEN_REUSE         0x04
#define BBR_ERR_FLASH_WRITE         0x10
#define BBR_ERR_FLASH_VERIFY        0x11
#define BBR_ERR_FLASH_RATE_LIMITED  0x12
#define BBR_ERR_SCHEMA_UNSUPPORTED  0x13
#define BBR_ERR_SENSOR_ABSENT       0x20
#define BBR_ERR_SENSOR_NAK          0x21
#define BBR_ERR_SENSOR_TIMEOUT      0x22
#define BBR_ERR_SENSOR_SATURATED    0x23
#define BBR_ERR_INSUFFICIENT_SIGNAL 0x24
#define BBR_ERR_CFG_INVALID_SLOT    0x30
#define BBR_ERR_IMU_NOT_STATIONARY  0x40
#define BBR_ERR_LOC_BAD_PARAMS      0x41

/* ------------------------------------------------------------------
 * STATUS bits
 * ------------------------------------------------------------------ */

#define BBR_STATUS_READY            (1u << 0)  /* bit 0 */
#define BBR_STATUS_CFG_DIRTY        (1u << 1)  /* bit 1 */
#define BBR_STATUS_FLASH_ERROR      (1u << 2)  /* bit 2 */
#define BBR_STATUS_SENSOR_BUS_ERROR (1u << 3)  /* bit 3 */
#define BBR_STATUS_IMU_PRESENT      (1u << 4)  /* bit 4 */
#define BBR_STATUS_IMU_CALIBRATED   (1u << 5)  /* bit 5 */
#define BBR_STATUS_CMD_OVERRUN      (1u << 6)  /* bit 6 */
#define BBR_STATUS_CFG_DEFAULTED    (1u << 7)  /* bit 7 */

/* ------------------------------------------------------------------
 * CAP bits
 * ------------------------------------------------------------------ */

#define BBR_CAP_ENCODERS (1u << 0)  /* bit 0 */
#define BBR_CAP_COLOR    (1u << 1)  /* bit 1 */
#define BBR_CAP_IMU      (1u << 2)  /* bit 2 */
#define BBR_CAP_DOUT     (1u << 3)  /* bit 3 */

/* ------------------------------------------------------------------
 * SENSOR_STATUS bits
 * ------------------------------------------------------------------ */

#define BBR_SSTAT_VALID               (1u << 0)  /* bit 0 */
#define BBR_SSTAT_STALE               (1u << 1)  /* bit 1 */
#define BBR_SSTAT_QUALITY_FAIL        (1u << 2)  /* bit 2 */
#define BBR_SSTAT_OVERFLOW            (1u << 3)  /* bit 3 */
#define BBR_SSTAT_BUS_ERROR           (1u << 4)  /* bit 4 */
#define BBR_SSTAT_INSUFFICIENT_SIGNAL (1u << 5)  /* bit 5 */

/* ------------------------------------------------------------------
 * IMU_STATUS bits
 * ------------------------------------------------------------------ */

#define BBR_ISTAT_PRESENT      (1u << 0)  /* bit 0 */
#define BBR_ISTAT_FUSION_VALID (1u << 1)  /* bit 1 */
#define BBR_ISTAT_SATURATED    (1u << 2)  /* bit 2 */
#define BBR_ISTAT_BIAS_VALID   (1u << 3)  /* bit 3 */
#define BBR_ISTAT_STATIONARY   (1u << 4)  /* bit 4 */
#define BBR_ISTAT_VIBRATION    (1u << 5)  /* bit 5 */
#define BBR_ISTAT_FAULT        (1u << 6)  /* bit 6 */
#define BBR_ISTAT_UNEXPECTED   (1u << 7)  /* bit 7 */

/* ------------------------------------------------------------------
 * CLASS_FLAGS bits
 * ------------------------------------------------------------------ */

#define BBR_CLASSF_PROX_GATE (1u << 0)  /* bit 0 */

/* ------------------------------------------------------------------
 * DOUT_MODE_FLAGS bits
 * ------------------------------------------------------------------ */

#define BBR_DOUT_MODE_ACTIVE_LOW (1u << 0)  /* bit 0 */

/* ------------------------------------------------------------------
 * LOC_FLAGS bits
 * ------------------------------------------------------------------ */

#define BBR_LOCF_POSE_CLIPPED        (1u << 0)  /* bit 0 */
#define BBR_LOCF_GYRO_SATURATED_EVER (1u << 1)  /* bit 1 */
#define BBR_LOCF_PORT_CONFLICT       (1u << 2)  /* bit 2 */

/* ------------------------------------------------------------------
 * SENSOR_TYPE values
 * ------------------------------------------------------------------ */

#define BBR_STYPE_EMPTY    0
#define BBR_STYPE_COLOR    1
#define BBR_STYPE_DISTANCE 2
#define BBR_STYPE_UNKNOWN  0xFF

/* ------------------------------------------------------------------
 * HW_VARIANT values
 * ------------------------------------------------------------------ */

#define BBR_VARIANT_BASE     1
#define BBR_VARIANT_ODOMETRY 2

/* ------------------------------------------------------------------
 * GAIN values
 * ------------------------------------------------------------------ */

#define BBR_GAIN_1X  0
#define BBR_GAIN_3X  1
#define BBR_GAIN_6X  2
#define BBR_GAIN_9X  3
#define BBR_GAIN_18X 4

/* ------------------------------------------------------------------
 * INTEGRATION values
 * ------------------------------------------------------------------ */

#define BBR_INTEG_400MS_20BIT  0
#define BBR_INTEG_200MS_19BIT  1
#define BBR_INTEG_100MS_18BIT  2
#define BBR_INTEG_50MS_17BIT   3
#define BBR_INTEG_25MS_16BIT   4
#define BBR_INTEG_3MS125_13BIT 5

/* ------------------------------------------------------------------
 * MEAS_RATE values
 * ------------------------------------------------------------------ */

#define BBR_MRATE_25MS   0
#define BBR_MRATE_50MS   1
#define BBR_MRATE_100MS  2
#define BBR_MRATE_200MS  3
#define BBR_MRATE_500MS  4
#define BBR_MRATE_1000MS 5
#define BBR_MRATE_2000MS 6

/* ------------------------------------------------------------------
 * LED_CURRENT values
 * ------------------------------------------------------------------ */

#define BBR_LED_2MA5  0
#define BBR_LED_5MA   1
#define BBR_LED_10MA  2
#define BBR_LED_25MA  3
#define BBR_LED_50MA  4
#define BBR_LED_75MA  5
#define BBR_LED_100MA 6
#define BBR_LED_125MA 7

/* ------------------------------------------------------------------
 * IMU_AXIS_UP values
 * ------------------------------------------------------------------ */

#define BBR_AXIS_POS_X 0
#define BBR_AXIS_NEG_X 1
#define BBR_AXIS_POS_Y 2
#define BBR_AXIS_NEG_Y 3
#define BBR_AXIS_POS_Z 4
#define BBR_AXIS_NEG_Z 5

/* ------------------------------------------------------------------
 * IMU_CALIB values
 * ------------------------------------------------------------------ */

#define BBR_CALIB_NONE      0
#define BBR_CALIB_COARSE    1
#define BBR_CALIB_GOOD      2
#define BBR_CALIB_EXCELLENT 3

/* ------------------------------------------------------------------
 * DOUT_SRC values
 * ------------------------------------------------------------------ */

#define BBR_DOUT_SRC_DISABLED     0
#define BBR_DOUT_SRC_SENSOR_CLASS 1
#define BBR_DOUT_SRC_ENCODER      2
#define BBR_DOUT_SRC_IMU_HEADING  3

/* ------------------------------------------------------------------
 * DOUT_MODE values
 * ------------------------------------------------------------------ */

#define BBR_DOUT_MODE_LEVEL   0
#define BBR_DOUT_MODE_LATCHED 1
#define BBR_DOUT_MODE_PULSE   2

/* ------------------------------------------------------------------
 * LOC_STATUS values
 * ------------------------------------------------------------------ */

#define BBR_LOC_NOT_READY       0
#define BBR_LOC_WARMING_UP_IMU  1
#define BBR_LOC_CALIBRATING_IMU 2
#define BBR_LOC_RUNNING         3
#define BBR_LOC_FAULT_NO_IMU    4

/* ------------------------------------------------------------------
 * Snapshot regions (latched on pointer entry; see md)
 * ------------------------------------------------------------------ */

#define BBR_SNAP_TELEMETRY_START   0x10
#define BBR_SNAP_TELEMETRY_END     0x6F  /* inclusive */
#define BBR_SNAP_PERSISTENCE_START 0x94
#define BBR_SNAP_PERSISTENCE_END   0x99  /* inclusive */
#define BBR_SNAP_IMU_START         0xA0
#define BBR_SNAP_IMU_END           0xBF  /* inclusive */
#define BBR_SNAP_LOCALIZER_START   0xE0
#define BBR_SNAP_LOCALIZER_END     0xF5  /* inclusive */

/* ------------------------------------------------------------------
 * Flash persistence
 * ------------------------------------------------------------------ */

#define BBR_FLASH_MAGIC              0x43524242  /* "BBRC" little-endian */
#define BBR_FLASH_SCHEMA_VER         3
#define BBR_FLASH_HEADER_SIZE        20
#define BBR_FLASH_PAYLOAD_SIZE       706  /* at the current schema */
#define BBR_FLASH_PAYLOAD_SIZE_V1    671  /* at schema 1 */
#define BBR_FLASH_PAYLOAD_SIZE_V2    688  /* at schema 2 */
#define BBR_FLASH_SECTOR_SIZE        4096
#define BBR_FLASH_SLOT_COUNT         2
#define BBR_FLASH_SAVE_RATE_LIMIT_MS 1000

/* ------------------------------------------------------------------
 * Constants and scale factors
 * ------------------------------------------------------------------ */

#define BBR_NUM_SENSOR_PORTS         4
#define BBR_NUM_ENCODERS             4
#define BBR_NUM_DOUTS                4
#define BBR_NUM_CLASS_SLOTS          7
#define BBR_CLASS_SLOT_FIRST         1
#define BBR_CLASS_SLOT_LAST          7
#define BBR_CLASS_NO_MATCH           0
#define BBR_SENSOR_SELECT_ALL        0xFF
#define BBR_ACTIVE_SLOT_DEFAULTS     0xFF
#define BBR_TOKEN_NONE               0x00
#define BBR_CHROMA_SCALE             1000
#define BBR_PROX_MAX_VALUE           2047
#define BBR_DISTANCE_INVALID         0xFFFF
#define BBR_QUAT_SCALE               16384
#define BBR_ANGLE_SCALE_CDEG         100
#define BBR_GYRO_SCALE_LSB_PER_DPS   16
#define BBR_ACCEL_SCALE_LSB_PER_MPS2 100
#define BBR_YAW_MIN_CDEG             -18000
#define BBR_YAW_MAX_CDEG             17999
#define BBR_PITCH_MIN_CDEG           -9000
#define BBR_PITCH_MAX_CDEG           9000
#define BBR_REBOOT_BOOTLOADER_ARG    0x5A
#define BBR_LOC_HEADING_SCALE        5000
#define BBR_LOC_HEADING_VEL_SCALE    600
#define BBR_LOC_IMU_SCALAR_SCALE     10000
#define BBR_LOC_IMU_SCALAR_MIN       8000
#define BBR_LOC_IMU_SCALAR_MAX       12000
#define BBR_LOC_TPM_MIN              1000
#define BBR_LOC_TPM_MAX              10000000
#define BBR_LOC_PORT_UNASSIGNED      0xFF
#define BBR_PWM_WIDTH_MAX_US         65535
#define BBR_PWM_NO_SIGNAL_TIMEOUT_MS 100

/* ------------------------------------------------------------------
 * Power-on defaults
 * ------------------------------------------------------------------ */

#define BBR_DEFAULT_VEL_INTERVAL_MS          25
#define BBR_DEFAULT_CFG_MIN_SUM              300
#define BBR_DEFAULT_CFG_INTEGRATION          2
#define BBR_DEFAULT_DEBOUNCE_ASSERT          2
#define BBR_DEFAULT_DEBOUNCE_RELEASE         2
#define BBR_DEFAULT_IR_COEFF                 0
#define BBR_DEFAULT_CAPTURE_CHROMA_TOLERANCE 50
#define BBR_DEFAULT_LOC_VEL_INTERVAL_MS      25
#define BBR_DEFAULT_LOC_IMU_SCALAR           10000

#endif /* BBR_REGMAP_H */
