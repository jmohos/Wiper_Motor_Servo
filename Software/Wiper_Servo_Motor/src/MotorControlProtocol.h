#pragma once
// =============================================================================
//  MotorControlProtocol.h — I2C register map shared by master and RP2040
//
//  I2C address: MCP_I2C_ADDR (0x40)
//
//  Write transactions (master → RP2040):
//    [START][MCP_I2C_ADDR W][reg][data...][STOP]
//
//  Read transactions (master → RP2040):
//    [START][MCP_I2C_ADDR W][reg][REPEATED START][MCP_I2C_ADDR R][byte][STOP]
// =============================================================================

// ---------------------------------------------------------------------------
// I2C address
// ---------------------------------------------------------------------------
#define MCP_I2C_ADDR           0x40

// ---------------------------------------------------------------------------
// Write registers
// ---------------------------------------------------------------------------

// 0x00 : Motor speed — uint8_t, motor-byte encoding:
//           0   = full reverse
//          127  = coast / idle  (MCP_SPEED_IDLE)
//          255  = full forward
#define MCP_REG_SPEED_BASE     0x00

// Speed encoding constants
#define MCP_SPEED_IDLE         127
#define MCP_SPEED_FULL_FWD     255
#define MCP_SPEED_FULL_REV     0

// 0x08 : Command register
#define MCP_REG_COMMAND        0x08

// 0x09 : Control mode
//          0x00 = MANUAL   — speed register drives motor directly
//          0x01 = VELOCITY — PID regulates angular velocity to target
#define MCP_REG_MODE           0x09
#define MCP_MODE_MANUAL        0x00
#define MCP_MODE_VELOCITY      0x01
#define MCP_MODE_POSITION      0x02

// 0x0A–0x0B : Target velocity (int16, big-endian, tenths of deg/sec)
//             e.g. 3600 = 360.0 deg/sec CW,  -3600 = 360.0 deg/sec CCW
//             Write both bytes in a single 2-byte transaction starting at 0x0A.
#define MCP_REG_TARGET_VEL_H   0x0A
#define MCP_REG_TARGET_VEL_L   0x0B

// 0x0C–0x0D : Target position (uint16, big-endian, tenths of degrees, 0–3599)
//             e.g. 900 = 90.0°,  1800 = 180.0°,  3599 = 359.9°
//             Write both bytes in a single 2-byte transaction starting at 0x0C.
#define MCP_REG_TARGET_POS_H   0x0C
#define MCP_REG_TARGET_POS_L   0x0D

// ---------------------------------------------------------------------------
// Command codes
// ---------------------------------------------------------------------------
#define MCP_CMD_ALL_STOP       0x01   // coast motor immediately
#define MCP_CMD_ENABLE         0x02   // re-enable after disable
#define MCP_CMD_DISABLE        0x03   // disable output (coast)

// ---------------------------------------------------------------------------
// Read registers
// ---------------------------------------------------------------------------

// 0x20 : Status byte (read-only)
//        bit 0 — ready (1 after successful initialisation)
//        bit 1 — magnet detected on AS5600
#define MCP_REG_STATUS         0x20

// 0x21–0x22 : AS5600 raw angle (uint16, big-endian, 0–4095 = 0°–360°)
#define MCP_REG_ANGLE_H        0x21
#define MCP_REG_ANGLE_L        0x22

// 0x23–0x24 : Measured angular velocity (int16, big-endian, tenths of deg/sec)
#define MCP_REG_MEAS_VEL_H     0x23
#define MCP_REG_MEAS_VEL_L     0x24

// ---------------------------------------------------------------------------
// Motor count
// ---------------------------------------------------------------------------
#define MCP_NUM_MOTORS         1

