#pragma once
// =============================================================================
//  Settings.h — Non-volatile storage for PID gains, calibration, and UI config
//
//  Uses the RP2040 EEPROM emulation (flash-backed) provided by the
//  Earle Philhower arduino-pico core.
//
//  Workflow:
//    Power-up : Settings::begin() loads saved values; if none found,
//               factory defaults are applied (but NOT auto-saved).
//    Testing  : Change gains/cal in RAM via console commands freely.
//    Persist  : Type "save" to write current RAM values to flash.
//    Restore  : Type "load" to discard RAM changes and reload from flash.
//    Reset    : Type "defaults" to apply factory defaults (then "save" to keep).
//
//  NvmSettings layout (v3):
//    Header (8 bytes) + MotorSettings[NUM_MOTORS] (52 bytes each)
//    Total for 2 motors: 8 + 104 = 112 bytes — within the 256-byte allocation.
// =============================================================================

#include <Arduino.h>
#include "Config.h"

// Bump NVM_VERSION whenever the NvmSettings struct layout changes.
// Mismatched version causes load() to reject stale data and apply defaults.
static constexpr uint32_t NVM_MAGIC   = 0x57504944u;  // "WPID"
static constexpr uint8_t  NVM_VERSION = 3;             // bumped: nodeId + uiType + velLimit
static constexpr int      NVM_ADDR    = 0;
static constexpr int      NVM_SIZE    = 256;           // bytes allocated in flash

// Motor UI type — controls how the encoder adjusts the motor in MANUAL mode.
enum UiMotorType : uint8_t {
    UI_PWM_PERCENT = 0,  // encoder changes raw duty (±1% steps)
    UI_VELOCITY    = 1,  // encoder changes velocity setpoint (deg/s steps)
    UI_POSITION    = 2,  // encoder changes position setpoint (deg steps)
};

struct NvmSettings {
    uint32_t magic;
    uint8_t  version;
    uint8_t  nodeId;      // RS485 station ID (0x01–0xFE)
    uint8_t  _pad[2];     // align MotorSettings array to 4-byte boundary

    // Per-motor settings block — one entry per motor.
    struct MotorSettings {
        uint8_t  posPathMode;   // 0=shortest arc, 1=constrained
        uint8_t  uiType;        // UiMotorType — encoder mode in MANUAL
        uint8_t  _mpad[2];      // align floats to 4-byte boundary

        // Velocity PID
        float velKp, velKi, velKd;
        float velAccel;         // setpoint ramp rate (deg/s²)
        float velLimit;         // absolute velocity cap (deg/s) — VELOCITY and POSITION modes

        // Position PID
        float posKp, posKi, posKd;
        float traverseVel;      // position profile traverse speed (deg/s)
        float posMin, posMax;   // constrained travel limits (deg)

        // Calibration
        float zeroOffset;       // AS5600 reading (deg) at true mech. zero
    } motor[NUM_MOTORS];
};

class Settings {
public:
    static NvmSettings s;   // working copy (RAM); push to flash with save()

    // Call once in setup() after hardware init.
    // Loads from flash if valid; otherwise applies factory defaults.
    static void begin();

    // Load from flash into s.  Returns true if data was valid.
    static bool load();

    // Write s to flash.
    static void save();

    // Populate s with factory defaults (does NOT write to flash).
    static void defaults();
};
