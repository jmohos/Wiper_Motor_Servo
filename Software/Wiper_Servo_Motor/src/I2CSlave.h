#pragma once
// =============================================================================
//  I2CSlave.h — I2C1 slave implementing the MotorControlProtocol register map
//
//  Listens on Wire1 (I2C1) at MCP_I2C_ADDR (0x40).
//  onReceive: parses register address + data, applies speed / commands.
//  onRequest: returns register contents (status, AS5600 angle).
//
//  Callbacks run in interrupt context on Core 0.
// =============================================================================

#include <Arduino.h>
#include "Config.h"
#include "MotorControlProtocol.h"

class I2CSlave {
public:
    // Initialise Wire1 as I2C slave.  Call once during setup().
    static void begin();

    // Timestamp of the last valid speed-write command (millis()).
    // Read by Core 1 watchdog; volatile ensures visibility across cores.
    static volatile uint32_t lastCmdMs;

    // Set to true the first time a speed command is received over I2C.
    // Core 1 only enforces the watchdog after this flag is set, so
    // console-only operation is unaffected.
    static volatile bool i2cMasterSeen;

    // -----------------------------------------------------------------------
    // Shared state — written by main loop (Core 0), read by ISR (Core 0).
    // Also written by ISR when the I2C master sends new commands.
    // -----------------------------------------------------------------------

    // AS5600 raw angle (0–4095).  Written by main loop.
    static volatile uint16_t encoderAngle;

    // Measured angular velocity in tenths of deg/sec (e.g. 3600 = 360.0°/s).
    // Written by main loop, readable by I2C master.
    static volatile int16_t measVelTenths;

    // Control mode commanded by I2C master (MCP_MODE_MANUAL / MCP_MODE_VELOCITY).
    // Read by main loop.
    static volatile uint8_t cmdMode;

    // Target velocity commanded by I2C master (tenths of deg/sec, signed).
    // Read by main loop.
    static volatile int16_t cmdTargetVelTenths;

    // Target position commanded by I2C master (tenths of degrees, 0–3599).
    // Read by main loop when cmdMode == MCP_MODE_POSITION.
    static volatile uint16_t cmdTargetPosTenths;

    // True when the I2C master has written a new mode or velocity command.
    // Cleared by the main loop after it consumes the command.
    static volatile bool newI2CCmd;

private:
    static volatile uint8_t _pendingReg;

    static void _onReceive(int numBytes);
    static void _onRequest();
};
