#include "I2CSlave.h"
#include "MotorPWM.h"
#include <Wire.h>

// Static member definitions
volatile uint32_t  I2CSlave::lastCmdMs          = 0;
volatile uint8_t   I2CSlave::_pendingReg        = 0;
volatile uint16_t  I2CSlave::encoderAngle       = 0;
volatile int16_t   I2CSlave::measVelTenths      = 0;
volatile uint8_t   I2CSlave::cmdMode            = MCP_MODE_MANUAL;
volatile int16_t   I2CSlave::cmdTargetVelTenths = 0;
volatile uint16_t  I2CSlave::cmdTargetPosTenths = 0;
volatile bool      I2CSlave::newI2CCmd          = false;
volatile bool      I2CSlave::i2cMasterSeen      = false;

void I2CSlave::begin() {
    Wire1.setSDA(I2C_SDA_PIN);
    Wire1.setSCL(I2C_SCL_PIN);
    Wire1.begin(MCP_I2C_ADDR);
    Wire1.onReceive(_onReceive);
    Wire1.onRequest(_onRequest);
}

// ---------------------------------------------------------------------------
// Called by Wire1 ISR when the master writes data.
// First byte = register address; subsequent bytes = data.
// ---------------------------------------------------------------------------
void I2CSlave::_onReceive(int numBytes) {
    if (numBytes < 1 || !Wire1.available()) return;

    uint8_t reg = Wire1.read();
    numBytes--;
    _pendingReg = reg;

    // ---- Command register ------------------------------------------------
    if (reg == MCP_REG_COMMAND) {
        if (numBytes >= 1) {
            uint8_t cmd = Wire1.read();
            if (cmd == MCP_CMD_ALL_STOP || cmd == MCP_CMD_DISABLE) {
                MotorPWM::stopAll();
                cmdMode = MCP_MODE_MANUAL;
                newI2CCmd = true;
            }
        }
    }

    // ---- Speed register (manual mode only) -------------------------------
    else if (reg == MCP_REG_SPEED_BASE) {
        if (numBytes >= 1) {
            uint8_t spd = Wire1.read();
            if (cmdMode == MCP_MODE_MANUAL) {
                MotorPWM::setMotor(0, spd);
            }
            lastCmdMs = millis();
            i2cMasterSeen = true;
        }
    }

    // ---- Control mode ----------------------------------------------------
    else if (reg == MCP_REG_MODE) {
        if (numBytes >= 1) {
            cmdMode = Wire1.read();
            newI2CCmd = true;
            lastCmdMs = millis();
            i2cMasterSeen = true;
        }
    }

    // ---- Target velocity (two-byte big-endian write starting at H reg) ---
    else if (reg == MCP_REG_TARGET_VEL_H && numBytes >= 2) {
        uint8_t hi = Wire1.read();
        uint8_t lo = Wire1.read();
        cmdTargetVelTenths = (int16_t)(((uint16_t)hi << 8) | lo);
        newI2CCmd = true;
        lastCmdMs = millis();
        i2cMasterSeen = true;
    }

    // ---- Target position (two-byte big-endian write starting at H reg) ---
    else if (reg == MCP_REG_TARGET_POS_H && numBytes >= 2) {
        uint8_t hi = Wire1.read();
        uint8_t lo = Wire1.read();
        cmdTargetPosTenths = ((uint16_t)hi << 8) | lo;
        newI2CCmd = true;
        lastCmdMs = millis();
        i2cMasterSeen = true;
    }

    // Drain any leftover bytes
    while (Wire1.available()) Wire1.read();
}

// ---------------------------------------------------------------------------
// Called by Wire1 ISR when the master issues a read request.
// ---------------------------------------------------------------------------
void I2CSlave::_onRequest() {
    switch (_pendingReg) {

        case MCP_REG_STATUS:
            Wire1.write((uint8_t)0x01);  // bit0 = ready
            break;

        case MCP_REG_ANGLE_H:
            Wire1.write((uint8_t)(encoderAngle >> 8));
            break;

        case MCP_REG_ANGLE_L:
            Wire1.write((uint8_t)(encoderAngle & 0xFF));
            break;

        case MCP_REG_MEAS_VEL_H:
            Wire1.write((uint8_t)((uint16_t)measVelTenths >> 8));
            break;

        case MCP_REG_MEAS_VEL_L:
            Wire1.write((uint8_t)(measVelTenths & 0xFF));
            break;

        default:
            Wire1.write(0x00);
            break;
    }
}
