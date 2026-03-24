#include "AS5600.h"
#include "Config.h"

bool AS5600::begin(uint8_t sda, uint8_t scl) {
    _bus.setSDA(sda);
    _bus.setSCL(scl);
    _bus.begin();
    _bus.setClock(AS5600_I2C_FREQ);

    // Probe the sensor — a zero-byte write that just checks for ACK
    _bus.beginTransmission(I2C_ADDR);
    return (_bus.endTransmission() == 0);
}

uint16_t AS5600::readAngle() {
    // Point to the high byte of RAW ANGLE
    _bus.beginTransmission(I2C_ADDR);
    _bus.write(REG_RAWANGLE_H);
    if (_bus.endTransmission(false) != 0) return 0xFFFF;  // NACK

    // Request 2 bytes: high byte then low byte
    if (_bus.requestFrom((uint8_t)I2C_ADDR, (uint8_t)2) != 2) return 0xFFFF;

    uint8_t hi = _bus.read();
    uint8_t lo = _bus.read();

    // 12-bit result: bits[11:8] in hi[3:0]
    return ((uint16_t)(hi & 0x0F) << 8) | lo;
}

bool AS5600::isMagnetDetected() {
    _bus.beginTransmission(I2C_ADDR);
    _bus.write(REG_STATUS);
    if (_bus.endTransmission(false) != 0) return false;

    if (_bus.requestFrom((uint8_t)I2C_ADDR, (uint8_t)1) != 1) return false;
    uint8_t status = _bus.read();

    return (status & 0x20) != 0;  // bit 5 = MD
}
