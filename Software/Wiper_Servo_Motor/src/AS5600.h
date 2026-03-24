#pragma once
// =============================================================================
//  AS5600.h — Magnetic angle sensor driver
//
//  Instance-based: one object per I2C bus, so two sensors can coexist even
//  though both use the fixed I2C address 0x36.
//
//  Usage:
//    static AS5600 g_enc0(Wire);   // Motor 0 — I2C0
//    static AS5600 g_enc1(Wire1);  // Motor 1 — I2C1
//
//    g_enc0.begin(AS5600_0_SDA_PIN, AS5600_0_SCL_PIN);
//    g_enc1.begin(AS5600_1_SDA_PIN, AS5600_1_SCL_PIN);
//
//    uint16_t angle = g_enc0.readAngle();  // 0–4095
// =============================================================================

#include <Arduino.h>
#include <Wire.h>

class AS5600 {
public:
    explicit AS5600(TwoWire& bus) : _bus(bus) {}

    // Initialise the I2C bus and confirm the sensor is reachable.
    // Returns true if the sensor ACKs on the bus.
    bool begin(uint8_t sda, uint8_t scl);

    // Read the 12-bit raw angle register (0x0C / 0x0D).
    // Returns 0–4095 representing 0°–360°.
    // Returns 0xFFFF on communication error.
    uint16_t readAngle();

    // Returns true when the STATUS register reports a magnet is in range.
    bool isMagnetDetected();

private:
    TwoWire& _bus;
    static constexpr uint8_t I2C_ADDR       = 0x36;
    static constexpr uint8_t REG_STATUS     = 0x0B;  // bit5 = MD (magnet detected)
    static constexpr uint8_t REG_RAWANGLE_H = 0x0C;  // bits [11:8]
    // REG_RAWANGLE_L = 0x0D, read via two-byte request
};
