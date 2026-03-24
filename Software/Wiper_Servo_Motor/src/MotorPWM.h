#pragma once
// =============================================================================
//  MotorPWM.h — 25 kHz PWM generation for two BTS7960 motor drivers
//
//  GPIO mapping: IN1 = idx*2, IN2 = idx*2+1
//    Motor 0: IN1=GPIO0 (slice 0A), IN2=GPIO1 (slice 0B)
//    Motor 1: IN1=GPIO2 (slice 1A), IN2=GPIO3 (slice 1B)
//
//  Both slices share the same base frequency via analogWriteFreq().
//
//  Speed encoding (DMX-style uint8_t):
//    0   = full reverse
//    127 = coast / idle (MCP_SPEED_IDLE)
//    255 = full forward
// =============================================================================

#include <Arduino.h>
#include "Config.h"
#include "MotorControlProtocol.h"

class MotorPWM {
public:
    // Configure PWM outputs at 25 kHz.  Call once during setup().
    static void begin();

    // Set motor speed via DMX-style uint8_t (0=full rev, 127=coast, 255=full fwd).
    // Used by manual mode and I2C commands.
    static void setMotor(uint8_t idx, uint8_t speed);

    // Set motor directly from a signed duty cycle (-PWM_WRAP … +PWM_WRAP).
    // Positive = forward (IN1 active), negative = reverse (IN2 active), 0 = coast.
    // Used by the PID output path to preserve full PWM resolution.
    static void setMotorDuty(uint8_t idx, int16_t duty);

    // Zero all PWM outputs immediately (coast).
    static void stopAll();

    // Last commanded DMX speed (updated by setMotor/stopAll).
    static uint8_t speeds[NUM_MOTORS];

    // Last commanded signed duty cycle (updated by setMotorDuty and setMotor).
    // Range: -PWM_WRAP … +PWM_WRAP.
    static int16_t rawDuty[NUM_MOTORS];
};
