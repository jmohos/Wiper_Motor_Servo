#include "MotorPWM.h"

uint8_t  MotorPWM::speeds [NUM_MOTORS] = {};
int16_t  MotorPWM::rawDuty[NUM_MOTORS] = {};

// Pin table — indexed by motor number, matches MOTOR_x_IN1/IN2_PIN in Config.h.
static const uint8_t kIn1[NUM_MOTORS] = { MOTOR_0_IN1_PIN, MOTOR_1_IN1_PIN };
static const uint8_t kIn2[NUM_MOTORS] = { MOTOR_0_IN2_PIN, MOTOR_1_IN2_PIN };

void MotorPWM::begin() {
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_WRAP);

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(kIn1[i], OUTPUT);
        pinMode(kIn2[i], OUTPUT);
        analogWrite(kIn1[i], 0);
        analogWrite(kIn2[i], 0);
    }
}

// ---------------------------------------------------------------------------
// setMotorDuty — high-resolution PID output path
// duty: -PWM_WRAP … +PWM_WRAP  (positive = forward, negative = reverse)
// ---------------------------------------------------------------------------
void MotorPWM::setMotorDuty(uint8_t idx, int16_t duty) {
    if (idx >= NUM_MOTORS) return;

    if      (duty >  (int16_t)PWM_WRAP) duty =  (int16_t)PWM_WRAP;
    else if (duty < -(int16_t)PWM_WRAP) duty = -(int16_t)PWM_WRAP;

    rawDuty[idx] = duty;

    if (duty > 0) {
        analogWrite(kIn1[idx], (uint16_t) duty);
        analogWrite(kIn2[idx], 0);
    } else if (duty < 0) {
        analogWrite(kIn1[idx], 0);
        analogWrite(kIn2[idx], (uint16_t)(-duty));
    } else {
        analogWrite(kIn1[idx], 0);
        analogWrite(kIn2[idx], 0);
    }
}

// ---------------------------------------------------------------------------
// setMotor — motor-byte uint8_t path (manual mode, I2C commands)
// Also updates rawDuty so the display is always consistent.
// ---------------------------------------------------------------------------
void MotorPWM::setMotor(uint8_t idx, uint8_t speed) {
    if (idx >= NUM_MOTORS) return;
    speeds[idx] = speed;

    int16_t  delta = (int16_t)speed - MCP_SPEED_IDLE;  // -127 … +128
    uint16_t duty  = 0;

    if (delta > 0) {
        duty = (delta >= 128) ? (uint16_t)PWM_WRAP
                              : (uint16_t)delta * PWM_WRAP / 128;
        analogWrite(kIn1[idx], duty);
        analogWrite(kIn2[idx], 0);
        rawDuty[idx] = (int16_t)duty;
    } else if (delta < 0) {
        uint16_t mag = (uint16_t)(-delta);
        duty = (mag >= 127) ? (uint16_t)PWM_WRAP
                            : mag * PWM_WRAP / 127;
        analogWrite(kIn1[idx], 0);
        analogWrite(kIn2[idx], duty);
        rawDuty[idx] = -(int16_t)duty;
    } else {
        analogWrite(kIn1[idx], 0);
        analogWrite(kIn2[idx], 0);
        rawDuty[idx] = 0;
    }
}

void MotorPWM::stopAll() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        speeds [i] = MCP_SPEED_IDLE;
        rawDuty[i] = 0;
        analogWrite(kIn1[i], 0);
        analogWrite(kIn2[i], 0);
    }
}
