#include "MotorPWM.h"

uint8_t  MotorPWM::speeds [NUM_MOTORS] = {};
int16_t  MotorPWM::rawDuty[NUM_MOTORS] = {};

void MotorPWM::begin() {
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_WRAP);

    for (uint8_t gpio = 0; gpio < NUM_MOTORS * 2; gpio++) {
        pinMode(gpio, OUTPUT);
        analogWrite(gpio, 0);
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

    uint8_t in1 = idx * 2;
    uint8_t in2 = idx * 2 + 1;

    if (duty > 0) {
        analogWrite(in1, (uint16_t) duty);
        analogWrite(in2, 0);
    } else if (duty < 0) {
        analogWrite(in1, 0);
        analogWrite(in2, (uint16_t)(-duty));
    } else {
        analogWrite(in1, 0);
        analogWrite(in2, 0);
    }
}

// ---------------------------------------------------------------------------
// setMotor — motor-byte uint8_t path (manual mode, I2C commands)
// Also updates rawDuty so the display is always consistent.
// ---------------------------------------------------------------------------
void MotorPWM::setMotor(uint8_t idx, uint8_t speed) {
    if (idx >= NUM_MOTORS) return;
    speeds[idx] = speed;

    uint8_t  in1   = idx * 2;
    uint8_t  in2   = idx * 2 + 1;
    int16_t  delta = (int16_t)speed - MCP_SPEED_IDLE;  // -127 … +128

    uint16_t duty  = 0;
    if (delta > 0) {
        duty = (delta >= 128) ? (uint16_t)PWM_WRAP
                              : (uint16_t)delta * PWM_WRAP / 128;
        analogWrite(in1, duty);
        analogWrite(in2, 0);
        rawDuty[idx] = (int16_t)duty;
    } else if (delta < 0) {
        uint16_t mag = (uint16_t)(-delta);
        duty = (mag >= 127) ? (uint16_t)PWM_WRAP
                            : mag * PWM_WRAP / 127;
        analogWrite(in1, 0);
        analogWrite(in2, duty);
        rawDuty[idx] = -(int16_t)duty;
    } else {
        analogWrite(in1, 0);
        analogWrite(in2, 0);
        rawDuty[idx] = 0;
    }
}

void MotorPWM::stopAll() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        speeds [i] = MCP_SPEED_IDLE;
        rawDuty[i] = 0;
        analogWrite(i * 2,     0);
        analogWrite(i * 2 + 1, 0);
    }
}

