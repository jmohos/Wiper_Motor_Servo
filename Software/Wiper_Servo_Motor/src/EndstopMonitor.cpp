#include "EndstopMonitor.h"

const uint8_t EndstopMonitor::_pins[NUM_MOTORS][2] = {
    { ENDSTOP_M0_A_PIN, ENDSTOP_M0_B_PIN },  // Motor 0
    { ENDSTOP_M1_A_PIN, ENDSTOP_M1_B_PIN },  // Motor 1
};

void EndstopMonitor::begin() {
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        for (uint8_t e = 0; e < 2; e++) {
            pinMode(_pins[m][e], INPUT_PULLUP);
        }
    }
}

bool EndstopMonitor::isTripped(uint8_t motorIdx, uint8_t endIdx) {
    if (motorIdx >= NUM_MOTORS || endIdx >= 2) return false;
    return digitalRead(_pins[motorIdx][endIdx]) == LOW;
}

bool EndstopMonitor::anyTripped(uint8_t motorIdx) {
    if (motorIdx >= NUM_MOTORS) return false;
    return isTripped(motorIdx, 0) || isTripped(motorIdx, 1);
}
