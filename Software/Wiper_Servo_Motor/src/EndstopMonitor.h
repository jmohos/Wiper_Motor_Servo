#pragma once
// =============================================================================
//  EndstopMonitor.h — Dual-motor endstop switch monitoring
//
//  Each motor has two limit switches (A = one end, B = other end).
//  Switches are wired active-low; a tripped switch reads LOW.
//  Internal pull-ups are enabled so no external resistors are needed.
//
//  Pin mapping (from Config.h):
//    Motor 0: End-A = GPIO8,  End-B = GPIO9
//    Motor 1: End-A = GPIO10, End-B = GPIO11
//
//  Usage:
//    EndstopMonitor::begin();                      // once in setup()
//    if (EndstopMonitor::isTripped(0, 0)) { ... }  // motor 0, end A
//    if (EndstopMonitor::anyTripped(1))   { ... }  // motor 1, either end
// =============================================================================

#include <Arduino.h>
#include "Config.h"

class EndstopMonitor {
public:
    // Configure all endstop pins as INPUT_PULLUP.  Call once in setup().
    static void begin();

    // Returns true if the specified switch is currently active (reads LOW).
    // motorIdx: 0 … NUM_MOTORS-1   endIdx: 0 = A end, 1 = B end
    static bool isTripped(uint8_t motorIdx, uint8_t endIdx);

    // Returns true if either switch on the given motor is active.
    static bool anyTripped(uint8_t motorIdx);

private:
    // _pins[motor][end] — populated from Config.h pin defines
    static const uint8_t _pins[NUM_MOTORS][2];
};
