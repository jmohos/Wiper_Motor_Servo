#pragma once
// =============================================================================
//  Display.h — ST7789V2 240×280 status display
//
//  Layout (portrait, 240 wide × 280 tall):
//    y=  0 – 17  : Header bar  "WIPER MOTOR CTRL"
//    y= 18 – 19  : Divider
//    y= 20 – 89  : Motor 1 (AS5600 closed-loop)
//    y= 90 – 91  : Divider
//    y= 92 –161  : Motor 2 (AS5600 closed-loop)
//    y=162 –163  : Divider
//    y=164 –207  : Servo 1
//    y=208 –209  : Divider
//    y=210 –253  : Servo 2
//                  (26 px spare at bottom)
//
//  Each motor section shows:
//    - Colored mode bar  (YELLOW=manual, GREEN=velocity, CYAN=position)
//    - Primary value in scale-2 text (measured velocity, absolute position,
//      or raw duty depending on mode)
//    - Three scale-1 detail lines whose content varies by mode
//
//  Each servo section shows:
//    - Magenta "SN  SERVO" bar
//    - Commanded angle in scale-2 text
//
//  Usage:
//    Call Display::begin() once in Core 0 setup() after SPI/hardware init.
//    Call Display::update(state) from Core 1 at whatever rate is desired.
//    The display SPI is used exclusively from Core 1 after begin().
// =============================================================================

#include <Arduino.h>
#include "Config.h"

// Snapshot of all data the display needs for one frame.
// Populated by Core 1 from volatile globals in main.cpp.
struct DisplayState {
    // Closed-loop motors — index 0 = M1, index 1 = M2
    uint8_t mode[NUM_MOTORS];          // 0=MANUAL  1=VELOCITY  2=POSITION
    float   measVel[NUM_MOTORS];       // measured angular velocity (deg/s)
    float   measPosAbs[NUM_MOTORS];    // absolute multi-turn position (deg)
    float   targetVel[NUM_MOTORS];     // velocity setpoint (deg/s)
    float   commandedVel[NUM_MOTORS];  // ramped velocity command (deg/s)
    float   targetPos[NUM_MOTORS];     // position setpoint (abs deg)
    float   posError[NUM_MOTORS];      // position error (deg)
    int16_t duty[NUM_MOTORS];          // applied PWM duty (±4999)

    // RC servos — index 0 = S1, index 1 = S2
    int servoActual[NUM_SERVOS];       // current ramped output angle (0–180°)
    int servoTarget[NUM_SERVOS];       // commanded target angle (0–180°)
};

class Display {
public:
    // Call once from Core 0 setup() — initialises SPI and paints the skeleton.
    static void begin();

    // Call from Core 1 loop — updates only the data regions (no full redraw).
    static void update(const DisplayState& s);

private:
    static void drawMotorSection(uint8_t m, uint16_t ybase, const DisplayState& s);
    static void drawServoSection(uint8_t idx, uint16_t ybase, const DisplayState& s);
    // Clear a row then print left-aligned text in scale sz.
    static void txtAt(int16_t x, int16_t y, uint8_t sz, uint16_t fg, const char* str);
};
