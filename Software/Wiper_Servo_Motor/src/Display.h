#pragma once
// =============================================================================
//  Display.h — ST7735S 128×160 colour TFT display driver
//
//  Layout — MENU SCREEN (shown while g_inMenu == true):
//    y=  0 –  15 : Header "WIPER MOTOR CTRL"
//    y= 16 –  55 : Item 0 — DISABLED  (red)
//    y= 56 –  95 : Item 1 — MANUAL    (green)
//    y= 96 – 135 : Item 2 — ANIMCOM   (cyan)
//    y=136 – 159 : Hint bar "Enc:scroll  Btn:enter"
//
//  Layout — STATUS SCREEN (shown while in an AppState):
//    y=  0 –  15 : Header bar — mode name + "< MENU" hint
//    y= 16 –  59 : Motor 0 section (44 px)
//                    mode-colour bar (12 px)
//                    primary value × scale-2 (16 px)
//                    detail line 1  × scale-1 (8 px)
//                    detail line 2  × scale-1 (8 px)
//    y= 60 –  61 : Divider
//    y= 62 – 105 : Motor 1 section (44 px, same layout)
//    y=106 – 107 : Divider
//    y=108 – 119 : Servo 0 bar (12 px)
//    y=120 – 127 : Servo 0 value
//    y=128 – 139 : Servo 1 bar (12 px)
//    y=140 – 147 : Servo 1 value
//    y=148 – 159 : Status/hint bar (12 px)
//
//  Usage:
//    Call Display::begin() once in setup() after hardware init.
//    From Core 1:
//      Display::drawMenu(selectedIdx)         — while in top-level navigation
//      Display::update(appState, displayState) — while inside a mode
// =============================================================================

#include <Arduino.h>
#include "Config.h"

// ---------------------------------------------------------------------------
// Top-level application states — shared with main.cpp.
// ---------------------------------------------------------------------------
enum AppState : uint8_t {
    STATE_DISABLED = 0,   // Motors coasting, read-only status display
    STATE_MANUAL   = 1,   // Local control via console commands / encoder
    STATE_ANIMCOM  = 2,   // RS485 AnimCom slave mode
    STATE_CONFIG   = 3,   // Controller configuration screen
    NUM_APP_STATES = 4
};

// ---------------------------------------------------------------------------
// Config screen data — flat snapshot of all editable settings.
// Populated by Core 1 from RAM settings; drawn by drawConfig().
// ---------------------------------------------------------------------------

// Number of config items in the scrollable list (matches main.cpp CFG_* enum).
// 9 items × 16 px + 16 px header = 160 px exactly.
static constexpr uint8_t NUM_CFG_ITEMS = 9;

struct ConfigDisplayState {
    uint8_t selectedItem;           // 0 – NUM_CFG_ITEMS-1
    bool    editMode;               // true while encoder adjusts selected item
    uint8_t nodeId;                 // RS485 station ID (0x01–0xFE)
    uint8_t mType[NUM_MOTORS];      // UiMotorType: 0=PWM%  1=VEL  2=POS
    float   velLimit[NUM_MOTORS];   // velocity cap (deg/s)
    float   travVel[NUM_MOTORS];    // traverse velocity (deg/s)
};

// ---------------------------------------------------------------------------
// Snapshot of all data needed for one display frame.
// Populated by Core 1 from volatile globals in main.cpp.
// ---------------------------------------------------------------------------
struct DisplayState {
    // Closed-loop motors — index 0 = M1, index 1 = M2
    uint8_t mode[NUM_MOTORS];          // 0=MANUAL  1=VELOCITY  2=POSITION
    bool    encoderOffline[NUM_MOTORS]; // true when AS5600 returns 0xFFFF
    float   measVel[NUM_MOTORS];       // measured angular velocity (deg/s)
    float   measPosAbs[NUM_MOTORS];    // absolute multi-turn position (deg)
    float   targetVel[NUM_MOTORS];     // velocity setpoint (deg/s)
    float   commandedVel[NUM_MOTORS];  // ramped velocity command (deg/s)
    float   targetPos[NUM_MOTORS];     // position setpoint (abs deg)
    float   posError[NUM_MOTORS];      // position error (deg)
    int16_t duty[NUM_MOTORS];          // applied PWM duty (±PWM_WRAP)

    // RC servos — index 0 = S1, index 1 = S2
    int     servoActual[NUM_SERVOS];   // current ramped output angle (0–180°)
    int     servoTarget[NUM_SERVOS];   // commanded target angle (0–180°)

    // AnimCom RS485 state (for ANIMCOM mode status bar)
    uint8_t animState;       // 0=STOP  1=MANUAL  2=RUN_AUTO
    uint8_t animPattern;     // active pattern index
    uint8_t animShowIntensity;  // show intensity 0–200 %

    // MANUAL mode cursor state (for drawing selection highlight)
    uint8_t manualSel;       // 0=M0 1=M1 2=S0 3=S1 4=EXIT
    bool    manualEdit;      // true while encoder is in adjust mode
};

// ---------------------------------------------------------------------------
// Display driver
// ---------------------------------------------------------------------------
class Display {
public:
    // Initialise SPI and the TFT controller.  Call once from setup().
    static void begin();

    // Draw the top-level mode-selection menu.
    //   selectedIdx : 0 = DISABLED, 1 = MANUAL, 2 = ANIMCOM
    // Call from Core 1 while the user is navigating the menu.
    static void drawMenu(uint8_t selectedIdx);

    // Draw (or refresh) the status screen for the active AppState.
    // Call from Core 1 at whatever rate is desired (5 Hz recommended).
    static void update(AppState state, const DisplayState& ds);

    // Draw the configuration screen.
    // Call from Core 1 while g_appState == STATE_CONFIG.
    static void drawConfig(const ConfigDisplayState& cfg);

private:
    static void _drawMotorSection(uint8_t m, uint16_t ybase, const DisplayState& ds);
    static void _drawServoRow(uint8_t idx, uint16_t ybar, const DisplayState& ds);
    static void _drawStatusBar(AppState state, const DisplayState& ds);

    // Print padded text with opaque background (eliminates clear-then-draw flicker).
    static void _txtAt(int16_t x, int16_t y, uint8_t sz, uint16_t fg, const char* str);

    // Fill a full-width band and print a small left-aligned label inside it.
    static void _colorBar(uint16_t y, uint16_t h, uint16_t bg, const char* label);

    static bool _ready;    // true after begin() completes
};
