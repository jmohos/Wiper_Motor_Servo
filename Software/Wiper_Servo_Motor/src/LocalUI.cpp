#include "LocalUI.h"

// Static member definitions
volatile int16_t LocalUI::_encCount   = 0;
volatile bool    LocalUI::_btnPressed = false;
volatile uint8_t LocalUI::_encHistory = 0xFF;  // invalid sentinel; seeds cleanly on first edge

void LocalUI::begin() {
    // Rotary encoder — both channels with pull-up
    pinMode(ENC_CLK_PIN, INPUT_PULLUP);
    pinMode(ENC_DT_PIN,  INPUT_PULLUP);
    pinMode(ENC_SW_PIN,  INPUT_PULLUP);

    // SPI display control lines
    pinMode(DISP_DC_PIN,  OUTPUT);
    pinMode(DISP_CS_PIN,  OUTPUT);
    pinMode(DISP_RST_PIN, OUTPUT);
    digitalWrite(DISP_CS_PIN,  HIGH);  // deselect display
    digitalWrite(DISP_RST_PIN, HIGH);  // not in reset

    // Seed encoder history with the current pin state in all four slots so
    // the first real edge enters the state machine cleanly.
    uint8_t s = ((uint8_t)digitalRead(ENC_CLK_PIN) << 1)
               | (uint8_t)digitalRead(ENC_DT_PIN);
    _encHistory = (s << 6) | (s << 4) | (s << 2) | s;

    // Attach interrupts — fire on any edge of either encoder channel
    attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), _encISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN),  _encISR, CHANGE);

    // Button: fire on falling edge (HIGH → LOW = press, active-low)
    attachInterrupt(digitalPinToInterrupt(ENC_SW_PIN),  _btnISR, FALLING);
}

int16_t LocalUI::encoderDelta() {
    noInterrupts();
    int16_t delta = _encCount;
    _encCount = 0;
    interrupts();
    return delta;
}

bool LocalUI::buttonPressed() {
    noInterrupts();
    bool pressed = _btnPressed;
    _btnPressed = false;
    interrupts();
    return pressed;
}

// ---------------------------------------------------------------------------
// Encoder ISR — full quadrature cycle detector.
//
// Pin states (both INPUT_PULLUP, active-low):
//   state = (CLK<<1)|DT   — 4 possible values: 0–3
//   Detent (rest) = 3     — both pins HIGH
//   CW  detent-to-detent: 3 → 2 → 0 → 1 → 3
//   CCW detent-to-detent: 3 → 1 → 0 → 2 → 3
//
// _encHistory is an 8-bit rolling buffer of the last four 2-bit states,
// oldest in bits 7:6, newest in bits 1:0.
//
// A count is emitted only when the sequence returns to the detent (state=3)
// AND the last four states spell out a valid complete cycle:
//   CW  pattern: 2,0,1,3 → 0b10_00_01_11 = 0x87
//   CCW pattern: 1,0,2,3 → 0b01_00_10_11 = 0x4B
//
// Bounced or partial transitions produce neither pattern, so they are
// silently discarded.
// ---------------------------------------------------------------------------
void LocalUI::_encISR() {
    uint8_t state = ((uint8_t)digitalRead(ENC_CLK_PIN) << 1)
                  |  (uint8_t)digitalRead(ENC_DT_PIN);

    // Ignore repeated samples (both pins unchanged since last interrupt).
    if (state == (_encHistory & 0x03)) return;

    // Shift new state into the rolling history.
    _encHistory = (_encHistory << 2) | state;

    // Only evaluate at the detent — this is the natural debounce gate.
    if (state != 3) return;

    if      (_encHistory == 0x87) _encCount++;   // CW  complete cycle
    else if (_encHistory == 0x4B) _encCount--;   // CCW complete cycle
    // Any other pattern at state=3 means noise/bounce — ignore.
}

// ---------------------------------------------------------------------------
// Button ISR — debounced via 50 ms dead-time after each accepted press.
// ---------------------------------------------------------------------------
void LocalUI::_btnISR() {
    static uint32_t lastUs = 0;
    uint32_t now = micros();
    if (now - lastUs < 50000UL) return;   // 50 ms debounce window
    lastUs = now;
    _btnPressed = true;
}
