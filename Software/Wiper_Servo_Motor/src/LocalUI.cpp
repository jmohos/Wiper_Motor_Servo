#include "LocalUI.h"

// Static member definitions
volatile int16_t LocalUI::_encCount    = 0;
volatile bool    LocalUI::_btnPressed  = false;
volatile uint8_t LocalUI::_lastEncState = 0;

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

    // Seed encoder state so the first edge produces a valid transition
    _lastEncState = ((uint8_t)digitalRead(ENC_CLK_PIN) << 1)
                  |  (uint8_t)digitalRead(ENC_DT_PIN);

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
// Gray-code ISR — called on any edge of ENC_CLK or ENC_DT.
// Looks up the (prev, curr) 2-bit transition in a 16-entry table to
// determine direction.  Invalid transitions (noise/bounce) produce 0.
// ---------------------------------------------------------------------------
void LocalUI::_encISR() {
    uint8_t clk   = (uint8_t)digitalRead(ENC_CLK_PIN);
    uint8_t dt    = (uint8_t)digitalRead(ENC_DT_PIN);
    uint8_t state = (clk << 1) | dt;
    uint8_t last  = _lastEncState;
    _lastEncState = state;

    // Standard gray-code direction table indexed by (prev<<2 | curr)
    static const int8_t table[16] = {
         0, -1, +1,  0,   // prev = 00
        +1,  0,  0, -1,   // prev = 01
        -1,  0,  0, +1,   // prev = 10
         0, +1, -1,  0    // prev = 11
    };
    _encCount += table[((last << 2) | state) & 0x0F];
}

// ---------------------------------------------------------------------------
// Button ISR — set flag on falling edge (active-low press).
// ---------------------------------------------------------------------------
void LocalUI::_btnISR() {
    _btnPressed = true;
}
