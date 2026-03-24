#include "Display.h"
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>
#include <SPI.h>

// ---------------------------------------------------------------------------
// Colors — RGB565
// ---------------------------------------------------------------------------
static constexpr uint16_t C_BG       = 0x0000;  // black background
static constexpr uint16_t C_WHITE    = 0xFFFF;  // primary value text
static constexpr uint16_t C_LGRAY    = 0xC618;  // detail line text
static constexpr uint16_t C_DIV      = 0x39E7;  // section divider
static constexpr uint16_t C_HEADER   = 0x000D;  // dark navy header
static constexpr uint16_t C_MANUAL   = 0xFFE0;  // yellow — manual mode bar
static constexpr uint16_t C_VELOCITY = 0x07E0;  // green  — velocity mode bar
static constexpr uint16_t C_POSITION = 0x07FF;  // cyan   — position mode bar
static constexpr uint16_t C_SERVO    = 0xF81F;  // magenta — servo bar

// ---------------------------------------------------------------------------
// Layout — all y-coordinates and heights in pixels (240 × 280 portrait)
// ---------------------------------------------------------------------------
static constexpr uint16_t SCR_W      = 240;

static constexpr uint16_t Y_HEADER   =   0;  static constexpr uint16_t H_HEADER = 18;
static constexpr uint16_t Y_DIV0     =  18;  static constexpr uint16_t H_DIV    =  2;
static constexpr uint16_t Y_M0       =  20;  static constexpr uint16_t H_MOTOR  = 70;
static constexpr uint16_t Y_DIV1     =  90;
static constexpr uint16_t Y_M1       =  92;
static constexpr uint16_t Y_DIV2     = 162;
static constexpr uint16_t Y_S0       = 164;  static constexpr uint16_t H_SERVO  = 44;
static constexpr uint16_t Y_DIV3     = 208;
static constexpr uint16_t Y_S1       = 210;
// Y_S1 + H_SERVO = 254 — 26 px spare at bottom

// Within a motor section (offsets relative to section ybase):
static constexpr uint8_t MO_BAR_H    = 12;  // mode-colour bar
static constexpr uint8_t MO_VAL_Y    = 14;  // primary value  (scale 2, 16 px tall)
static constexpr uint8_t MO_LINE1_Y  = 33;  // detail line 1  (scale 1,  8 px tall)
static constexpr uint8_t MO_LINE2_Y  = 43;  // detail line 2
static constexpr uint8_t MO_LINE3_Y  = 53;  // detail line 3

// Within a servo section (offsets relative to section ybase):
static constexpr uint8_t SO_BAR_H    = 12;  // servo-colour bar
static constexpr uint8_t SO_VAL_Y    = 14;  // angle value (scale 2, 16 px tall)

// ---------------------------------------------------------------------------
// TFT instance — hardware SPI0 (SCK=GPIO18, MOSI=GPIO19)
// ---------------------------------------------------------------------------
static Adafruit_ST7789 tft(&SPI, DISP_CS_PIN, DISP_DC_PIN, DISP_RST_PIN);

// Guard: prevent update() calls before begin() completes.
static bool _ready = false;

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Fill a full-width horizontal band.
static inline void hband(uint16_t y, uint16_t h, uint16_t color) {
    tft.fillRect(0, y, SCR_W, h, color);
}

// Print text with an opaque background — no separate fillRect needed.
// Adafruit GFX setTextColor(fg, bg) fills each character's background pixels
// in the same SPI pass as the foreground, eliminating the clear-then-draw flicker.
// Strings are left-padded to a fixed column width so a shorter new value
// overwrites every pixel of a longer previous one.
//   Scale 1 (6×8 px/char): 32 columns from x=4 → 192 px, well within 236 px available.
//   Scale 2 (12×16 px/char): 16 columns from x=4 → 192 px.
void Display::txtAt(int16_t x, int16_t y, uint8_t sz, uint16_t fg, const char* str) {
    char padded[36];
    snprintf(padded, sizeof(padded), (sz == 1) ? "%-32s" : "%-16s", str);
    tft.setTextSize(sz);
    tft.setTextColor(fg, C_BG);
    tft.setCursor(x, y);
    tft.print(padded);
}

// Draw a full-width coloured bar containing small left-aligned text.
static void colorBar(uint16_t y, uint16_t h, uint16_t bg, const char* label) {
    hband(y, h, bg);
    tft.setTextSize(1);
    tft.setTextColor(0x0000, bg);  // black text on coloured bar
    tft.setCursor(4, y + (h - 8) / 2);
    tft.print(label);
}

// ---------------------------------------------------------------------------
// Motor section
// ---------------------------------------------------------------------------
void Display::drawMotorSection(uint8_t m, uint16_t yb, const DisplayState& s) {
    char buf[32];

    // --- Mode bar (full width, colour-coded) ---
    uint16_t    barCol;
    const char* barLabel;
    switch (s.mode[m]) {
        case 2:  barCol = C_POSITION; barLabel = m ? "M2  POSITION" : "M1  POSITION"; break;
        case 1:  barCol = C_VELOCITY; barLabel = m ? "M2  VELOCITY" : "M1  VELOCITY"; break;
        default: barCol = C_MANUAL;   barLabel = m ? "M2  MANUAL  " : "M1  MANUAL  "; break;
    }
    colorBar(yb, MO_BAR_H, barCol, barLabel);

    // --- Primary value (scale 2) ---
    switch (s.mode[m]) {
        case 2:
            snprintf(buf, sizeof(buf), "%.1f deg", s.measPosAbs[m]);
            break;
        case 1:
            snprintf(buf, sizeof(buf), "%.1f d/s", s.measVel[m]);
            break;
        default:
            snprintf(buf, sizeof(buf), "Duty %+d", (int)s.duty[m]);
            break;
    }
    txtAt(4, yb + MO_VAL_Y, 2, C_WHITE, buf);

    // --- Three detail lines (scale 1) ---
    switch (s.mode[m]) {
        case 2:  // POSITION
            snprintf(buf, sizeof(buf), "Tgt: %.1f deg", s.targetPos[m]);
            txtAt(4, yb + MO_LINE1_Y, 1, C_LGRAY, buf);
            snprintf(buf, sizeof(buf), "Err: %+.2f deg", s.posError[m]);
            txtAt(4, yb + MO_LINE2_Y, 1, C_LGRAY, buf);
            snprintf(buf, sizeof(buf), "Vel: %.1f d/s  Dty:%+d", s.measVel[m], (int)s.duty[m]);
            txtAt(4, yb + MO_LINE3_Y, 1, C_LGRAY, buf);
            break;
        case 1:  // VELOCITY
            snprintf(buf, sizeof(buf), "Cmd: %.1f d/s", s.commandedVel[m]);
            txtAt(4, yb + MO_LINE1_Y, 1, C_LGRAY, buf);
            snprintf(buf, sizeof(buf), "Tgt: %.1f d/s", s.targetVel[m]);
            txtAt(4, yb + MO_LINE2_Y, 1, C_LGRAY, buf);
            snprintf(buf, sizeof(buf), "Abs: %.1f deg  Dty:%+d", s.measPosAbs[m], (int)s.duty[m]);
            txtAt(4, yb + MO_LINE3_Y, 1, C_LGRAY, buf);
            break;
        default:  // MANUAL
            snprintf(buf, sizeof(buf), "Abs: %.1f deg", s.measPosAbs[m]);
            txtAt(4, yb + MO_LINE1_Y, 1, C_LGRAY, buf);
            txtAt(4, yb + MO_LINE2_Y, 1, C_LGRAY, "");
            txtAt(4, yb + MO_LINE3_Y, 1, C_LGRAY, "");
            break;
    }
}

// ---------------------------------------------------------------------------
// Servo section
// ---------------------------------------------------------------------------
void Display::drawServoSection(uint8_t idx, uint16_t yb, const DisplayState& s) {
    char buf[20];

    // --- Header bar ---
    colorBar(yb, SO_BAR_H, C_SERVO, idx ? "S2  SERVO" : "S1  SERVO");

    // --- Ramped actual angle (scale 2, primary value) ---
    snprintf(buf, sizeof(buf), "%d deg", s.servoActual[idx]);
    txtAt(4, yb + SO_VAL_Y, 2, C_WHITE, buf);

    // --- Target detail line (scale 1) — blank when actual has reached target ---
    if (s.servoActual[idx] != s.servoTarget[idx]) {
        snprintf(buf, sizeof(buf), "Tgt: %d deg", s.servoTarget[idx]);
    } else {
        buf[0] = '\0';
    }
    txtAt(4, yb + SO_VAL_Y + 19, 1, C_LGRAY, buf);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void Display::begin() {
    // GPIO18 / GPIO19 are the default SPI0 SCK/TX pins in arduino-pico, which
    // matches DISP_SCK_PIN / DISP_MOSI_PIN.  Set explicitly for clarity.
    SPI.setSCK(DISP_SCK_PIN);
    SPI.setTX(DISP_MOSI_PIN);

    // ST7789V2, 240×280.
    // NOTE: Some 240×280 modules sit on a 240×320 chip and need a 20-pixel
    // row offset.  If the image appears shifted down, the display driver IC
    // may require a non-zero start row.  Consult your module's datasheet and
    // adjust the init parameters accordingly.
    tft.init(240, 280);
    tft.setRotation(0);      // 0 = portrait, connector at bottom — adjust if needed
    tft.fillScreen(C_BG);

    // --- Static skeleton (drawn once, never overwritten) ---
    hband(Y_HEADER, H_HEADER, C_HEADER);
    tft.setTextSize(1);
    tft.setTextColor(C_WHITE, C_HEADER);
    // Centre "WIPER MOTOR CTRL" (16 chars × 6 px = 96 px) in 240-px width.
    tft.setCursor((SCR_W - 16 * 6) / 2, Y_HEADER + (H_HEADER - 8) / 2);
    tft.print("WIPER MOTOR CTRL");

    hband(Y_DIV0, H_DIV, C_DIV);
    hband(Y_DIV1, H_DIV, C_DIV);
    hband(Y_DIV2, H_DIV, C_DIV);
    hband(Y_DIV3, H_DIV, C_DIV);

    _ready = true;
}

void Display::update(const DisplayState& s) {
    if (!_ready) return;
    drawMotorSection(0, Y_M0, s);
    drawMotorSection(1, Y_M1, s);
    drawServoSection(0, Y_S0, s);
    drawServoSection(1, Y_S1, s);
}
