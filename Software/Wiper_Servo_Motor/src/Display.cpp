#include "Display.h"
#include <Adafruit_ST7735.h>
#include <Adafruit_GFX.h>
#include <SPI.h>

// ---------------------------------------------------------------------------
// Static member definitions
// ---------------------------------------------------------------------------
bool Display::_ready = false;

// ---------------------------------------------------------------------------
// Colours — RGB565
// ---------------------------------------------------------------------------
static constexpr uint16_t C_BG       = 0x0000;  // black background
static constexpr uint16_t C_WHITE    = 0xFFFF;  // primary text
static constexpr uint16_t C_LGRAY    = 0xC618;  // detail text
static constexpr uint16_t C_DGRAY    = 0x39E7;  // dividers / unselected items
static constexpr uint16_t C_HEADER   = 0x000D;  // dark navy — header bar
static constexpr uint16_t C_MANUAL   = 0xFFE0;  // yellow  — MANUAL mode bar
static constexpr uint16_t C_VELOCITY = 0x07E0;  // green   — VELOCITY mode bar
static constexpr uint16_t C_POSITION = 0x07FF;  // cyan    — POSITION mode bar
static constexpr uint16_t C_SERVO    = 0xF81F;  // magenta — servo bar
static constexpr uint16_t C_DISABLED = 0xF800;  // red     — DISABLED state
static constexpr uint16_t C_ANIMCOM  = 0x07FF;  // cyan    — ANIMCOM state

// ---------------------------------------------------------------------------
// Layout constants — 128 × 160 portrait
// ---------------------------------------------------------------------------
static constexpr uint16_t SCR_W = 128;
static constexpr uint16_t SCR_H = 160;

// --- Status screen ---
static constexpr uint16_t Y_HDR         =   0;  static constexpr uint16_t H_HDR       = 16;
static constexpr uint16_t Y_M0          =  16;  static constexpr uint16_t H_MOTOR     = 44;
static constexpr uint16_t Y_DIV0        =  60;  static constexpr uint16_t H_DIV       =  2;
static constexpr uint16_t Y_M1          =  62;
static constexpr uint16_t Y_DIV1        = 106;
static constexpr uint16_t Y_S0_BAR      = 108;  static constexpr uint16_t H_SRV_BAR   = 12;
static constexpr uint16_t Y_S0_VAL      = 120;
static constexpr uint16_t Y_S1_BAR      = 128;
static constexpr uint16_t Y_S1_VAL      = 140;
static constexpr uint16_t Y_STATUS_BAR  = 148;  static constexpr uint16_t H_STATUS_BAR = 12;
// y = 160 — end

// Motor-section offsets (relative to section ybase):
static constexpr uint8_t MO_BAR_H   = 12;   // mode-colour bar
static constexpr uint8_t MO_VAL_Y   = 12;   // scale-2 primary value
static constexpr uint8_t MO_LINE1_Y = 28;   // scale-1 detail line 1
static constexpr uint8_t MO_LINE2_Y = 36;   // scale-1 detail line 2
// H_MOTOR = 44 total (no wasted padding)

// --- Menu screen ---
static constexpr uint16_t MENU_HDR_H  = 16;
static constexpr uint16_t MENU_ITEM_H = 40;   // 3 items × 40 = 120 px
static constexpr uint16_t MENU_ITEM_Y = 16;   // first item starts here
static constexpr uint16_t MENU_HINT_Y = 136;  // 16 + 3×40 = 136
static constexpr uint16_t MENU_HINT_H = 24;   // to y=160

// ---------------------------------------------------------------------------
// TFT instance — hardware SPI0 (SCK=GPIO18, MOSI=GPIO19)
// Canvas — full-screen RAM framebuffer; all drawing targets this, then the
// completed frame is blasted to the TFT in one SPI burst (true double buffer).
// 128×160×2 = 40,960 bytes — fits comfortably in RP2040's 264 KB SRAM.
// ---------------------------------------------------------------------------
static Adafruit_ST7735  tft(&SPI, DISP_CS_PIN, DISP_DC_PIN, DISP_RST_PIN);
static GFXcanvas16      canvas(SCR_W, SCR_H);

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

static inline void hband(uint16_t y, uint16_t h, uint16_t color) {
    canvas.fillRect(0, y, SCR_W, h, color);
}

// Print text at (x,y) with opaque background so old content is overwritten.
// Scale-1 chars: 6×8 px — pad to 21 chars to cover the 126 px usable width.
// Scale-2 chars: 12×16 px — pad to 10 chars.
void Display::_txtAt(int16_t x, int16_t y, uint8_t sz, uint16_t fg, const char* str) {
    char padded[24];
    snprintf(padded, sizeof(padded), (sz == 1) ? "%-21s" : "%-10s", str);
    canvas.setTextSize(sz);
    canvas.setTextColor(fg, C_BG);
    canvas.setCursor(x, y);
    canvas.print(padded);
}

// Fill a full-width band and print a small label inside it.
void Display::_colorBar(uint16_t y, uint16_t h, uint16_t bg, const char* label) {
    hband(y, h, bg);
    canvas.setTextSize(1);
    canvas.setTextColor(C_BG, bg);
    canvas.setCursor(3, y + (h - 8) / 2);
    canvas.print(label);
}

// ---------------------------------------------------------------------------
// Motor section — 44 px starting at ybase
// ---------------------------------------------------------------------------
void Display::_drawMotorSection(uint8_t m, uint16_t yb, const DisplayState& ds) {
    char buf[22];

    // Mode bar
    uint16_t    barCol;
    const char* barLabel;
    switch (ds.mode[m]) {
        case 2:  barCol = C_POSITION; barLabel = m ? "M2 POSITION" : "M1 POSITION"; break;
        case 1:  barCol = C_VELOCITY; barLabel = m ? "M2 VELOCITY" : "M1 VELOCITY"; break;
        default: barCol = C_MANUAL;   barLabel = m ? "M2 MANUAL  " : "M1 MANUAL  "; break;
    }
    _colorBar(yb + 0, MO_BAR_H, barCol, barLabel);

    // Primary value (scale 2)
    switch (ds.mode[m]) {
        case 2:  snprintf(buf, sizeof(buf), "%.1fdeg", ds.measPosAbs[m]);  break;
        case 1:  snprintf(buf, sizeof(buf), "%.1fd/s", ds.measVel[m]);     break;
        default: snprintf(buf, sizeof(buf), "Dty%+d",  (int)ds.duty[m]);   break;
    }
    _txtAt(2, yb + MO_VAL_Y, 2, C_WHITE, buf);

    // Detail lines (scale 1)
    switch (ds.mode[m]) {
        case 2:  // POSITION
            snprintf(buf, sizeof(buf), "Tgt:%.1f E:%+.1f", ds.targetPos[m], ds.posError[m]);
            _txtAt(2, yb + MO_LINE1_Y, 1, C_LGRAY, buf);
            snprintf(buf, sizeof(buf), "V:%.1fd/s", ds.measVel[m]);
            _txtAt(2, yb + MO_LINE2_Y, 1, C_LGRAY, buf);
            break;
        case 1:  // VELOCITY
            snprintf(buf, sizeof(buf), "Cmd:%.1fd/s", ds.commandedVel[m]);
            _txtAt(2, yb + MO_LINE1_Y, 1, C_LGRAY, buf);
            snprintf(buf, sizeof(buf), "Abs:%.1fdeg", ds.measPosAbs[m]);
            _txtAt(2, yb + MO_LINE2_Y, 1, C_LGRAY, buf);
            break;
        default:  // MANUAL
            snprintf(buf, sizeof(buf), "Abs:%.1fdeg", ds.measPosAbs[m]);
            _txtAt(2, yb + MO_LINE1_Y, 1, C_LGRAY, buf);
            _txtAt(2, yb + MO_LINE2_Y, 1, C_LGRAY, "");
            break;
    }
}

// ---------------------------------------------------------------------------
// Servo row — bar (H_SRV_BAR px) + value line (8 px) starting at ybar
// ---------------------------------------------------------------------------
void Display::_drawServoRow(uint8_t idx, uint16_t ybar, const DisplayState& ds) {
    char buf[22];
    _colorBar(ybar, H_SRV_BAR, C_SERVO, idx ? "S2 SERVO" : "S1 SERVO");
    if (ds.servoActual[idx] != ds.servoTarget[idx]) {
        snprintf(buf, sizeof(buf), "%d->%ddeg", ds.servoActual[idx], ds.servoTarget[idx]);
    } else {
        snprintf(buf, sizeof(buf), "%ddeg", ds.servoActual[idx]);
    }
    _txtAt(2, ybar + H_SRV_BAR, 1, C_WHITE, buf);
}

// ---------------------------------------------------------------------------
// Bottom status bar — mode-coloured 12 px band with contextual info
// ---------------------------------------------------------------------------
void Display::_drawStatusBar(AppState state, const DisplayState& ds) {
    char buf[22];
    uint16_t barCol;

    switch (state) {
        case STATE_DISABLED:
            barCol = C_DISABLED;
            snprintf(buf, sizeof(buf), "DISABLED  BTN:MENU");
            break;
        case STATE_MANUAL:
            barCol = C_VELOCITY;
            snprintf(buf, sizeof(buf), "LOCAL CTRL  BTN:MENU");
            break;
        case STATE_ANIMCOM:
            barCol = C_ANIMCOM;
            switch (ds.animState) {
                case 1:  snprintf(buf, sizeof(buf), "RS485:MANUAL  BTN:MENU"); break;
                case 2:  snprintf(buf, sizeof(buf), "RS485:P%d@%d%%  BTN:MENU",
                                  ds.animPattern, ds.animSpeedScale); break;
                default: snprintf(buf, sizeof(buf), "RS485:IDLE  BTN:MENU");  break;
            }
            break;
        default:
            barCol = C_DGRAY;
            snprintf(buf, sizeof(buf), "BTN:MENU");
            break;
    }
    _colorBar(Y_STATUS_BAR, H_STATUS_BAR, barCol, buf);
}

// ---------------------------------------------------------------------------
// Public: begin
// ---------------------------------------------------------------------------
void Display::begin() {
    // On arduino-pico (Earle Philhower), pin assignments must be set before
    // SPI.begin().  The Adafruit library calls begin() internally inside
    // initR(), so set pins and call begin() explicitly first.
    SPI.setSCK(DISP_SCK_PIN);
    SPI.setTX(DISP_MOSI_PIN);
    SPI.begin();

    // ST7735S 128×160 — INITR_BLACKTAB is the standard init sequence for
    // the common 1.8" red-PCB module.  Use INITR_GREENTAB for green-tab
    // variants (adds a 2-pixel row/col offset correction automatically).
    tft.initR(INITR_BLACKTAB);
    tft.setSPISpeed(DISP_SPI_FREQ);
    tft.setRotation(0);       // portrait, connector at bottom
    tft.fillScreen(C_BG);

    _ready = true;
}

// ---------------------------------------------------------------------------
// Public: drawMenu
// ---------------------------------------------------------------------------
void Display::drawMenu(uint8_t selectedIdx) {
    if (!_ready) return;

    // Header bar
    hband(0, MENU_HDR_H, C_HEADER);
    canvas.setTextSize(1);
    canvas.setTextColor(C_WHITE, C_HEADER);
    canvas.setCursor(4, (MENU_HDR_H - 8) / 2);
    canvas.print("WIPER MOTOR CTRL");

    // Item definitions: per-item colour and label
    struct Item { uint16_t color; const char* label; };
    static const Item kItems[NUM_APP_STATES] = {
        { C_DISABLED, "DISABLED" },
        { C_VELOCITY, "MANUAL"   },
        { C_ANIMCOM,  "ANIMCOM"  },
    };

    for (uint8_t i = 0; i < NUM_APP_STATES; i++) {
        uint16_t iy  = MENU_ITEM_Y + i * MENU_ITEM_H;
        bool     sel = (i == selectedIdx);
        uint16_t bg  = sel ? kItems[i].color : C_DGRAY;

        hband(iy, MENU_ITEM_H, bg);

        // Selection arrow and label in scale-2 text, vertically centred.
        uint16_t ty = iy + (MENU_ITEM_H - 16) / 2;
        uint16_t fg = sel ? C_BG : C_WHITE;
        canvas.setTextSize(2);
        canvas.setTextColor(fg, bg);

        // Arrow at x=4
        canvas.setCursor(4, ty);
        canvas.print(sel ? ">" : " ");

        // Label at x=22
        canvas.setCursor(22, ty);
        canvas.print(kItems[i].label);
    }

    // Hint bar at bottom
    hband(MENU_HINT_Y, MENU_HINT_H, C_BG);
    canvas.setTextSize(1);
    canvas.setTextColor(C_LGRAY, C_BG);
    canvas.setCursor(4, MENU_HINT_Y + (MENU_HINT_H - 8) / 2);
    canvas.print("Enc:scroll  Btn:enter");

    // Blit completed frame to display in one SPI burst — no partial updates visible.
    tft.drawRGBBitmap(0, 0, canvas.getBuffer(), SCR_W, SCR_H);
}

// ---------------------------------------------------------------------------
// Public: update  (status screen)
// ---------------------------------------------------------------------------
void Display::update(AppState state, const DisplayState& ds) {
    if (!_ready) return;

    // Header bar — mode name + hint
    hband(Y_HDR, H_HDR, C_HEADER);
    canvas.setTextSize(1);
    canvas.setTextColor(C_WHITE, C_HEADER);
    canvas.setCursor(3, Y_HDR + (H_HDR - 8) / 2);
    switch (state) {
        case STATE_DISABLED: canvas.print("DISABLED      <MENU"); break;
        case STATE_MANUAL:   canvas.print("MANUAL        <MENU"); break;
        case STATE_ANIMCOM:  canvas.print("ANIMCOM       <MENU"); break;
        default:             canvas.print("?             <MENU"); break;
    }

    // Motor sections
    _drawMotorSection(0, Y_M0, ds);
    hband(Y_DIV0, H_DIV, C_DGRAY);
    _drawMotorSection(1, Y_M1, ds);
    hband(Y_DIV1, H_DIV, C_DGRAY);

    // Servo rows
    _drawServoRow(0, Y_S0_BAR, ds);
    _drawServoRow(1, Y_S1_BAR, ds);

    // Bottom status bar
    _drawStatusBar(state, ds);

    // Blit completed frame to display in one SPI burst — no partial updates visible.
    tft.drawRGBBitmap(0, 0, canvas.getBuffer(), SCR_W, SCR_H);
}
