#pragma once
// =============================================================================
//  LocalUI.h — Local user interface: SPI OLED display + rotary encoder
//
//  Hardware:
//    Display  : SPI monochrome OLED (SSD1306 / SH1106 or compatible)
//               SCK=GPIO18, MOSI=GPIO19, DC=GPIO20, CS=GPIO21, RST=GPIO22
//    Encoder  : Rotary encoder with pushbutton
//               CLK=GPIO12, DT=GPIO13, SW=GPIO14 (all active-low, pull-up)
//
//  Encoder reading uses interrupt-driven gray-code decoding so no counts
//  are lost even during fast turns.  Button is debounced via edge interrupt.
//
//  Display library integration (U8g2 / Adafruit SSD1306 / etc.) and
//  mode-change UI logic are implemented separately on top of this driver.
//
//  Usage:
//    LocalUI::begin();            // once in setup()
//    // each loop or task:
//    int16_t delta = LocalUI::encoderDelta();  // steps since last call
//    if (LocalUI::buttonPressed()) { ... }
// =============================================================================

#include <Arduino.h>
#include "Config.h"

class LocalUI {
public:
    // Configure all pins and attach encoder/button interrupts.
    // Call once in setup() after Serial is initialised.
    static void begin();

    // Returns the encoder step count accumulated since the last call,
    // then resets the internal counter to zero.
    // Positive = clockwise, negative = counter-clockwise.
    static int16_t encoderDelta();

    // Returns true if the button was pressed since the last call,
    // then clears the flag.
    static bool buttonPressed();

private:
    static volatile int16_t _encCount;
    static volatile bool    _btnPressed;
    static volatile uint8_t _lastEncState;

    // Interrupt service routines (static so compatible with attachInterrupt)
    static void _encISR();
    static void _btnISR();
};
