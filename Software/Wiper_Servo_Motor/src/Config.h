#pragma once
// =============================================================================
//  Config.h — RP2040 Dual Brushless Wiper Motor Controller
//
//  Hardware:
//    Motor drivers : BTS7960 ×2
//      Motor 0: IN1=GPIO0 (PWM slice 0A), IN2=GPIO1 (PWM slice 0B)
//      Motor 1: IN1=GPIO2 (PWM slice 1A), IN2=GPIO3 (PWM slice 1B)
//    Encoders      : AS5600 magnetic angle sensor ×2, separate I2C buses
//      Motor 0: I2C0 SDA=GPIO4, SCL=GPIO5
//      Motor 1: I2C1 SDA=GPIO6, SCL=GPIO7
//    Endstops      : 2 per motor, active-low, internal pull-up
//      Motor 0: End-A=GPIO8,  End-B=GPIO9
//      Motor 1: End-A=GPIO10, End-B=GPIO11
//    AnimCom RS485 : UART0, RS485 transceiver
//      DE=GPIO15, TX=GPIO16, RX=GPIO17
//    Local UI      : Rotary encoder + button
//      ENC_CLK=GPIO12, ENC_DT=GPIO13, ENC_SW=GPIO14
//    Local UI      : SPI OLED display (SPI0)
//      SCK=GPIO18, MOSI=GPIO19, DC=GPIO20, CS=GPIO21, RST=GPIO22
//    Status LED    : GPIO25 (LED_BUILTIN)
//    RC Servos     : Servo 0=GPIO26, Servo 1=GPIO27 (PWM slice 5, Servo lib)
// =============================================================================
#include <Arduino.h>

// ---------------------------------------------------------------------------
// PWM parameters — 25 kHz, 5000-step duty range
//
// MotorPWM maps motor index to GPIOs as: IN1 = idx*2, IN2 = idx*2+1
//   Motor 0 → GPIO0 / GPIO1  (PWM slice 0)
//   Motor 1 → GPIO2 / GPIO3  (PWM slice 1)
// analogWriteFreq() / analogWriteRange() are global, so both slices run at
// exactly the same 25 kHz base — no inter-motor frequency drift.
// ---------------------------------------------------------------------------
#define PWM_FREQ_HZ   25000
#define PWM_WRAP      4999    // duty range: 0 … PWM_WRAP (inclusive)

// ---------------------------------------------------------------------------
// I2C0 — Motor 0 AS5600 encoder (Pico as MASTER)
// ---------------------------------------------------------------------------
#define AS5600_0_SDA_PIN    4
#define AS5600_0_SCL_PIN    5
#define AS5600_I2C_FREQ     400000   // 400 kHz fast-mode (applied to both buses)

// ---------------------------------------------------------------------------
// I2C1 — Motor 1 AS5600 encoder (Pico as MASTER)
//   NOTE: Wire1 is reserved as I2C MASTER for this encoder.
//         The former I2C slave interface (GPIO26/27) has been retired;
//         local control is now provided by the OLED + rotary encoder.
// ---------------------------------------------------------------------------
#define AS5600_1_SDA_PIN    6
#define AS5600_1_SCL_PIN    7

// ---------------------------------------------------------------------------
// Endstop switches — active-low, configured INPUT_PULLUP
// ---------------------------------------------------------------------------
#define ENDSTOP_M0_A_PIN    8
#define ENDSTOP_M0_B_PIN    9
#define ENDSTOP_M1_A_PIN   10
#define ENDSTOP_M1_B_PIN   11

// ---------------------------------------------------------------------------
// AnimCom RS485 serial interface (UART0, RS485 transceiver)
//   DE  = GPIO15  Direction enable: driven LOW (receive only, unidirectional)
//   TX  = GPIO16  UART0 TX — not actively used (protocol is receive-only)
//   RX  = GPIO17  UART0 RX — AnimCom frames from Animation Controller
//
//   Baud rate: 115200, 8N1 (AnimCom standard)
//   Use Serial1 (UART0) in the arduino-pico core.
// ---------------------------------------------------------------------------
#define RS485_DE_PIN  15   // RS485 direction enable (driven LOW = receive)
#define RS485_TX_PIN  16   // UART0 TX (declared but unused)
#define RS485_RX_PIN  17   // UART0 RX
#define RS485_BAUD   115200

// Node identity — change to a unique station ID (0x03-0x09) for each unit.
#define NODE_ID      0x03

// ---------------------------------------------------------------------------
// Local UI — rotary encoder with pushbutton
// ---------------------------------------------------------------------------
#define ENC_CLK_PIN   12   // Encoder channel A
#define ENC_DT_PIN    13   // Encoder channel B
#define ENC_SW_PIN    14   // Pushbutton (active-low)

// ---------------------------------------------------------------------------
// Local UI — SPI OLED display (SPI0)
// ---------------------------------------------------------------------------
#define DISP_SCK_PIN    18   // SPI0 SCK
#define DISP_MOSI_PIN   19   // SPI0 TX / MOSI
#define DISP_DC_PIN     20   // Data / Command select
#define DISP_CS_PIN     21   // Chip select (active-low)
#define DISP_RST_PIN    22   // Reset (active-low; tie HIGH if unused)

// ---------------------------------------------------------------------------
// Safety watchdog
// If no valid command is received within this window the motors are coasted.
// ---------------------------------------------------------------------------
#define WATCHDOG_TIMEOUT_MS  5000

// ---------------------------------------------------------------------------
// Motor count
// ---------------------------------------------------------------------------
#define NUM_MOTORS  2

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------
#define STATUS_LED_PIN  LED_BUILTIN

// ---------------------------------------------------------------------------
// RC Servo outputs — standard 50 Hz PWM (1–2 ms pulse width, 0–180°)
//   Servo 0: GPIO26  (PWM slice 5A)
//   Servo 1: GPIO27  (PWM slice 5B)
//
//   IMPORTANT: Use the arduino-pico Servo library, NOT analogWrite(), to
//   avoid reconfiguring the global motor PWM frequency (analogWriteFreq()
//   affects all slices; Servo library configures hardware registers directly).
// ---------------------------------------------------------------------------
#define SERVO_0_PIN  26
#define SERVO_1_PIN  27
#define NUM_SERVOS   2

// ---------------------------------------------------------------------------
// Legacy — I2C slave (GPIO26/27) retired.  Symbols kept only to avoid
// breaking I2CSlave.cpp compilation.  I2CSlave::begin() must NOT be called.
// ---------------------------------------------------------------------------
#define I2C_SDA_PIN  26   // deprecated — do not use
#define I2C_SCL_PIN  27   // deprecated — do not use

