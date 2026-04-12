# Quad Motor Controller — Wiper Servo Motor

**Authoritative design and implementation reference.**
Read this document before starting any new implementation session.

---

## Table of Contents

1. [Overview](#overview)
2. [Hardware](#hardware)
3. [Pin Assignments](#pin-assignments)
4. [Application States](#application-states)
5. [Display UI Design](#display-ui-design)
6. [Encoder Navigation](#encoder-navigation)
7. [AnimCom Protocol](#animcom-protocol)
8. [Station-Specific Animation Patterns](#station-specific-animation-patterns)
9. [NVM Settings](#nvm-settings)
10. [Console Commands](#console-commands)
11. [Build and Flash](#build-and-flash)
12. [Implementation Status and Known Gaps](#implementation-status-and-known-gaps)

---

## Overview

This firmware runs on a Raspberry Pi Pico / Pico W (RP2040) or Pico 2 / Pico 2 W (RP2350) and implements a quad motor controller driving:

- **2 brushed DC motors** via BTS7960 H-bridge drivers with AS5600 magnetic encoders for closed-loop velocity and position control
- **2 RC servo motors** with software ramp control

The controller is a slave node on an RS485 AnimCom bus, receiving motion commands from an upstream animation controller. It also supports local manual control via a rotary encoder and 128x160 ST7735S colour TFT display, plus a USB serial console for development and calibration.

**Framework:** PlatformIO / Arduino, Earle Philhower `arduino-pico` core
**Core assignment:**
- Core 0 — 50 Hz control loop, console input, encoder/button UI state machine
- Core 1 — Display refresh (5 Hz), USB serial status print (2 Hz)

---

## Hardware

| Component | Part | Notes |
|-----------|------|-------|
| MCU | RP2040 / RP2350 | Pico, Pico W, Pico 2, or Pico 2 W |
| DC motor drivers | BTS7960 x2 | H-bridge, 25 kHz complementary PWM |
| DC motor encoders | AS5600 x2 | Magnetic angle sensor, 12-bit, I2C |
| Servo outputs | Standard RC servos x2 | 50 Hz PWM, 1-2 ms pulse |
| Endstop switches | Limit switches x4 | Active-low, INPUT_PULLUP |
| RS485 transceiver | Any standard RS485 IC | Half-duplex, receive-only (DE driven LOW) |
| Display | ST7735S 128x160 TFT | 1.8" red-PCB module, SPI |
| Rotary encoder | EC11 or equivalent | With pushbutton |
| Status LED | GPIO25 | LED_BUILTIN (onboard LED) |

### Motor Control

Each DC motor operates in one of three software control modes selected per motor via the `uiType` NVM setting:

- **MANUAL (PWM%)** — Raw PWM duty applied directly. Encoder steps at 1% increments in MANUAL UI mode.
- **VELOCITY** — Velocity PID loop regulates angular velocity (deg/s). Encoder steps at 5 deg/s increments.
- **POSITION** — Cascade: position PID generates velocity setpoint, velocity PID generates PWM duty. Encoder steps at 5 deg increments.

Position tracking is multi-turn absolute (accumulated from encoder deltas), so targets like 940 deg or -180 deg work directly without wraparound.

### Servo Control

Both servos use the `arduino-pico` Servo library (not `analogWrite`, which would reconfigure the global motor PWM frequency). A software ramp interpolates from the current output angle to the commanded target at a configurable rate (default 90 deg/s).

### Dual-Core Architecture

```
Core 0 (loop):
  - 50 Hz control loop — reads encoders, runs PID, drives PWM and servos
  - AnimCom RS485 poll (g_animcom.poll())
  - USB serial console input (non-blocking)
  - Encoder / button UI state machine

Core 1 (loop1):
  - 5 Hz display refresh (Display::drawMenu / Display::update / Display::drawConfig)
  - 2 Hz USB serial status print
  - All Serial.print() is here — cannot stall Core 0 control loop
```

Cross-core state is shared via `volatile` 32-bit-aligned variables. Reads/writes are atomic on Cortex-M0+. Display SPI is used exclusively from Core 1 after `Display::begin()`.

---

## Pin Assignments

### DC Motors — BTS7960 H-Bridge

| Signal | GPIO | PWM Slice |
|--------|------|-----------|
| Motor 0 IN1 (forward) | GPIO1 | Slice 0A |
| Motor 0 IN2 (reverse) | GPIO0 | Slice 0B |
| Motor 1 IN1 (forward) | GPIO3 | Slice 1A |
| Motor 1 IN2 (reverse) | GPIO2 | Slice 1B |

Both IN1/IN2 pins of each motor are on the same PWM slice so they share a common counter (complementary PWM). Global PWM frequency: **25 kHz**, wrap: **4999** (5000-step duty range, ±4999 signed).

### AS5600 Magnetic Encoders

| Signal | Motor 0 | Motor 1 |
|--------|---------|---------|
| I2C Bus | I2C0 / Wire | I2C1 / Wire1 |
| SDA | GPIO4 | GPIO6 |
| SCL | GPIO5 | GPIO7 |
| Frequency | 400 kHz | 400 kHz |

Both sensors share I2C address 0x36 but are isolated on separate buses.

### Endstop Switches

| Signal | GPIO |
|--------|------|
| Motor 0 End-A | GPIO8 |
| Motor 0 End-B | GPIO9 |
| Motor 1 End-A | GPIO10 |
| Motor 1 End-B | GPIO11 |

All endstops: active-low, INPUT_PULLUP.

### RC Servos

| Signal | GPIO | PWM Slice |
|--------|------|-----------|
| Servo 0 | GPIO26 | Slice 5A |
| Servo 1 | GPIO27 | Slice 5B |

Standard 50 Hz PWM, 0-180 deg range. Use `Servo` library only — `analogWrite` on these pins would corrupt the motor PWM configuration.

### RS485 AnimCom (UART0)

| Signal | GPIO | Notes |
|--------|------|-------|
| DE (Direction Enable) | GPIO15 | Driven LOW (receive-only) |
| TX | GPIO16 | Declared, not actively used |
| RX | GPIO17 | AnimCom frames from animation controller |

Baud: **115200, 8N1**

### Local UI — Rotary Encoder

| Signal | GPIO |
|--------|------|
| CLK (Channel A) | GPIO12 |
| DT (Channel B) | GPIO13 |
| SW (Button) | GPIO14 |

All active-low, INPUT_PULLUP. Interrupt-driven.

### Local UI — ST7735S Display (SPI0)

| Signal | GPIO |
|--------|------|
| SCK | GPIO18 |
| MOSI | GPIO19 |
| DC | GPIO20 |
| CS | GPIO21 |
| RST | GPIO22 |

SPI clock: **8 MHz** (reduce to 1-4 MHz if display is unresponsive on long cables).
Display: **128x160 pixels**, portrait orientation, connector at bottom.
Init sequence: `INITR_BLACKTAB` (standard 1.8" red-PCB module). Use `INITR_GREENTAB` for green-tab variants which add a 2-pixel row/column offset correction automatically.

### Status LED

| Signal | GPIO |
|--------|------|
| Status LED | GPIO25 (LED_BUILTIN) |

---

## Application States

The system has four top-level application states, selected from the top-level menu.

```
+-------------------+
|    TOP MENU       |  <-- entered by pressing encoder button from any state
|    DISABLED       |
|    MANUAL         |
|    ANIMCOM        |  <-- DEFAULT STARTUP STATE (target)
|    CONFIG         |
+-------------------+
```

**Default startup state: ANIMCOM** (currently coded as `STATE_DISABLED` with `g_inMenu = true` — this must be corrected per Implementation Status). In ANIMCOM state with no RS485 commands received, the system runs the built-in default animation pattern on loop, providing a minimal show in case of animation controller or RS485 cabling failure.

### STATE_DISABLED

Motors coasting, read-only status display. No motor commands are accepted from any source while in this state. On entry: all motors are coasted, servos commanded to 90 deg (center).

### STATE_MANUAL

Local encoder control of motors and servos. The encoder scrolls the selection cursor between the four motor/servo channels and an EXIT item. Pressing the button on a channel enters ADJUST mode for that channel; the encoder then adjusts that channel's command in real time. Pressing the button again exits ADJUST mode. Pressing EXIT returns to the top menu and coasts all motors.

Motor adjustment step sizes when in ADJUST mode:
- **PWM% mode:** 1% steps (mapped from ±PWM_WRAP to ±100%)
- **VELOCITY mode:** 5 deg/s steps, clamped to ±velLimit
- **POSITION mode:** 5 deg steps
- **Servo channels:** 1 deg steps, clamped 0-180 deg

### STATE_ANIMCOM (Default Startup)

RS485 AnimCom slave mode. The controller listens for frames addressed to its configured station ID. Motor and servo commands arrive via `MANUAL_SINGLE` frames. Autonomous animation patterns run via `RUN_AUTO` frames.

**ManualHold detection:** If the RS485 state is `ANIMCOM_STATE_MANUAL` but no `MANUAL_SINGLE` frame has been received for a given motor channel within 400 ms, that motor is coasted. This distinguishes a deliberate hold (keepalive CONTROL_STATE frames only, no MANUAL_SINGLE frames) from active motion commands.

**Watchdog:** If no valid AnimCom frame is received within 5000 ms, all motors are coasted and servos centered. The internal state returns to `ANIMCOM_STATE_STOP`.

**Default animation fallback:** When `g_animState == ANIMCOM_STATE_RUN_AUTO`, `applyAnimRunPattern()` runs a sine-wave-driven pattern across all four outputs. `showIntensity` multiplies into the sine wave time argument (controls cycle rate, not amplitude). Pattern numbers:
- **0, 1 (default):** Both motors VELOCITY mode, sinusoidal velocity, servos sweep in phase
- **2:** Both motors POSITION mode, large arcs (90+/-70 deg and 180+/-120 deg), servos sweep
- **3:** Motor 0 VELOCITY bang-bang, Motor 1 POSITION, servos sweep at different rates

### STATE_CONFIG

Configuration screen. The encoder scrolls between editable items; pressing the button enters edit mode for the selected item (encoder adjusts value). Pressing the button again confirms the value. Pressing SAVE writes all settings to flash and returns to the menu. Exiting CONFIG without pressing SAVE discards unsaved RAM changes.

---

## Display UI Design

The display is a 128x160 ST7735S driven via a full-screen `GFXcanvas16` RAM framebuffer. All drawing targets the canvas; the completed frame is blasted to the TFT in one SPI burst (true double-buffering — no partial-update flicker). Display refresh runs at **5 Hz on Core 1**.

The UI uses a **cursor metaphor**: the currently selected item is highlighted with an inverted or colour-filled block. Turning the encoder moves the cursor between selectable items. Pressing the button activates the selected item.

### Colour Palette (RGB565)

| Constant | Colour | Used For |
|----------|--------|----------|
| C_BG | Black (0x0000) | Screen background |
| C_WHITE | White (0xFFFF) | Primary text |
| C_LGRAY | Light grey (0xC618) | Detail/secondary text |
| C_DGRAY | Dark grey (0x39E7) | Dividers, unselected menu items |
| C_HEADER | Dark navy (0x000D) | Header bars |
| C_MANUAL | Yellow (0xFFE0) | MANUAL mode bar, CONFIG item |
| C_VELOCITY | Green (0x07E0) | VELOCITY mode bar, edit highlight |
| C_POSITION | Cyan (0x07FF) | POSITION mode bar |
| C_SERVO | Magenta (0xF81F) | Servo bar |
| C_DISABLED | Red (0xF800) | DISABLED state bar, status bar |
| C_ANIMCOM | Cyan (0x07FF) | ANIMCOM state bar |

---

### Top-Level Menu Screen

Shown when `g_inMenu == true`. Entered by pressing the encoder button from any state, or at startup (current startup; target is to start directly in ANIMCOM).

```
y=  0 -  15   [WIPER MOTOR CTRL]        dark navy header, white text
y= 16 -  51   [> DISABLED      ]        36 px — red when selected, dark grey when not
y= 52 -  87   [  MANUAL        ]        36 px — green when selected, dark grey when not
y= 88 - 123   [  ANIMCOM       ]        36 px — cyan when selected, dark grey when not
y=124 - 159   [  CONFIG        ]        36 px — yellow when selected, dark grey when not
```

4 items x 36 px + 16 px header = 160 px exactly.

Selected item: filled with its state colour, black text, `>` arrow at x=4.
Unselected items: dark grey background, white text, space at x=4.
Label in scale-2 text (12x16 px), vertically centred within each 36 px band.

**Encoder:** scrolls cursor through the 4 items, wraps at top and bottom.
**Button:** enters the highlighted state, exits the menu.

---

### Status Screen Layout

Shared pixel layout for DISABLED, MANUAL, and ANIMCOM states (when `g_inMenu == false` and `g_appState != STATE_CONFIG`).

```
y=  0 -  15   Header bar: "[MODENAME]       <MENU"        dark navy, white text
y= 16 -  59   Motor 0 section  (44 px total)
               y+0  - y+11    Mode-colour bar "M1 [MODE]"   12 px
               y+12 - y+27    Primary value  scale-2          16 px
               y+28 - y+35    Detail line 1  scale-1           8 px
               y+36 - y+43    Detail line 2  scale-1           8 px
y= 60 -  61   Divider  (dark grey, 2 px)
y= 62 - 105   Motor 1 section  (44 px, same layout as Motor 0)
y=106 - 107   Divider  (dark grey, 2 px)
y=108 - 119   Servo 0 bar  "S1 SERVO"  magenta, 12 px
y=120 - 127   Servo 0 value line       scale-1, 8 px
y=128 - 139   Servo 1 bar  "S2 SERVO"  magenta, 12 px
y=140 - 147   Servo 1 value line       scale-1, 8 px
y=148 - 159   Status / hint bar        12 px, mode-coloured
```

#### Motor Section Content by Mode

| Motor Mode | Bar Colour | Bar Label | Primary Value (scale-2) | Detail Line 1 | Detail Line 2 |
|-----------|-----------|----------|------------------------|---------------|---------------|
| MANUAL | Yellow | M1/M2 MANUAL | `Dty+NNNN` (raw duty) | `Abs: NNN.N deg` | (empty) |
| VELOCITY | Green | M1/M2 VELOCITY | `NNN.N d/s` (measured vel) | `Cmd: NNN.N d/s` (ramped cmd) | `Abs: NNN.N deg` |
| POSITION | Cyan | M1/M2 POSITION | `NNN.N deg` (measured abs pos) | `Tgt: NNN.N E: +/-NNN.N` | `V: NNN.N d/s` |

**ENCODER OFFLINE alert:** In ANIMCOM and MANUAL states, if a motor's `uiType` config requires an encoder (VELOCITY or POSITION) and the AS5600 is not responding, the primary value area displays `OFFLINE` (red, scale-2) and the first detail line shows `encoder offline`. The second detail line is cleared. This is driven by `g_encoderOffline[m]` set in the control loop when `AS5600::readAngle()` returns `0xFFFF`.

#### Servo Row Content

- Servo at target: `NNN deg`
- Servo ramping: `NNN->NNN deg`

#### Status Bar (y=148-159, 12 px)

| State | Bar Colour | Content |
|-------|-----------|---------|
| DISABLED | Red | `DISABLED  BTN:MENU` |
| MANUAL | Green | `LOCAL CTRL  BTN:MENU` |
| ANIMCOM — STOP | Cyan | `RS485:IDLE  BTN:MENU` |
| ANIMCOM — MANUAL | Cyan | `RS485:MANUAL  BTN:MENU` |
| ANIMCOM — RUN_AUTO | Cyan | `RS485:P[n]@[s]%  BTN:MENU` |

---

### DISABLED State Screen (design intent)

The DISABLED screen uses the standard status screen layout with the following intent:

- Header: `"DISABLED       <MENU"` (dark navy)
- All four motor/servo sections show STOPPED status
- If a motor's position is known (encoder was responding before entering DISABLED), show the last known absolute position in that motor's section
- The cursor rests permanently on an EXIT button at the bottom — no motor items are selectable
- Pressing the button returns to the top menu

---

### ANIMCOM State Screen (design intent)

The ANIMCOM screen uses the standard status screen layout with these additions:

- Header: `"ANIMCOM        <MENU"`
- All four sections show live values as they arrive from RS485 frames
- If a motor requires an encoder (uiType = VELOCITY or POSITION) and the encoder is OFFLINE, show `ENCODER OFFLINE` alert in that motor's primary value area
- Bottom status bar shows current RS485 mode, pattern number, and speed percent
- Button returns to top menu; on exit, all motors are coasted

**Extended COMM status section (design intent, not yet implemented as a separate band):** A dedicated section below the motor rows was intended to show RS485 bus live/timeout status separately from the bottom bar. Currently the bottom status bar carries all RS485 mode information.

---

### MANUAL State Screen (design intent)

The MANUAL screen uses the standard status screen layout with an active cursor:

- Header: `"MANUAL         <MENU"`
- Cursor scrolls between 5 items: Motor 1, Motor 2, Servo 1, Servo 2, EXIT
- The currently selected item is highlighted (inverted/colour-filled)
- **Button press on a motor or servo:** Enters ADJUST mode for that channel
  - Encoder immediately adjusts that channel's command
  - DC motors: command type follows `uiType` config (PWM% / VELOCITY deg/s / POSITION deg)
  - Servos: always adjust angle (0-180 deg)
  - Second button press exits ADJUST mode, returns to cursor scroll
- **Button press on EXIT:** Coasts all motors, returns to top menu

**Note on current implementation:** The current code cycles through M0 -> M1 -> EXIT with successive button presses rather than the full cursor-scroll model. Servo channels are not yet selectable in MANUAL mode. The full cursor-scroll model described above is the design intent for future implementation.

---

### CONFIG State Screen

```
y=  0 -  15   Header: "CONFIG  Btn:edit/save"     dark navy, white text
y= 16 -  33   Item 0: Station ID      value: 0xNN               18 px
y= 34 -  51   Item 1: M0 Type         value: PWM% / VEL / POS   18 px
y= 52 -  69   Item 2: M0 Vel Limit    value: NNNN deg/s          18 px
y= 70 -  87   Item 3: M0 Traverse Vel value: NNNN deg/s          18 px
y= 88 - 105   Item 4: M1 Type         value: PWM% / VEL / POS   18 px
y=106 - 123   Item 5: M1 Vel Limit    value: NNNN deg/s          18 px
y=124 - 141   Item 6: M1 Traverse Vel value: NNNN deg/s          18 px
y=142 - 159   Item 7: >>> SAVE                                    18 px
```

16 + 8 x 18 = 160 px exactly.

**Background colours:**
- Black: unselected item
- Dark grey: selected item, not editing
- Bright green: selected item, actively editing

**Navigation behaviour:**
- Encoder scrolls cursor between 8 items, wraps at top and bottom
- Button on a data item: enters edit mode (encoder adjusts the value, item turns green)
- Button in edit mode: confirms value, returns to scroll mode (item turns grey)
- Button on SAVE (item 7): saves all settings to flash, coasts motors, returns to top menu
- Return to menu without pressing SAVE: unsaved RAM changes are discarded on next `load` or power cycle

**Encoder step sizes when editing:**
- Station ID: +/-1, wraps 0x01-0xFE
- Motor type: +/-1, cycles PWM% -> VEL -> POS (wraps)
- Vel Limit: +/-5 deg/s, clamped 10-720 deg/s
- Traverse Vel: +/-5 deg/s, clamped 10-720 deg/s

**Note:** A dedicated EXIT item (saves nothing, returns to menu) is not yet implemented. The current EXIT from CONFIG is to press SAVE (saves and exits) or power-cycle. A future revision should add item 8 = EXIT as a 9th row, requiring adjustment of the layout arithmetic.

---

## Encoder Navigation

### Quadrature Decoding — Full-Cycle Detector

The encoder ISR uses a full quadrature-cycle detector that only registers a step when the encoder completes a full 4-state sequence back to the detent position (both channel A and channel B pins HIGH).

The last 4 pin states are packed into `_encHistory` as 2-bit pairs (oldest in MSBits):

- **CW pattern:** history byte = `0x87` — increment count
- **CCW pattern:** history byte = `0x4B` — decrement count

This approach naturally suppresses electrical contact bounce because a bouncing contact cannot complete the full 4-state sequence. No explicit debounce timer is needed for the encoder.

`LocalUI::encoderDelta()` atomically returns and clears the accumulated count. Positive values = clockwise, negative = counter-clockwise.

### Button Debounce

The encoder button uses a **50 ms dead-time debounce**. After detecting a falling edge on GPIO14 (active-low), the ISR sets a flag and ignores further edges for 50 ms. `LocalUI::buttonPressed()` returns and clears this one-shot flag.

---

## AnimCom Protocol

RS485 half-duplex, 115200 baud, 8N1. The controller is a **receive-only slave** — the DE pin is driven permanently LOW. The animation controller (master) is the only transmitter on the bus.

### Frame Format

```
Byte 0      sync0        0x55
Byte 1      sync1        0xAA
Byte 2      station_id   0x00=master, 0x01-0xFE=endpoint, 0xFF=broadcast
Byte 3      seq          rolling sequence number (sender increments each frame)
Byte 4      msg_type     see message types below
Byte 5      payload_len  0-32 bytes
Byte 6+     payload      message-specific bytes
Last 2      CRC-16       CRC-16/CCITT over bytes 2 through end of payload
```

Total frame overhead: 8 bytes. Maximum frame size: 40 bytes (8 + 32 payload).

The controller accepts frames addressed to its configured `nodeId` or to broadcast address `0xFF`.

### Message Types

#### 0x01 — CONTROL_STATE (3-byte payload)

```
[0] state        0=STOP, 1=RUN_AUTO, 2=MANUAL
[1] pattern      animation pattern number (used in RUN_AUTO)
[2] show_intensity  0-200 percent (used in RUN_AUTO)
```

Sets the global operational state:
- `STOP` (0): Coast all motors, center servos to 90 deg
- `RUN_AUTO` (1): Run built-in animation pattern; `showIntensity` controls cycle rate (not amplitude)
- `MANUAL` (2): Accept per-channel `MANUAL_SINGLE` frames

#### 0x02 — TRIGGER_EFFECT (4-byte payload)

```
[0] effect_type   0x01=AUDIO_PLAY, 0x02=MOTION_SPEC
[1] effect_id     track number or motion ID
[2..3] effect_param  16-bit parameter (little-endian)
```

Audio trigger placeholder — currently logs to USB serial. DFPlayer/I2S integration is a future task.

#### 0x03 — MANUAL_SINGLE (6-byte payload)

```
[0] channel_index  0=Motor0, 1=Motor1, 2=Servo0, 3=Servo1
[1] cmd_type       0x01=SPEED_PERCENT, 0x02=SPEED_DEG_PER_SEC, 0x03=POSITION_DEG
[2..5] value       int32, little-endian
```

**DC motor channels (0 and 1):**

| cmd_type | Action |
|----------|--------|
| SPEED_PERCENT (0x01) | value low byte = signed percent -100..+100; maps to ~360 deg/s at 100%; sets VELOCITY mode |
| SPEED_DEG_PER_SEC (0x02) | value low 16 bits = signed deg/s target; sets VELOCITY mode |
| POSITION_DEG (0x03) | value low 16 bits = signed absolute target degrees; sets POSITION mode |

**Servo channels (2 and 3):**

| cmd_type | Action |
|----------|--------|
| SPEED_PERCENT (0x01) | value low byte ±100% maps linearly to 0-180 deg, centered at 90 deg |
| POSITION_DEG (0x03) | value low 16 bits = absolute target angle 0-180 deg |

#### 0x04 — MANUAL_BULK (8-byte payload)

Eight signed speed-percent values for octal motor configurations. Received but not acted upon by this quad controller (protocol compatibility only).

### ManualHold Timeout

When `g_animState == ANIMCOM_STATE_MANUAL`, each DC motor independently tracks the timestamp of its last received `MANUAL_SINGLE` frame in `g_lastManualCmdMs[]`. If more than **400 ms** elapses without a `MANUAL_SINGLE` for a channel, that motor is coasted and held in MANUAL mode until commands resume.

This allows the animation controller to selectively park channels by ceasing per-channel updates while continuing to send `CONTROL_STATE` keepalives.

### RUN_AUTO — Speed Scale Behaviour

`showIntensity` (0-200%) multiplies into the time argument of the sine wave generator:

```cpp
const float intensityFactor = (float)g_animShowIntensity / 100.0f;
const float theta = ((float)nowMs * intensityFactor) * (2.0f * PI / 6000.0f);
```

At 100% the base cycle period is 6 seconds. At 200% the cycle period is 3 seconds. At 50% the period is 12 seconds. Amplitude of all position/velocity targets is fixed — only cycle rate scales.

### CRC Algorithm

CRC-16/CCITT: polynomial 0x1021, initial value 0xFFFF, no input/output reflection, no final XOR.
Coverage: bytes 2 through end of payload (station_id through last payload byte).
Transmitted big-endian (high byte first) in the final two bytes of the frame.

---

## Station-Specific Animation Patterns

Each physical installation of the Quad Motor Controller has a unique role in
the animated show, identified by its RS485 station ID.  When the animation
controller sends a `CONTROL_STATE RUN_AUTO` command, the firmware dispatches
to a station-specific pattern function (in `src/StationAnim.cpp`) instead of
the generic sine-wave patterns.

**Design rules:**

- Station-specific patterns take priority over generic patterns (0–3) whenever
  the configured `nodeId` has a registered handler in `stationAnim_hasHandler()`.
- On power-up with no animation controller connected, stations with a registered
  handler automatically enter `ANIMCOM_STATE_RUN_AUTO` at a built-in default
  speed scale so the prop animates immediately without any RS485 traffic.
- After an AnimCom watchdog timeout (5 s with no RS485 frames), stations with a
  registered handler restore their default `RUN_AUTO` state rather than coasting
  to `STOP`, keeping the prop active during a cable or controller outage.
- The `showIntensity` field in `CONTROL_STATE` frames is interpreted as an
  **intensity / position scalar** (0–200%) by station handlers rather than as a
  time multiplier.  Each station defines its own mapping.
- Uncontrolled motors and servos are left in their current state — handlers only
  write the output arrays for channels they explicitly own.

### Station Registry

| Station ID | Name      | Motor 0 Use                   | Motor 1 Use | Servo Use |
|------------|-----------|-------------------------------|-------------|-----------|
| 7          | Meter     | Needle position (0–180 deg)   | Unused      | Unused    |

### Station 7 — Meter

**Hardware:** Motor 0 drives the needle of a large analog gauge through a
positional range of 0–180 degrees.  Motor 0 must be configured for POSITION
mode with travel limits matching the physical gauge:

```
mtype 0 2         ; Motor 0 → POSITION mode
plim 0 0 180      ; Travel limits 0-180 deg
zero 0            ; Set mechanical zero (needle pointing to 0 on the gauge face)
nodeid 7          ; Set RS485 station ID
save              ; Persist to flash
```

**`showIntensity` interpretation:**

| showIntensity (%) | Needle position |
|----------------|-----------------|
| 0              | 0 deg           |
| 50 (default)   | 90 deg          |
| 100            | 180 deg         |
| > 100          | > 180 deg (clamped by `posMax`) |

**Live oscillation:** A sinusoidal wobble of ±5 deg at a 2-second period is
superimposed on the setpoint to simulate a live measurement reading with
visible noise.  The final commanded position is clamped to
`[posMin[0], posMax[0]]` from NVM so the needle never over-travels.

```
Final target = constrain(showIntensity × 1.8 + 5·sin(2π·t / 2 s), posMin, posMax)
```

**Standalone default:** With no animation controller present the needle rests
at 90 deg (50% full scale) with oscillation active.

**Implementation:** `src/StationAnim.cpp` — `updateMeter()`, dispatched via
`stationAnim_update()` from `applyAnimRunPattern()` in `src/main.cpp` at 50 Hz.

### How to Add a New Station

1. Add a `case` to `stationAnim_hasHandler()` in `src/StationAnim.cpp` returning
   `true` for the new station ID.
2. Return the appropriate default `showIntensity` from
   `stationAnim_defaultSpeedScale()`.
3. Write a static handler function (e.g. `updateMyStation()`) that fills in
   `outMode[]`, `outTargetPos[]`, `outTargetVel[]`, and/or `outServoTarget[]`
   only for the channels it controls.  Leave all other array elements unchanged.
4. Dispatch the new handler inside `stationAnim_update()`.
5. Document the station in the registry table above and update this section.

---

## NVM Settings

Non-volatile storage is provided by RP2040 flash-backed EEPROM emulation from the `arduino-pico` core (`EEPROM.h`). The working copy lives in the RAM struct `Settings::s`; flash is written only on explicit save commands.

**Magic:** `0x57504944` ("WPID")
**Version:** 3 (bump whenever the struct layout changes — mismatched version causes load failure and factory defaults are applied automatically)
**Address:** 0 (base of EEPROM emulation region)
**Size allocated:** 256 bytes

### NvmSettings Struct Layout

```
Header — 8 bytes:
  uint32_t  magic        0x57504944 "WPID"
  uint8_t   version      3
  uint8_t   nodeId       RS485 station ID (0x01-0xFE), default 0x03
  uint8_t   _pad[2]      4-byte alignment for MotorSettings array

Per-motor block — 52 bytes each, 2 motors = 104 bytes:
  uint8_t   posPathMode  0=shortest arc, 1=constrained (default)
  uint8_t   uiType       UiMotorType: 0=PWM%, 1=VELOCITY, 2=POSITION
  uint8_t   _mpad[2]     4-byte alignment
  float     velKp        velocity PID proportional gain
  float     velKi        velocity PID integral gain
  float     velKd        velocity PID derivative gain
  float     velAccel     velocity setpoint ramp rate (deg/s^2), default 200
  float     velLimit     absolute velocity cap (deg/s) for VELOCITY and POSITION modes
  float     posKp        position PID proportional gain
  float     posKi        position PID integral gain
  float     posKd        position PID derivative gain
  float     traverseVel  position profile traverse speed (deg/s), default 90
  float     posMin       constrained travel lower limit (deg), default 0
  float     posMax       constrained travel upper limit (deg), default 355
  float     zeroOffset   AS5600 raw reading (deg) at mechanical zero

Total: 8 + 2 x 52 = 112 bytes (well within 256-byte allocation)
```

### Settings Workflow

```
Power-up    Settings::begin() loads from flash; if magic/version invalid, factory defaults applied (not auto-saved)
Testing     Adjust gains and cal freely via console commands — all changes are RAM-only
Persist     Type "save" (console) or press SAVE in CONFIG screen
Restore     Type "load" to discard RAM changes and reload from flash
Reset       Type "defaults" then "save" to restore factory defaults to flash
```

### What Is Not Saved

The **absolute position accumulator** (`g_measPosAbs`) is RAM-only and resets to 0 at power-up. Run `zero <m>` after each power cycle, with the mechanism at its mechanical home position, before issuing position commands.

---

## Console Commands

Connect via USB serial at **115200 baud**. Commands are case-insensitive. Motor index `m` is 0 or 1. Servo index `s` is 0 or 1.

### Motor Commands (require motor index `m`)

| Command | Description |
|---------|-------------|
| `v <m> <deg/s>` | Set VELOCITY mode, target in deg/s (+CW, -CCW) |
| `va <m> <deg/s^2>` | Set velocity setpoint ramp rate (acceleration limit) |
| `p <m> <deg>` | Set POSITION mode, target absolute degrees |
| `p <m> <deg> <vel>` | POSITION mode with explicit traverse velocity |
| `pv <m> <deg/s>` | Set position traverse velocity |
| `pmode <m> 0` | Position path: shortest arc (no travel limits) |
| `pmode <m> 1` | Position path: constrained (target clamped to posMin/posMax) |
| `plim <m> <lo> <hi>` | Set constrained travel limits in absolute degrees |
| `r <m> <0-255>` | MANUAL mode: raw PWM byte (127=coast, 0=full reverse, 255=full forward) |
| `stop <m>` | Coast motor m, enter MANUAL mode |
| `stop` | Coast all motors |
| `gains` | Print all PID gains for all motors |
| `gains <m>` | Print gains for motor m only |
| `kp <m> <val>` | Velocity PID Kp |
| `ki <m> <val>` | Velocity PID Ki (also resets integrator) |
| `kd <m> <val>` | Velocity PID Kd |
| `pkp <m> <val>` | Position PID Kp |
| `pki <m> <val>` | Position PID Ki (also resets integrator) |
| `pkd <m> <val>` | Position PID Kd |
| `zero <m>` | Set current encoder position as mechanical zero; resets absolute accumulator to 0 (RAM only) |

### Servo Commands

| Command | Description |
|---------|-------------|
| `servo <s> <0-180>` | Set servo s target angle in degrees |
| `servor <s> <deg/s>` | Set servo s ramp rate in deg/s |

### Configuration Commands

| Command | Description |
|---------|-------------|
| `config` | Print all UI configuration parameters (nodeId, per-motor type/velLimit/traverseVel) |
| `nodeid` | Print current RS485 station ID |
| `nodeid <id>` | Set RS485 station ID (0x01-0xFE; hex accepted e.g. 0x05) |
| `mtype <m>` | Print motor m UI type |
| `mtype <m> <0\|1\|2>` | Set motor m UI type: 0=PWM%, 1=VELOCITY, 2=POSITION |
| `mvellim <m>` | Print motor m velocity limit |
| `mvellim <m> <val>` | Set motor m velocity limit in deg/s (range 10-720) |
| `mtravvel <m>` | Print motor m traverse velocity |
| `mtravvel <m> <val>` | Set motor m traverse velocity in deg/s (range 10-720) |

### NVM Commands

| Command | Description |
|---------|-------------|
| `save` | Capture RAM state and write all settings to flash |
| `load` | Reload settings from flash, discard RAM changes, apply to all motors |
| `defaults` | Apply factory defaults in RAM for all motors (type `save` to persist) |

### Data Logger Commands

| Command | Description |
|---------|-------------|
| `freeze` | Stop logging, coast all motors, dump CSV buffer to USB serial |
| `resume` | Clear buffer and restart logging |

The 500-record circular buffer logs Motor 0 at 50 Hz (approximately 10 seconds of history). CSV columns: `time_ms`, `setpoint`, `velocity_dps`, `error`, `pid_out`, `angle_raw`, `duty`.

### First-Time Setup Sequence

```
; 1. Verify encoders found at startup — look for [WARN] messages
; 2. Test motor directions
r 0 200     ; Motor 0 forward (~57%) — should be CW
r 0 127     ; Motor 0 coast
r 1 200     ; Motor 1 forward
r 1 127
; 3. Set mechanical zero
zero 0      ; move mechanism to home, then run
zero 1
; 4. Set travel limits (example)
plim 0 0 355
plim 1 0 355
; 5. Tune PID gains
kp 0 0.8
ki 0 0.1
; 6. Persist
save
```

---

## Build and Flash

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- Earle Philhower `arduino-pico` platform — installed automatically on first build via the `platform` URL in `platformio.ini`

### Board Selection

Edit `platformio.ini` and set the `board` field to match your hardware:

```ini
board = rpipicow    ; Pico W   (RP2040)  — current default in platformio.ini
board = rpipico     ; Pico     (RP2040)
board = rpipico2w   ; Pico 2 W (RP2350)
board = rpipico2    ; Pico 2   (RP2350)
```

### Dependencies (auto-installed by PlatformIO)

```ini
lib_deps =
    https://github.com/adafruit/Adafruit-ST7735-Library.git
    https://github.com/adafruit/Adafruit-GFX-Library.git
    https://github.com/adafruit/Adafruit_BusIO.git
```

The `arduino-pico` Servo library is part of the core and requires no separate dependency.

### Build

```bash
pio run
```

### Flash via USB (UF2 bootloader)

Hold the BOOTSEL button on the Pico while plugging in USB — it mounts as a USB mass storage device. Then:

```bash
pio run --target upload
```

Or copy the generated `.uf2` file from `.pio/build/rpipico/` to the mounted drive manually.

### Serial Monitor

```bash
pio device monitor --baud 115200
```

No external UART adapter required — the Pico uses USB CDC.

---

## Implementation Status and Known Gaps

This section is the handoff document for future implementation sessions. It captures what is fully working, what is designed but not coded, and what needs correction.

### Implemented and Working

- Core 0 / Core 1 dual-core split (control loop vs. display + serial output)
- BTS7960 H-bridge PWM drive (25 kHz, complementary, 5000-step resolution)
- AS5600 magnetic encoder reading on dual I2C buses (Wire / Wire1)
- Velocity measurement — 3-sample sliding window, 60 ms window at 50 Hz
- Multi-turn absolute position accumulation (short-arc delta integration)
- Velocity PID with setpoint ramping (velAccel deg/s^2)
- Position PID cascade (motion profile at traverseVel -> velocity setpoint -> PWM)
- Servo ramp control via arduino-pico Servo library
- RS485 AnimCom slave receiver — CONTROL_STATE, MANUAL_SINGLE, TRIGGER_EFFECT, MANUAL_BULK
- ManualHold timeout (400 ms per DC motor channel)
- AnimCom watchdog (5000 ms global timeout, coasts all outputs)
- RUN_AUTO built-in sine-wave animation patterns (0/1, 2, 3)
- Display driver: GFXcanvas16 full-screen framebuffer, SPI blit at 5 Hz from Core 1
- Display: top-level menu screen (4 items x 36 px + 16 px header = 160 px)
- Display: status screen layout (motor sections, servo rows, status bar with AnimCom info)
- Display: CONFIG screen (8 items x 18 px + 16 px header = 160 px)
- CONFIG screen: scroll, edit, save to flash with immediate applySettings()
- MANUAL mode motor adjustment via encoder (M0 -> M1 -> EXIT button-cycle, with per-uiType adjustment)
- NVM settings version 3 (nodeId, per-motor uiType, velLimit, traverseVel, PID gains, calibration)
- USB serial console — all motor, servo, config, NVM, and logger commands
- Endstop monitor hardware configuration (EndstopMonitor::begin())
- Full-quadrature-cycle encoder debounce (0x87 CW / 0x4B CCW patterns)
- 50 ms button dead-time debounce
- Station-specific animation dispatch (`src/StationAnim.h/cpp`) keyed on NVM nodeId
- Station 7 (Meter): needle position from showIntensity (0%=0°, 100%=180°) + ±5° / 2 s oscillation
- Station 7 power-on default: 90° (50%) with oscillation active, no controller required
- Station 7 watchdog recovery: resumes default 90° animation rather than coasting to STOP

### Not Yet Implemented — Design Intent Documented Above

**1. Default startup state must be ANIMCOM, not DISABLED.**

Currently:
```cpp
static volatile bool     g_inMenu   = true;
static volatile AppState g_appState = STATE_DISABLED;
```

Target:
```cpp
static volatile bool     g_inMenu   = false;
static volatile AppState g_appState = STATE_ANIMCOM;
```

And clear the `g_lastManualCmdMs[]` timestamps on startup just as is done when entering ANIMCOM from the menu. The system should boot directly into the ANIMCOM status screen displaying live data, with no manual menu navigation required for normal show operation.

**2. MANUAL mode full cursor-scroll UI for all four channels.**

The current implementation uses successive button presses to cycle M0 -> M1 -> EXIT. The design intent is a 5-item cursor (Motor 1, Motor 2, Servo 1, Servo 2, EXIT) scrolled by the encoder. Servo channels are not yet selectable or adjustable via the encoder in MANUAL mode. The full ADJUST mode with per-channel enter/exit is partially implemented for motors only.

**3. ~~ENCODER OFFLINE alert in motor sections.~~ — Implemented.**

When a motor's `uiType` requires an encoder (VELOCITY or POSITION) and `AS5600::readAngle()` returns `0xFFFF`, `g_encoderOffline[m]` is set in the Core 0 control loop. `_drawMotorSection()` checks `encFault = encoderOffline[m] && mode[m] != 0` and replaces the primary value with `OFFLINE` (red, scale-2) and detail line 1 with `encoder offline` (red, scale-1).

**4. ~~CONFIG screen dedicated EXIT item.~~ — Implemented.**

CONFIG screen now has 9 items (0-8): items 0-6 are editable settings, item 7 is `>>> SAVE` (saves and exits), item 8 is `>>> EXIT` (reloads settings from flash, resets PIDs, returns to menu without saving). Row height was reduced from 18 px to 16 px to fit: `16 + 9 × 16 = 160 px` exactly.

**5. DISABLED state cursor design.**

The DISABLED screen should place the cursor permanently on an EXIT button. The encoder should not scroll to motor sections in DISABLED state — all outputs are frozen. Only the EXIT action (returns to top menu) should be reachable via the button.

**6. ANIMCOM state dedicated COMM section.**

The design intent includes a visible COMM status area showing RS485 bus live/timeout state separately from the compact bottom status bar. The bottom bar text content is correct; the expanded layout was deferred.

**7. Audio playback via TRIGGER_EFFECT.**

`onAnimTriggerEffect()` currently logs to USB serial only. A DFPlayer Mini on a second UART, or I2S audio, is a future hardware and software task.

**8. I2CSlave legacy cleanup.**

`I2CSlave::begin()` is intentionally not called (Wire1 is I2C master for Motor 1 encoder). `I2CSlave` in-memory state is still referenced in the control loop for Motor 0 legacy compatibility. The `I2CSlave` dependency can be removed once confirmed fully retired, simplifying the codebase.
