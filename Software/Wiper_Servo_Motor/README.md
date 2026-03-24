# Dual Wiper Motor Controller

Closed-loop brushless windshield wiper motor controller running on a Raspberry Pi Pico (RP2040). Controls two independent motors simultaneously with velocity control, multi-turn position control, and raw manual drive, all tunable live over USB serial.

---

## Table of Contents

1. [Hardware](#hardware)
2. [Software Architecture](#software-architecture)
3. [Building and Flashing](#building-and-flashing)
4. [First-Time Setup and Calibration](#first-time-setup-and-calibration)
5. [Control Modes](#control-modes)
6. [Multi-Turn Position Control](#multi-turn-position-control)
7. [PID Tuning](#pid-tuning)
8. [Console Command Reference](#console-command-reference)
9. [Data Logging](#data-logging)
10. [Non-Volatile Settings](#non-volatile-settings)
11. [Source File Overview](#source-file-overview)

---

## Hardware

### Components

| Part | Qty | Description |
|------|-----|-------------|
| Raspberry Pi Pico | 1 | RP2040 microcontroller, dual-core Cortex-M0+ |
| BTS7960 (or equivalent) | 2 | H-bridge motor driver, two PWM inputs per driver |
| AS5600 | 2 | 12-bit magnetic angle encoder, I2C |
| 6 mm diametrically magnetized magnet | 2 | One per motor shaft, centered over AS5600 |
| SPI monochrome OLED | 1 | 1.3" display (SSD1306/SH1106 or compatible) |
| Rotary encoder with pushbutton | 1 | Integrated on OLED board |
| RS485 transceiver | 1 | SN75176 / MAX485 or equivalent, for DMX512 |
| RC servo (optional) | 0–2 | Standard 50 Hz PWM servo on GPIO26/27 |

### Pin Assignment

#### Motor PWM (BTS7960)

| Signal | GPIO | PWM Slice | Notes |
|--------|------|-----------|-------|
| Motor 0 IN1 | 0 | Slice 0A | Forward drive |
| Motor 0 IN2 | 1 | Slice 0B | Reverse drive |
| Motor 1 IN1 | 2 | Slice 1A | Forward drive |
| Motor 1 IN2 | 3 | Slice 1B | Reverse drive |

Both slices are driven at the same 25 kHz base frequency via `analogWriteFreq()`.

#### Encoders (AS5600, I2C master)

| Signal | GPIO | Bus | Notes |
|--------|------|-----|-------|
| Motor 0 SDA | 4 | I2C0 | 400 kHz |
| Motor 0 SCL | 5 | I2C0 | 400 kHz |
| Motor 1 SDA | 6 | I2C1 | 400 kHz |
| Motor 1 SCL | 7 | I2C1 | 400 kHz |

Both sensors share the AS5600 address (0x36) — isolation is achieved by placing them on separate I2C buses.

#### Endstop Switches (active-low, internal pull-up)

| Signal | GPIO |
|--------|------|
| Motor 0 End A | 8 |
| Motor 0 End B | 9 |
| Motor 1 End A | 10 |
| Motor 1 End B | 11 |

#### DMX512 Serial Interface (UART0, RS485)

| Signal | GPIO | Notes |
|--------|------|-------|
| DE (direction enable) | 15 | HIGH = transmit, LOW = receive |
| TX | 16 | UART0 TX → transceiver DI |
| RX | 17 | UART0 RX ← transceiver RO |

250000 baud, 8N2. Tie DE+RE together on the transceiver.

#### Local UI — Rotary Encoder with Pushbutton

| Signal | GPIO | Notes |
|--------|------|-------|
| ENC_CLK (A) | 12 | Interrupt-driven gray-code |
| ENC_DT (B) | 13 | Interrupt-driven gray-code |
| ENC_SW (button) | 14 | Active-low, falling-edge interrupt |

#### Local UI — SPI OLED Display (SPI0)

| Signal | GPIO | Notes |
|--------|------|-------|
| SCK | 18 | SPI0 clock |
| MOSI | 19 | SPI0 data |
| DC | 20 | Data/Command select |
| CS | 21 | Chip select (active-low) |
| RST | 22 | Reset (active-low; tie HIGH if unused) |

#### RC Servo Outputs (optional)

Standard 50 Hz PWM, 1–2 ms pulse width (0–180°). Uses the arduino-pico `Servo` library — **not** `analogWrite()` — so the motor PWM slices are not affected.

| Signal | GPIO | PWM Slice | Notes |
|--------|------|-----------|-------|
| Servo 0 | 26 | Slice 5A | Commanded via `servo 0 <angle>` |
| Servo 1 | 27 | Slice 5B | Commanded via `servo 1 <angle>` |

#### Miscellaneous

| Signal | GPIO |
|--------|------|
| Status LED | 25 (LED_BUILTIN) |

### PWM Parameters

- Frequency: **25 kHz** (above audible range, suitable for BTS7960)
- Resolution: **5000 steps** (PWM_WRAP = 4999)
- Direction: IN1 active = forward (CW), IN2 active = reverse (CCW)

### AS5600 Notes

- 12-bit absolute encoder: 0–4095 counts per revolution (0.088°/count).
- **Single-turn only** — the AS5600 has no built-in revolution counter. Multi-turn position tracking is handled entirely in firmware by accumulating per-sample deltas (see [Multi-Turn Position Control](#multi-turn-position-control)).
- The magnet must be mounted concentrically on the motor shaft within ~0.5–3 mm of the IC surface.
- The encoder zero point is arbitrary. Use the `zero <m>` command to calibrate.

---

## Software Architecture

### Dual-Core Assignment

| Core | Responsibilities |
|------|-----------------|
| Core 0 | 50 Hz control loop (both motors), USB serial console input (non-blocking) |
| Core 1 | 2 Hz status print to serial |

All `Serial.print()` calls are on Core 1 so they can never stall the Core 0 control loop. Cross-core state is shared via `volatile` 32-bit variables (atomic on Cortex-M0+).

### Control Loop (50 Hz)

Each 20 ms cycle on Core 0 runs the following for **each motor independently**:

1. Read AS5600 raw angle (12-bit, 0–4095)
2. Accumulate absolute multi-turn position (delta integration with short-arc detection)
3. Compute modular position (0–360°) for display
4. Compute velocity via 3-sample sliding-window average (60 ms window, ~30 ms lag)
5. Run PID update for the active mode
6. Map PID output → full-resolution PWM duty (±4999 steps)

Motor 0 additionally appends one record to the circular data logger each cycle.

### PID Structure

**Velocity mode** — single loop:
```
setpoint (deg/s) → [ramp at velAccel] → [Velocity PID] → PWM duty
```

**Position mode** — cascade with motion profile:
```
target pos (abs deg) → [Motion Profile at traverseVel] → commanded pos
commanded pos → [Position PID] → velocity setpoint
velocity setpoint → [Velocity PID] → PWM duty
```

The motion profile ramps `commandedPos` toward `targetPos` at `traverseVel` deg/s, so the position PID always sees a small tracking error and the velocity PID never saturates.

### Multi-Turn Absolute Position

The position control operates entirely in **absolute multi-turn degree space**. The accumulator integrates short-arc encoder deltas from the last `zero` command and can represent any value — positive or negative, across any number of revolutions.

The short-arc detection clamps each 20 ms sample to ±180° of apparent motion, which limits reliable maximum speed to ~1500 RPM. Wiper motor speeds are well within this limit.

**Important:** The absolute position accumulator is RAM-only and resets to 0 at power-up. `zero <m>` must be run after each power cycle before issuing position commands.

### PID Implementation Details

- **Derivative on measurement** — eliminates derivative kick on setpoint changes
- **Conditional anti-windup** — integrator only accumulates when output is not saturated in the same direction as the error
- **Velocity setpoint ramping** — `commandedVel` ramps toward `targetVel` at `velAccel` deg/s²

---

## Building and Flashing

### Requirements

- [PlatformIO](https://platformio.org/) (VS Code extension recommended)
- Earle Philhower's [arduino-pico](https://github.com/earlephilhower/arduino-pico) platform (installed automatically via `platformio.ini`)

### Build

```
pio run
```

### Flash

Hold the BOOTSEL button on the Pico while connecting USB, then:

```
pio run --target upload
```

Or drag the `.uf2` file from `.pio/build/rpipico/` onto the Pico's USB mass-storage drive.

### Serial Monitor

```
pio device monitor
```

Baud rate: **115200**. The Pico uses USB CDC — no external UART adapter needed.

---

## First-Time Setup and Calibration

Follow these steps on a fresh Pico or after a firmware update that changes the settings version.

### Step 1 — Verify Hardware

Connect serial monitor. You should see:

```
[INFO] Dual wiper motor controller ready
[NVM] No valid settings found — factory defaults applied
[NVM] Type 'save' to persist current settings
```

If you see `[WARN] Motor N AS5600 not found` check SDA/SCL wiring and confirm the magnet is within range of the encoder.

### Step 2 — Verify Motor Direction

Test each motor with manual commands (motor index is the first argument):

```
r 0 200    ; Motor 0 ~57% forward — should spin CW
r 0 127    ; Motor 0 coast
r 1 200    ; Motor 1 ~57% forward — should spin CW
r 1 127    ; Motor 1 coast
```

If a motor spins CCW when it should spin CW, swap that driver's IN1 and IN2 wires.

### Step 3 — Verify Encoder Direction

While each motor spins CW, the status line should show a **positive** velocity for that motor. If velocity is negative while spinning CW, the magnet is mounted with reversed polarity — flip it 180°.

### Step 4 — Set the Travel Limits

For each motor, set `posMin` and `posMax` to match your mechanism's physical travel in **absolute degrees from the zero point**. For a chain/gear drive that spans multiple revolutions, set the limit accordingly:

```
plim 0 0 1800    ; Motor 0: up to 5 full turns (5 × 360°)
plim 1 0 270     ; Motor 1: limited to 270° sweep
```

### Step 5 — Set Mechanical Zero

Move each mechanism to its true mechanical zero position, then run:

```
zero 0    ; set Motor 0 zero at current position
zero 1    ; set Motor 1 zero at current position
```

The absolute position accumulator is reset to 0 and the modular display will read near 0°. **This must be repeated every power cycle** before issuing position commands.

### Step 6 — Tune PID Gains

See [PID Tuning](#pid-tuning) below.

### Step 7 — Save to Flash

```
save
```

All gains, travel limits, path mode, and zero offsets are written to flash per motor and restored on every power-up. The absolute position accumulator itself is not saved — `zero` must still be run after each power cycle.

---

## Control Modes

All motor commands require an explicit motor index `m` (0 or 1) as the first argument.

### Manual Mode

Direct PWM control. The PID is inactive.

```
r 0 200    ; Motor 0 ~57% forward
r 0 127    ; Motor 0 coast
r 1 50     ; Motor 1 ~61% reverse
```

The DMX-style encoding maps 0–255 → full-reverse to full-forward with 127 as the coast midpoint.

### Velocity Mode

The velocity PID regulates angular velocity in degrees per second. Positive = CW.

```
v 0 90     ; Motor 0 — 90 deg/s CW
v 1 -45    ; Motor 1 — 45 deg/s CCW
v 0 0      ; Motor 0 — hold at zero velocity
stop 0     ; Motor 0 — coast
stop       ; all motors coast
```

### Position Mode

The cascade controller moves to a target absolute angle and holds it. Targets are in **absolute degrees from the last `zero` command** and can exceed 360°.

```
p 0 180     ; Motor 0 to 180°
p 0 940     ; Motor 0 to 940° (≈ 2.6 revolutions)
p 1 270     ; Motor 1 to 270°
p 0 0 45    ; Motor 0 to 0° at 45 deg/s traverse speed
stop 0      ; Motor 0 coast
```

In **constrained mode** (default, `pmode <m> 1`), the motor travels linearly in absolute degree space and targets outside `[posMin, posMax]` are clamped. This is correct for limited-travel mechanisms and chain/gear drives where overtravel would cause damage.

In **shortest-path mode** (`pmode <m> 0`), there are no travel limits. The target is accepted as any absolute degree value. **Note:** this mode does not compute a "short way around" — it simply accepts the target as-is with no clamping.

---

## Multi-Turn Position Control

For chain/gear or other multi-revolution mechanisms, position control works across any number of turns.

### Setup Sequence

```
zero 0               ; at mechanical home — resets accumulator to 0
plim 0 0 1800        ; set travel limit (e.g. 5 turns = 1800°)
save                 ; persist limits to flash
p 0 940              ; command 940° from home (≈ 2.6 turns)
p 0 1800             ; command to far end of travel
p 0 0                ; return to home
```

### After Every Power Cycle

The absolute position accumulator is not stored in flash. After power-up:

```
zero 0    ; move mechanism to home first, then run this
```

Then position commands work normally.

### Travel Limits and Path Mode

Use **constrained mode with absolute limits** (`pmode <m> 1` + `plim`) for mechanisms with physical stops. The limits are in absolute degrees from zero and can be any value:

```
plim 0 0 360     ; single revolution
plim 0 0 1440    ; four revolutions
plim 0 -180 180  ; ±180° from home
```

---

## PID Tuning

All gain commands require a motor index.

### Velocity PID

Start with `ki 0` and `kd 0`, then:

1. Increase `kp` until you get a fast response with acceptable overshoot (~10–20%).
2. Add `ki` slowly to eliminate steady-state error.
3. `kd` is usually not needed for velocity control.

```
gains 0        ; print Motor 0 current values
kp 0 0.8
ki 0 0.4
kd 0 0.0
va 0 200       ; velocity ramp rate deg/s²
v 0 100        ; test step
v 0 50
v 0 0
```

### Position PID

Tune velocity mode first. Then:

1. Set `traverseVel` to a comfortable speed.
2. Increase `pkp` until position tracks the ramp cleanly.
3. `pki` and `pkd` are rarely needed when the motion profile is in use.

```
pkp 0 3.0
pki 0 0.0
pkd 0 0.0
pv 0 90        ; traverse speed deg/s
p 0 180
p 0 0
```

Use `freeze` / `resume` to capture CSV data and plot for detailed analysis.

### Factory Default Gains (applied to all motors)

| Parameter | Default | Description |
|-----------|---------|-------------|
| Vel kp | 0.6 | Velocity PID proportional gain |
| Vel ki | 0.4 | Velocity PID integral gain |
| Vel kd | 0.0 | Velocity PID derivative gain |
| velAccel | 200 deg/s² | Velocity setpoint ramp rate |
| Pos kp | 3.0 | Position PID proportional gain |
| Pos ki | 0.0 | Position PID integral gain |
| Pos kd | 0.0 | Position PID derivative gain |
| traverseVel | 90 deg/s | Position profile speed |
| posPathMode | 1 (constrained) | No wrap-around |
| posMin | 0° | Constrained travel lower limit |
| posMax | 355° | Constrained travel upper limit |
| zeroOffset | 0° | Mechanical zero calibration |

---

## Console Command Reference

Connect at 115200 baud. Commands are case-insensitive. Press Enter to send.

`m` = motor index (0 or 1). All motor commands require this prefix.

### Motor Commands

| Command | Description |
|---------|-------------|
| `v <m> <deg/s>` | Velocity mode — target in deg/s (+CW, −CCW) |
| `va <m> <deg/s²>` | Velocity setpoint ramp rate (default 200) |
| `p <m> <deg>` | Position mode — absolute target in degrees (any value, multi-turn capable) |
| `p <m> <deg> <vel>` | Position mode — with explicit traverse speed |
| `pv <m> <deg/s>` | Set position traverse speed (default 90) |
| `pmode <m> 0` | Shortest-path mode — no travel limits |
| `pmode <m> 1` | Constrained mode (default) — target clamped to `[posMin, posMax]` |
| `plim <m> <lo> <hi>` | Set constrained travel limits in absolute degrees |
| `r <m> <0–255>` | Manual raw PWM (127=coast, 0=full reverse, 255=full forward) |
| `stop <m>` | Coast motor m, enter manual mode |
| `stop` | Coast all motors |

### PID Gain Commands

| Command | Description |
|---------|-------------|
| `gains` | Print all gains for all motors |
| `gains <m>` | Print gains for motor m |
| `kp <m> <n>` | Velocity PID Kp |
| `ki <m> <n>` | Velocity PID Ki (also resets integrator) |
| `kd <m> <n>` | Velocity PID Kd |
| `pkp <m> <n>` | Position PID Kp |
| `pki <m> <n>` | Position PID Ki (also resets integrator) |
| `pkd <m> <n>` | Position PID Kd |

### Calibration and Settings Commands

| Command | Description |
|---------|-------------|
| `zero <m>` | Set current position as mechanical zero; resets absolute accumulator to 0 (RAM only) |
| `save` | Write all motor settings and calibration to flash |
| `load` | Reload settings from flash, discarding unsaved RAM changes (applied to all motors) |
| `defaults` | Apply factory defaults in RAM for all motors (follow with `save` to persist) |

### RC Servo Commands

`s` = servo index (0 or 1).

| Command | Description |
|---------|-------------|
| `servo <s> <angle>` | Move servo s to angle (0–180°) |

### Data Logger Commands

| Command | Description |
|---------|-------------|
| `freeze` | Stop recording, coast all motors, dump CSV to serial console |
| `resume` | Clear buffer, restart recording |

---

## Data Logging

A 500-record circular buffer runs continuously at 50 Hz (~10 seconds of history). Logs Motor 0 data.

### Capture a Log

1. Run a motor in the desired mode (e.g. `v 0 100`).
2. Type `freeze` — all motors coast and the CSV is printed to the console.
3. Copy the console output and paste into a `.csv` file.
4. Open in Excel and chart the columns.
5. Type `resume` to restart logging.

### CSV Columns

| Column | Unit | Description |
|--------|------|-------------|
| `time_ms` | ms | millis() at sample time |
| `setpoint` | deg/s or deg | Velocity setpoint or commanded profile position |
| `velocity_dps` | deg/s | Measured angular velocity |
| `error` | deg/s or deg | Setpoint − measurement |
| `pid_out` | ±127 | Velocity PID output |
| `angle_raw` | 0–4095 | Raw AS5600 encoder count |
| `duty` | ±4999 | Signed PWM duty applied to motor |

---

## Non-Volatile Settings

Settings are stored in RP2040 flash using EEPROM emulation (Earle Philhower arduino-pico core). A magic number (`0x57504944` = "WPID") and version byte (currently `2`) validate the stored data. If the version does not match (e.g. after a firmware update that changes the settings layout), factory defaults are applied automatically.

Settings are stored **independently per motor**.

### What Is Saved (per motor)

| Setting | Command |
|---------|---------|
| Velocity PID gains and ramp rate | `kp/ki/kd/va` |
| Position PID gains and traverse speed | `pkp/pki/pkd/pv` |
| Position path mode | `pmode` |
| Travel limits | `plim` |
| Mechanical zero offset | `zero` |

### What Is NOT Saved

The **absolute position accumulator** (`g_measPosAbs`) is RAM-only and resets to 0 at power-up. Run `zero <m>` after each power cycle before issuing position commands.

### Workflow

```
; Change something in RAM
kp 0 1.2
plim 0 0 1800
zero 0

; Verify it works, then persist
save

; To discard RAM changes and revert to last saved
load

; To start over with factory defaults (does not automatically save)
defaults
save
```

---

## Source File Overview

| File | Description |
|------|-------------|
| `src/main.cpp` | Main application: dual-motor control loop, console parser, dual-core setup |
| `src/Config.h` | All pin assignments, PWM parameters, watchdog timeout |
| `src/PIDController.h/.cpp` | Generic PID: derivative-on-measurement, conditional anti-windup |
| `src/AS5600.h/.cpp` | Instance-based AS5600 driver — one instance per I2C bus |
| `src/MotorPWM.h/.cpp` | 25 kHz PWM generation for two motors; signed full-resolution duty path |
| `src/EndstopMonitor.h/.cpp` | Active-low endstop monitoring for 2 switches per motor |
| `src/LocalUI.h/.cpp` | SPI OLED display pins + interrupt-driven rotary encoder/button driver |
| `src/DataLogger.h/.cpp` | 500-record circular buffer; CSV freeze/dump |
| `src/Settings.h/.cpp` | Flash-backed NVM; independent per-motor storage (NVM version 2) |
| `src/MotorControlProtocol.h` | Legacy I2C register map (slave interface retired; file retained for reference) |
| `src/I2CSlave.h/.cpp` | Legacy I2C1 slave (retired — Wire1 is now Motor 1 encoder master; not initialised) |
| `platformio.ini` | PlatformIO build config — rpipico, arduino-pico/earlephilhower |
