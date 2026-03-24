/*
  RP2040 Dual Brushless Wiper Motor Controller
  =============================================
  Closed-loop motor controller for two brushless windshield wiper motors.

  Hardware:
    Motor 0 driver : BTS7960 — IN1=GPIO0, IN2=GPIO1  (25 kHz PWM, slice 0)
    Motor 1 driver : BTS7960 — IN1=GPIO2, IN2=GPIO3  (25 kHz PWM, slice 1)
    Motor 0 encoder: AS5600  — SDA=GPIO4, SCL=GPIO5  (I2C0 master)
    Motor 1 encoder: AS5600  — SDA=GPIO6, SCL=GPIO7  (I2C1 master)
    Endstops       : Motor 0 A/B = GPIO8/9, Motor 1 A/B = GPIO10/11
    Local UI       : Encoder CLK=GPIO12, DT=GPIO13, SW=GPIO14
                     SPI OLED SCK=GPIO18, MOSI=GPIO19, DC=GPIO20,
                               CS=GPIO21, RST=GPIO22

  Note: The former I2C slave interface (GPIO26/27) has been retired.
        Local control is now provided by the OLED + rotary encoder.

  Control modes (per motor):
    MANUAL   — raw speed (0–255) drives PWM directly
    VELOCITY — velocity PID regulates angular velocity (deg/sec)
    POSITION — cascade: position PID → velocity setpoint → velocity PID → PWM

  Core assignment:
    Core 0 — control loop (50 Hz), console input (non-blocking reads only)
    Core 1 — status print (2 Hz)
             Serial output is entirely on Core 1 so it cannot stall Core 0.

  USB Serial console commands  (m = motor index: 0 or 1):
    v  <m> <n>         Velocity mode — target in deg/sec (+CW, −CCW)
    va <m> <n>         Velocity accel limit in deg/s² (default 200)
    p  <m> <n>         Position mode — target angle in degrees
    p  <m> <n> <vel>   Position mode with explicit traverse velocity
    pv <m> <n>         Position traverse velocity in deg/s (default 90)
    pmode <m> <0|1>    Position path: 0=shortest arc, 1=constrained (default)
    plim <m> <lo> <hi> Constrained travel limits in degrees (default 0 355)
    r  <m> <n>         Manual mode — raw PWM 0–255 (127=coast)
    stop <m>           Coast motor m, enter MANUAL mode
    stop               Coast all motors
    gains              Print all PID gains
    gains <m>          Print gains for motor m only
    kp  <m> <n>        Velocity PID Kp
    ki  <m> <n>        Velocity PID Ki (also resets integrator)
    kd  <m> <n>        Velocity PID Kd
    pkp <m> <n>        Position PID Kp
    pki <m> <n>        Position PID Ki (also resets integrator)
    pkd <m> <n>        Position PID Kd
    zero <m>           Set current position as mechanical zero (RAM only)
    servo  <s> <n>     RC servo s (0 or 1) — target angle 0–180°
    servor <s> <n>     RC servo s ramp rate in deg/s (default 90)
    freeze             Stop logging, coast all motors, dump CSV to console
    resume             Clear buffer, restart logging
    save               Write motor 0 settings/cal to non-volatile flash
    load               Reload settings from flash (applied to all motors)
    defaults           Reset all settings to factory defaults (RAM only)
*/

#include <Arduino.h>
#include "Config.h"
#include "MotorPWM.h"
#include "I2CSlave.h"       // retained for in-memory state; begin() not called
#include "AS5600.h"
#include "EndstopMonitor.h"
#include "LocalUI.h"
#include "PIDController.h"
#include "MotorControlProtocol.h"
#include "DataLogger.h"
#include "Settings.h"
#include <Servo.h>
#include "Display.h"

// RC servo instances — attached in setup(); driven by controlLoop() ramp.
static Servo g_servo[NUM_SERVOS];
// Per-servo ramp state — written by Core 0 controlLoop(), read by Core 1 display.
static volatile float g_servoTarget  [NUM_SERVOS] = {90.0f, 90.0f}; // commanded target (deg)
static volatile float g_servoActual  [NUM_SERVOS] = {90.0f, 90.0f}; // current ramped output (deg)
static volatile float g_servoRampRate[NUM_SERVOS] = {90.0f, 90.0f}; // ramp rate (deg/s)

// One encoder instance per I2C bus — both sensors share address 0x36 but
// are isolated on separate buses so they coexist without conflict.
static AS5600 g_enc0(Wire);   // Motor 0 — I2C0 (GPIO4/5)
static AS5600 g_enc1(Wire1);  // Motor 1 — I2C1 (GPIO6/7)

// ---------------------------------------------------------------------------
// Per-motor control state
// All volatile so Core 1 can safely read for status printing.
// 32-bit aligned reads/writes are atomic on Cortex-M0+.
// ---------------------------------------------------------------------------
enum ControlMode  : uint8_t { MANUAL = 0, VELOCITY = 1, POSITION = 2 };
enum PosPathMode  : uint8_t { POS_PATH_SHORTEST = 0, POS_PATH_CONSTRAINED = 1 };

static volatile ControlMode g_mode        [NUM_MOTORS] = {MANUAL,              MANUAL};
static volatile float       g_targetVel   [NUM_MOTORS] = {0.0f,                0.0f};
static volatile float       g_commandedVel[NUM_MOTORS] = {0.0f,                0.0f};
static volatile float       g_velAccel    [NUM_MOTORS] = {200.0f,              200.0f};
static volatile float       g_targetPos   [NUM_MOTORS] = {0.0f,                0.0f};
static volatile float       g_commandedPos[NUM_MOTORS] = {0.0f,                0.0f};
static volatile float       g_traverseVel [NUM_MOTORS] = {90.0f,               90.0f};
static volatile PosPathMode g_posPathMode [NUM_MOTORS] = {POS_PATH_CONSTRAINED, POS_PATH_CONSTRAINED};
static volatile float       g_posMin      [NUM_MOTORS] = {0.0f,                0.0f};
static volatile float       g_posMax      [NUM_MOTORS] = {355.0f,              355.0f};
static volatile float       g_zeroOffset  [NUM_MOTORS] = {0.0f,                0.0f};
static volatile uint16_t    g_lastRawAngle[NUM_MOTORS] = {0,                   0};
static volatile float       g_measVel     [NUM_MOTORS] = {0.0f,                0.0f};
static volatile float       g_measPos     [NUM_MOTORS] = {0.0f,                0.0f};
static volatile float       g_pidOutput   [NUM_MOTORS] = {0.0f,                0.0f};
static volatile float       g_posError    [NUM_MOTORS] = {0.0f,                0.0f};
static volatile float       g_measPosAbs  [NUM_MOTORS] = {0.0f,                0.0f};  // accumulated multi-turn (deg)

// ---------------------------------------------------------------------------
// PID controllers — Core 0 only, not volatile
// ---------------------------------------------------------------------------
static constexpr float POS_VEL_LIMIT = 360.0f;

static PIDController g_velPid[NUM_MOTORS] = {
    PIDController(0.8f, 0.1f, 0.0f, -127.0f,        127.0f),
    PIDController(0.8f, 0.1f, 0.0f, -127.0f,        127.0f)
};
static PIDController g_posPid[NUM_MOTORS] = {
    PIDController(3.0f, 0.0f, 0.0f, -POS_VEL_LIMIT, POS_VEL_LIMIT),
    PIDController(3.0f, 0.0f, 0.0f, -POS_VEL_LIMIT, POS_VEL_LIMIT)
};

// ---------------------------------------------------------------------------
// Settings helpers — NVM stores independent settings per motor.
// ---------------------------------------------------------------------------
static void applySettings() {
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        const NvmSettings::MotorSettings& ms = Settings::s.motor[m];
        g_velPid[m].kp   = ms.velKp;
        g_velPid[m].ki   = ms.velKi;
        g_velPid[m].kd   = ms.velKd;
        g_velAccel[m]    = ms.velAccel;
        g_posPid[m].kp   = ms.posKp;
        g_posPid[m].ki   = ms.posKi;
        g_posPid[m].kd   = ms.posKd;
        g_traverseVel[m] = ms.traverseVel;
        g_posPathMode[m] = (PosPathMode)ms.posPathMode;
        g_posMin[m]      = ms.posMin;
        g_posMax[m]      = ms.posMax;
        g_zeroOffset[m]  = ms.zeroOffset;
    }
}

static void captureSettings() {
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        NvmSettings::MotorSettings& ms = Settings::s.motor[m];
        ms.velKp       = g_velPid[m].kp;
        ms.velKi       = g_velPid[m].ki;
        ms.velKd       = g_velPid[m].kd;
        ms.velAccel    = g_velAccel[m];
        ms.posKp       = g_posPid[m].kp;
        ms.posKi       = g_posPid[m].ki;
        ms.posKd       = g_posPid[m].kd;
        ms.traverseVel = g_traverseVel[m];
        ms.posPathMode = (uint8_t)g_posPathMode[m];
        ms.posMin      = g_posMin[m];
        ms.posMax      = g_posMax[m];
        ms.zeroOffset  = g_zeroOffset[m];
    }
}

// ---------------------------------------------------------------------------
// Core 1 — status print (2 Hz)
// All Serial output lives here so it cannot stall the Core 0 control loop.
// ---------------------------------------------------------------------------
void setup1() {}

void loop1() {
    static uint32_t lastPrintMs = 0;
    uint32_t now = millis();
    if (now - lastPrintMs >= 500) {
        lastPrintMs = now;

        // Build display state from the same volatile snapshot.
        DisplayState ds;
        for (uint8_t si = 0; si < NUM_SERVOS; si++) {
            ds.servoActual[si] = (int)roundf(g_servoActual[si]);
            ds.servoTarget[si] = (int)roundf(g_servoTarget[si]);
        }

        for (uint8_t m = 0; m < NUM_MOTORS; m++) {
            // Snapshot all volatiles for a consistent frame.
            ControlMode mode    = g_mode[m];
            float       measVel = g_measVel[m];
            float       measPos = g_measPos[m];       // modular 0-360
            float       absPos  = g_measPosAbs[m];    // multi-turn accumulator
            float       tgtVel  = g_targetVel[m];
            float       cmdVel  = g_commandedVel[m];
            float       tgtPos  = g_targetPos[m];
            float       cmdPos  = g_commandedPos[m];
            float       travVel = g_traverseVel[m];
            float       pidOut  = g_pidOutput[m];
            float       posErr  = g_posError[m];
            int16_t     duty    = MotorPWM::rawDuty[m];

            // Populate display state for this motor.
            ds.mode[m]         = (uint8_t)mode;
            ds.measVel[m]      = measVel;
            ds.measPosAbs[m]   = absPos;
            ds.targetVel[m]    = tgtVel;
            ds.commandedVel[m] = cmdVel;
            ds.targetPos[m]    = tgtPos;
            ds.posError[m]     = posErr;
            ds.duty[m]         = duty;

            Serial.print("M"); Serial.print(m);

            if (mode == POSITION) {
                Serial.print(" POS");
                Serial.print("  Abs:");  Serial.print(absPos,  1); Serial.print("deg");
                Serial.print("  Cmd:");  Serial.print(cmdPos,  1); Serial.print("deg");
                Serial.print("  Tgt:");  Serial.print(tgtPos,  1); Serial.print("deg");
                Serial.print("  Err:");  Serial.print(posErr,  1); Serial.print("deg");
                Serial.print("  @");     Serial.print(travVel, 1); Serial.print("deg/s");
            } else {
                Serial.print(mode == VELOCITY ? " VEL" : " MAN");
                Serial.print("  Pos:");  Serial.print(measPos, 1); Serial.print("deg");
                Serial.print("  Abs:");  Serial.print(absPos,  1); Serial.print("deg");
                Serial.print("  Vel:");  Serial.print(measVel, 1);
                Serial.print("deg/s("); Serial.print(measVel / 6.0f, 1); Serial.print("RPM)");
                if (mode == VELOCITY) {
                    Serial.print("  Cmd:"); Serial.print(cmdVel, 1); Serial.print("deg/s");
                    Serial.print("  Tgt:"); Serial.print(tgtVel, 1); Serial.print("deg/s");
                }
            }
            Serial.print("  PID:"); Serial.print(pidOut, 1);
            Serial.print("  Duty:"); Serial.println(duty);
        }
        Display::update(ds);
    }

    delay(100);
}

// ---------------------------------------------------------------------------
// Core 0 — setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    for (uint8_t i = 0; i < 20 && !Serial; i++) delay(50);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // I2CSlave::begin() intentionally omitted — Wire1 is now I2C master for
    // Motor 1 AS5600.  I2CSlave in-memory state is still used by console
    // command handlers; the watchdog in loop1() is harmlessly inactive.
    bool mag0Found = g_enc0.begin(AS5600_0_SDA_PIN, AS5600_0_SCL_PIN);
    bool mag1Found = g_enc1.begin(AS5600_1_SDA_PIN, AS5600_1_SCL_PIN);
    MotorPWM::begin();
    EndstopMonitor::begin();
    LocalUI::begin();

    const uint8_t servoPins[NUM_SERVOS] = { SERVO_0_PIN, SERVO_1_PIN };
    for (uint8_t i = 0; i < NUM_SERVOS; i++) g_servo[i].attach(servoPins[i]);

    Display::begin();
    DataLogger::begin();
    Settings::begin();
    applySettings();
    I2CSlave::lastCmdMs = millis();

    Serial.println("[INFO] Dual wiper motor controller ready");
    if (!mag0Found) Serial.println("[WARN] Motor 0 AS5600 not found — check wiring");
    if (!mag1Found) Serial.println("[WARN] Motor 1 AS5600 not found — check wiring");
    Serial.println("[INFO] Commands require motor index m (0 or 1): v/va/p/pv/pmode/plim/r/stop/gains/kp/ki/kd/pkp/pki/pkd/zero");
    Serial.println("[INFO] Non-indexed: freeze  resume  save  load  defaults");
}

// ---------------------------------------------------------------------------
// Velocity measurement — sliding window average (VEL_WINDOW samples).
// Each motor maintains independent state via the m index.
// ---------------------------------------------------------------------------
static constexpr uint8_t VEL_WINDOW = 3;  // 3×20ms = 60ms window

static float measureVelocity(uint8_t m, uint16_t newAngle, float dt) {
    static uint16_t prevAngle  [NUM_MOTORS]              = {};
    static bool     firstSample[NUM_MOTORS]              = {true, true};
    static float    wDelta     [NUM_MOTORS][VEL_WINDOW]  = {};
    static float    wTime      [NUM_MOTORS][VEL_WINDOW]  = {};
    static uint8_t  wIdx       [NUM_MOTORS]              = {};
    static uint8_t  filled     [NUM_MOTORS]              = {};
    static float    sumDelta   [NUM_MOTORS]              = {};
    static float    sumTime    [NUM_MOTORS]              = {};

    if (firstSample[m]) {
        prevAngle[m]   = newAngle;
        firstSample[m] = false;
        return 0.0f;
    }

    int16_t raw = (int16_t)newAngle - (int16_t)prevAngle[m];
    if      (raw >  2048) raw -= 4096;
    else if (raw < -2048) raw += 4096;
    prevAngle[m] = newAngle;

    uint8_t i = wIdx[m];
    sumDelta[m] -= wDelta[m][i];
    sumTime[m]  -= wTime[m][i];
    wDelta[m][i] = (float)raw;
    wTime[m][i]  = dt;
    sumDelta[m] += wDelta[m][i];
    sumTime[m]  += wTime[m][i];
    wIdx[m] = (i + 1) % VEL_WINDOW;
    if (filled[m] < VEL_WINDOW) filled[m]++;

    if (sumTime[m] <= 0.0f) return 0.0f;
    return sumDelta[m] / sumTime[m] * (360.0f / 4096.0f);
}

// ---------------------------------------------------------------------------
// Control loop — 50 Hz gate, runs both motors each tick.
// ---------------------------------------------------------------------------
static void controlLoop(uint32_t now) {
    static uint32_t lastMs           = 0;
    static uint8_t  prevMode[NUM_MOTORS] = {MANUAL, MANUAL};

    if (now - lastMs < 20) return;
    float dt = (now - lastMs) * 0.001f;
    lastMs = now;

    AS5600* encoders[NUM_MOTORS] = { &g_enc0, &g_enc1 };

    for (uint8_t m = 0; m < NUM_MOTORS; m++) {

        // --- Encoder read ---------------------------------------------
        uint16_t rawAngle = encoders[m]->readAngle();
        if (rawAngle == 0xFFFF) continue;   // skip this motor this tick
        if (m == 0) I2CSlave::encoderAngle = rawAngle;  // legacy compat
        g_lastRawAngle[m] = rawAngle;

        // Accumulate absolute multi-turn position.
        // Uses the same short-arc detection as measureVelocity so a single
        // 20 ms sample can never appear as more than a half-revolution step.
        {
            static uint16_t prevRaw  [NUM_MOTORS] = {};
            static bool     firstRead[NUM_MOTORS] = {true, true};
            if (firstRead[m]) {
                prevRaw[m]   = rawAngle;
                firstRead[m] = false;
            } else {
                int16_t delta = (int16_t)rawAngle - (int16_t)prevRaw[m];
                if      (delta >  2048) delta -= 4096;
                else if (delta < -2048) delta += 4096;
                prevRaw[m] = rawAngle;
                g_measPosAbs[m] += delta * (360.0f / 4096.0f);
            }
        }

        // Modular position (0–360°) — kept for I2C slave register and display
        float curPosDeg = rawAngle * (360.0f / 4096.0f) - g_zeroOffset[m];
        if      (curPosDeg <    0.0f) curPosDeg += 360.0f;
        else if (curPosDeg >= 360.0f) curPosDeg -= 360.0f;
        g_measPos[m] = curPosDeg;

        // --- Velocity measurement -------------------------------------
        float meas = measureVelocity(m, rawAngle, dt);
        g_measVel[m] = meas;
        if (m == 0) I2CSlave::measVelTenths = (int16_t)(meas * 10.0f);

        // --- Consume I2C commands (motor 0 only, legacy) --------------
        if (m == 0 && I2CSlave::newI2CCmd) {
            I2CSlave::newI2CCmd = false;
            ControlMode newMode = (ControlMode)I2CSlave::cmdMode;

            if (newMode == VELOCITY) {
                float vel = I2CSlave::cmdTargetVelTenths * 0.1f;
                g_targetVel[0] = vel;
                if (g_mode[0] != VELOCITY) g_velPid[0].reset();
            } else if (newMode == POSITION) {
                float pos = I2CSlave::cmdTargetPosTenths * 0.1f;
                g_targetPos[0] = pos;
                g_velPid[0].reset();
                g_posPid[0].reset();
                g_commandedPos[0] = (float)g_measPosAbs[0];
                g_mode[0] = POSITION;
            }
            if (newMode != g_mode[0]) {
                g_velPid[0].reset();
                g_posPid[0].reset();
                g_mode[0] = newMode;
            }
        }

        // --- Reset PIDs on mode transition ----------------------------
        uint8_t curMode = g_mode[m];
        if (curMode != prevMode[m]) {
            g_velPid[m].reset();
            g_posPid[m].reset();
            if (curMode == VELOCITY)      g_commandedVel[m] = meas;
            else if (curMode == POSITION) g_commandedPos[m] = (float)g_measPosAbs[m];
        }
        prevMode[m] = curMode;

        // --- PID update -----------------------------------------------
        float logSetpoint = 0.0f;
        float logError    = 0.0f;
        float pidOut      = 0.0f;

        if (curMode == VELOCITY) {
            float cmdVel    = g_commandedVel[m];
            float tgtVel    = g_targetVel[m];
            float remaining = tgtVel - cmdVel;
            float step      = g_velAccel[m] * dt;
            if (fabsf(remaining) <= step) cmdVel = tgtVel;
            else cmdVel += (remaining > 0.0f) ? step : -step;
            g_commandedVel[m] = cmdVel;

            g_velPid[m].setSetpoint(cmdVel);
            pidOut      = g_velPid[m].update(meas, dt);
            logSetpoint = cmdVel;
            logError    = cmdVel - meas;

        } else if (curMode == POSITION) {
            // All position arithmetic is in absolute multi-turn degree space.
            // g_measPosAbs accumulates raw encoder deltas since the last zero,
            // so targets like 940° (≈2.6 revolutions) work directly with no
            // wraparound correction needed.
            float curAbs    = (float)g_measPosAbs[m];
            float cmdPos    = g_commandedPos[m];
            float tgtPos    = g_targetPos[m];
            float remaining = tgtPos - cmdPos;

            // Linear ramp toward target in absolute space
            float step = (float)g_traverseVel[m] * dt;
            if (fabsf(remaining) <= step) {
                cmdPos = tgtPos;
            } else {
                cmdPos += (remaining > 0.0f) ? step : -step;
            }
            g_commandedPos[m] = cmdPos;

            float posErr = cmdPos - curAbs;
            g_posError[m] = posErr;

            g_posPid[m].setSetpoint(cmdPos);
            float velCmd = g_posPid[m].update(curAbs, dt);
            g_targetVel[m] = velCmd;

            g_velPid[m].setSetpoint(velCmd);
            pidOut      = g_velPid[m].update(meas, dt);
            logSetpoint = cmdPos;
            logError    = posErr;
        }

        g_pidOutput[m] = pidOut;

        if (curMode == VELOCITY || curMode == POSITION) {
            int16_t duty = (int16_t)(pidOut * ((float)PWM_WRAP / 127.0f));
            MotorPWM::setMotorDuty(m, duty);
        }

        // Data logger — motor 0 only (single-channel logger)
        if (m == 0) {
            DataLogger::record(now,
                               logSetpoint, meas, logError,
                               pidOut, rawAngle, MotorPWM::rawDuty[0]);
        }
    }

    // --- RC servo ramp (both servos, same 50 Hz tick) ---
    for (uint8_t s = 0; s < NUM_SERVOS; s++) {
        float target = g_servoTarget[s];
        float actual = g_servoActual[s];
        float diff   = target - actual;
        float step   = g_servoRampRate[s] * dt;
        if (fabsf(diff) <= step) actual = target;
        else                     actual += (diff > 0.0f) ? step : -step;
        actual = constrain(actual, 0.0f, 180.0f);
        g_servoActual[s] = actual;
        g_servo[s].write((int)roundf(actual));
    }
}

// ---------------------------------------------------------------------------
// Console command helpers
// ---------------------------------------------------------------------------

// Print an error for a missing or out-of-range motor index.
static void printMotorIdxError(const String& cmd) {
    Serial.print("[ERR] "); Serial.print(cmd);
    Serial.print(": missing or invalid motor index (0–");
    Serial.print(NUM_MOTORS - 1); Serial.println(")");
}

// Parse "idx rest..." from a trimmed argument string.
// Returns false and prints an error if the index is missing or out of range.
static bool parseMotorIdx(const String& cmdName, const String& args,
                          uint8_t& outIdx, String& outRest) {
    String a = args;
    a.trim();
    if (a.length() == 0) { printMotorIdxError(cmdName); return false; }

    int sp = a.indexOf(' ');
    if (sp < 0) {
        outIdx  = (uint8_t)a.toInt();
        outRest = "";
    } else {
        outIdx  = (uint8_t)a.toInt();
        outRest = a.substring(sp + 1);
        outRest.trim();
    }
    if (outIdx >= NUM_MOTORS) { printMotorIdxError(cmdName); return false; }
    return true;
}

// Print gains for a single motor.
static void printGains(uint8_t m) {
    Serial.print("[M"); Serial.print(m);
    Serial.print(" VEL] kp="); Serial.print(g_velPid[m].kp, 4);
    Serial.print(" ki=");      Serial.print(g_velPid[m].ki, 4);
    Serial.print(" kd=");      Serial.print(g_velPid[m].kd, 4);
    Serial.print("  accel=");  Serial.print((float)g_velAccel[m], 1); Serial.println(" deg/s^2");

    Serial.print("[M"); Serial.print(m);
    Serial.print(" POS] kp="); Serial.print(g_posPid[m].kp, 4);
    Serial.print(" ki=");      Serial.print(g_posPid[m].ki, 4);
    Serial.print(" kd=");      Serial.print(g_posPid[m].kd, 4);
    Serial.print(" velLim=±"); Serial.print(POS_VEL_LIMIT, 1);
    Serial.print("  traverseVel="); Serial.print((float)g_traverseVel[m], 1); Serial.println(" deg/s");

    Serial.print("[M"); Serial.print(m);
    Serial.print(" POS] pathMode=");
    if (g_posPathMode[m] == POS_PATH_SHORTEST) {
        Serial.println("SHORTEST");
    } else {
        Serial.print("CONSTRAINED [");
        Serial.print((float)g_posMin[m], 1); Serial.print(", ");
        Serial.print((float)g_posMax[m], 1); Serial.println("] deg");
    }

    Serial.print("[M"); Serial.print(m);
    Serial.print(" CAL] zeroOffset="); Serial.print((float)g_zeroOffset[m], 2); Serial.println(" deg");
}

// ---------------------------------------------------------------------------
// Console command processor
// ---------------------------------------------------------------------------
static void processCommand(const String& raw) {
    String cmd = raw;
    cmd.trim();
    if (cmd.length() == 0) return;

    uint8_t m;
    String  rest;

    // ---- Data logger commands (no motor index) -----------------------
    if (cmd.equalsIgnoreCase("freeze")) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            g_mode[i] = MANUAL;
            g_velPid[i].reset();
            g_posPid[i].reset();
            MotorPWM::setMotor(i, MCP_SPEED_IDLE);
        }
        DataLogger::freeze();
        Serial.print("# Frozen — ");
        Serial.print(DataLogger::count());
        Serial.print(" records (");
        Serial.print(DataLogger::count() / 50.0f, 1);
        Serial.println("s)\n");
        DataLogger::dump();
        return;

    } else if (cmd.equalsIgnoreCase("resume")) {
        DataLogger::resume();
        Serial.println("[LOG] Recording resumed");
        return;

    // ---- NVM commands (no motor index) -------------------------------
    } else if (cmd.equalsIgnoreCase("save")) {
        captureSettings();
        Settings::save();
        Serial.println("[NVM] All motor settings saved to flash");
        return;

    } else if (cmd.equalsIgnoreCase("load")) {
        if (Settings::load()) {
            applySettings();
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                g_velPid[i].reset();
                g_posPid[i].reset();
            }
            Serial.println("[NVM] Settings loaded from flash (applied to all motors)");
        } else {
            Serial.println("[NVM] No valid settings in flash — use 'save' to store current values");
        }
        return;

    } else if (cmd.equalsIgnoreCase("defaults")) {
        Settings::defaults();
        applySettings();
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            g_velPid[i].reset();
            g_posPid[i].reset();
        }
        Serial.println("[NVM] Factory defaults applied to all motors (RAM only — type 'save' to persist)");
        return;

    // ---- gains (optional motor index) --------------------------------
    } else if (cmd.equalsIgnoreCase("gains")) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) printGains(i);
        Serial.println("[NVM] (RAM values — type 'save' to persist, 'load' to revert)");
        return;

    } else if (cmd.startsWith("gains ") || cmd.startsWith("GAINS ")) {
        if (!parseMotorIdx("gains", cmd.substring(6), m, rest)) return;
        printGains(m);
        return;

    // ---- stop (optional motor index: omit = all) ---------------------
    } else if (cmd.equalsIgnoreCase("stop")) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            g_mode[i] = MANUAL;
            g_velPid[i].reset();
            g_posPid[i].reset();
            MotorPWM::setMotor(i, MCP_SPEED_IDLE);
        }
        I2CSlave::lastCmdMs = millis();
        Serial.println("[CMD] All motors stopped");
        return;

    } else if (cmd.startsWith("stop ") || cmd.startsWith("STOP ")) {
        if (!parseMotorIdx("stop", cmd.substring(5), m, rest)) return;
        g_mode[m] = MANUAL;
        g_velPid[m].reset();
        g_posPid[m].reset();
        MotorPWM::setMotor(m, MCP_SPEED_IDLE);
        if (m == 0) I2CSlave::lastCmdMs = millis();
        Serial.print("[CMD] M"); Serial.print(m); Serial.println(" stopped");
        return;

    // ---- velocity ----------------------------------------------------
    } else if (cmd.startsWith("va ") || cmd.startsWith("VA ")) {
        if (!parseMotorIdx("va", cmd.substring(3), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] va: missing accel value"); return; }
        float accel = max(1.0f, rest.toFloat());
        g_velAccel[m] = accel;
        Serial.print("[M"); Serial.print(m);
        Serial.print(" CMD] Velocity accel "); Serial.print(accel, 1); Serial.println(" deg/s^2");
        return;

    } else if (cmd.startsWith("v ") || cmd.startsWith("V ") || cmd.startsWith("vel ")) {
        uint8_t pfx = cmd.startsWith("vel") ? 4 : 2;
        if (!parseMotorIdx("v", cmd.substring(pfx), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] v: missing velocity value"); return; }
        float vel = rest.toFloat();
        g_targetVel[m] = vel;
        if (g_mode[m] != VELOCITY) {
            g_velPid[m].reset();
            g_posPid[m].reset();
            g_mode[m] = VELOCITY;
        }
        if (m == 0) {
            I2CSlave::cmdMode            = MCP_MODE_VELOCITY;
            I2CSlave::cmdTargetVelTenths = (int16_t)(vel * 10.0f);
            I2CSlave::lastCmdMs          = millis();
        }
        Serial.print("[M"); Serial.print(m);
        Serial.print(" CMD] Velocity "); Serial.print(vel, 1); Serial.println(" deg/s");
        return;

    // ---- position ----------------------------------------------------
    } else if (cmd.startsWith("pv ") || cmd.startsWith("PV ")) {
        if (!parseMotorIdx("pv", cmd.substring(3), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] pv: missing velocity value"); return; }
        float vel = max(1.0f, rest.toFloat());
        g_traverseVel[m] = vel;
        Serial.print("[M"); Serial.print(m);
        Serial.print(" CMD] Traverse velocity "); Serial.print(vel, 1); Serial.println(" deg/s");
        return;

    } else if (cmd.startsWith("pmode") || cmd.startsWith("PMODE")) {
        if (!parseMotorIdx("pmode", cmd.substring(5), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] pmode: missing 0 or 1"); return; }
        g_posPathMode[m] = ((int)rest.toFloat() == 0) ? POS_PATH_SHORTEST : POS_PATH_CONSTRAINED;
        Serial.print("[M"); Serial.print(m); Serial.print(" POS] Path mode: ");
        if (g_posPathMode[m] == POS_PATH_SHORTEST) {
            Serial.println("SHORTEST (wrap-around allowed)");
        } else {
            Serial.print("CONSTRAINED [");
            Serial.print((float)g_posMin[m], 1); Serial.print(", ");
            Serial.print((float)g_posMax[m], 1); Serial.println("] deg");
        }
        return;

    } else if (cmd.startsWith("plim") || cmd.startsWith("PLIM")) {
        if (!parseMotorIdx("plim", cmd.substring(4), m, rest)) return;
        int sp = rest.indexOf(' ');
        if (sp <= 0) { Serial.println("[ERR] Usage: plim <m> <min> <max>"); return; }
        float lo = rest.toFloat();
        float hi = rest.substring(sp + 1).toFloat();
        if (hi <= lo) { Serial.println("[ERR] plim: max must be > min"); return; }
        g_posMin[m] = lo;
        g_posMax[m] = hi;
        Serial.print("[M"); Serial.print(m); Serial.print(" POS] Limits set: [");
        Serial.print(lo, 1); Serial.print(", "); Serial.print(hi, 1); Serial.println("] deg");
        return;

    } else if (cmd.startsWith("p ") || cmd.startsWith("P ")) {
        if (!parseMotorIdx("p", cmd.substring(2), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] p: missing position value"); return; }

        int sp = rest.indexOf(' ');
        float pos = rest.toFloat();

        // Targets are absolute multi-turn degrees — 940°, -180°, etc. are all valid.
        // CONSTRAINED mode clamps to the configured absolute travel limits.
        if (g_posPathMode[m] == POS_PATH_CONSTRAINED) {
            float lo = g_posMin[m], hi = g_posMax[m];
            if      (pos < lo) { pos = lo; Serial.print("[WARN] Position clamped to "); Serial.print(lo, 1); Serial.println(" deg"); }
            else if (pos > hi) { pos = hi; Serial.print("[WARN] Position clamped to "); Serial.print(hi, 1); Serial.println(" deg"); }
        }
        g_targetPos[m] = pos;

        if (sp > 0) {
            float vel = rest.substring(sp + 1).toFloat();
            if (vel >= 1.0f) g_traverseVel[m] = vel;
        }

        g_velPid[m].reset();
        g_posPid[m].reset();
        g_commandedPos[m] = (float)g_measPosAbs[m];
        g_mode[m] = POSITION;

        if (m == 0) {
            I2CSlave::cmdMode            = MCP_MODE_POSITION;
            I2CSlave::cmdTargetPosTenths = (uint16_t)(pos * 10.0f);
            I2CSlave::lastCmdMs          = millis();
        }
        Serial.print("[M"); Serial.print(m);
        Serial.print(" CMD] Position "); Serial.print(pos, 1);
        Serial.print(" deg  @ "); Serial.print((float)g_traverseVel[m], 1); Serial.println(" deg/s");
        return;

    // ---- raw PWM -----------------------------------------------------
    } else if (cmd.startsWith("r ") || cmd.startsWith("R ") || cmd.startsWith("raw ")) {
        uint8_t pfx = cmd.startsWith("raw") ? 4 : 2;
        if (!parseMotorIdx("r", cmd.substring(pfx), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] r: missing PWM value (0–255)"); return; }
        int val = constrain((int)rest.toFloat(), 0, 255);
        g_mode[m] = MANUAL;
        g_velPid[m].reset();
        g_posPid[m].reset();
        MotorPWM::setMotor(m, (uint8_t)val);
        if (m == 0) I2CSlave::lastCmdMs = millis();
        Serial.print("[M"); Serial.print(m);
        Serial.print(" CMD] Raw "); Serial.println(val);
        return;

    // ---- calibration -------------------------------------------------
    } else if (cmd.startsWith("zero") || cmd.startsWith("ZERO")) {
        if (!parseMotorIdx("zero", cmd.substring(4), m, rest)) return;
        float rawDeg = g_lastRawAngle[m] * (360.0f / 4096.0f);
        g_zeroOffset[m]   = rawDeg;   // keeps the modular 0–360 display correct
        g_measPosAbs[m]   = 0.0f;     // reset multi-turn accumulator to zero
        g_commandedPos[m] = 0.0f;     // prevent a jump if already in position mode
        g_targetPos[m]    = 0.0f;
        Serial.print("[M"); Serial.print(m);
        Serial.println(" CAL] Zero set — absolute position reset to 0 (RAM only — type 'save' to persist)");
        return;

    // ---- PID gain commands -------------------------------------------
    } else if (cmd.startsWith("pkp") || cmd.startsWith("Pkp") || cmd.startsWith("PKP")) {
        if (!parseMotorIdx("pkp", cmd.substring(3), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] pkp: missing value"); return; }
        g_posPid[m].kp = rest.toFloat();
        Serial.print("[M"); Serial.print(m); Serial.print(" POS] kp="); Serial.println(g_posPid[m].kp, 4);
        return;

    } else if (cmd.startsWith("pki") || cmd.startsWith("Pki") || cmd.startsWith("PKI")) {
        if (!parseMotorIdx("pki", cmd.substring(3), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] pki: missing value"); return; }
        g_posPid[m].ki = rest.toFloat();
        g_posPid[m].reset();
        Serial.print("[M"); Serial.print(m); Serial.print(" POS] ki="); Serial.println(g_posPid[m].ki, 4);
        return;

    } else if (cmd.startsWith("pkd") || cmd.startsWith("Pkd") || cmd.startsWith("PKD")) {
        if (!parseMotorIdx("pkd", cmd.substring(3), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] pkd: missing value"); return; }
        g_posPid[m].kd = rest.toFloat();
        Serial.print("[M"); Serial.print(m); Serial.print(" POS] kd="); Serial.println(g_posPid[m].kd, 4);
        return;

    } else if (cmd.startsWith("kp") || cmd.startsWith("Kp") || cmd.startsWith("KP")) {
        if (!parseMotorIdx("kp", cmd.substring(2), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] kp: missing value"); return; }
        g_velPid[m].kp = rest.toFloat();
        Serial.print("[M"); Serial.print(m); Serial.print(" VEL] kp="); Serial.println(g_velPid[m].kp, 4);
        return;

    } else if (cmd.startsWith("ki") || cmd.startsWith("Ki") || cmd.startsWith("KI")) {
        if (!parseMotorIdx("ki", cmd.substring(2), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] ki: missing value"); return; }
        g_velPid[m].ki = rest.toFloat();
        g_velPid[m].reset();
        Serial.print("[M"); Serial.print(m); Serial.print(" VEL] ki="); Serial.println(g_velPid[m].ki, 4);
        return;

    } else if (cmd.startsWith("kd") || cmd.startsWith("Kd") || cmd.startsWith("KD")) {
        if (!parseMotorIdx("kd", cmd.substring(2), m, rest)) return;
        if (rest.length() == 0) { Serial.println("[ERR] kd: missing value"); return; }
        g_velPid[m].kd = rest.toFloat();
        Serial.print("[M"); Serial.print(m); Serial.print(" VEL] kd="); Serial.println(g_velPid[m].kd, 4);
        return;

    // ---- RC servo ramp rate ------------------------------------------
    } else if (cmd.startsWith("servor ") || cmd.startsWith("SERVOR ")) {
        String args = cmd.substring(7);
        args.trim();
        int sp = args.indexOf(' ');
        if (sp <= 0) { Serial.println("[ERR] Usage: servor <idx> <rate deg/s>"); return; }
        uint8_t idx  = (uint8_t)args.toInt();
        float   rate = args.substring(sp + 1).toFloat();
        if (idx >= NUM_SERVOS) {
            Serial.print("[ERR] servor: invalid index (0–"); Serial.print(NUM_SERVOS - 1); Serial.println(")");
            return;
        }
        g_servoRampRate[idx] = max(1.0f, rate);
        Serial.print("[SERVO"); Serial.print(idx); Serial.print("] ramp rate ");
        Serial.print(g_servoRampRate[idx], 1); Serial.println(" deg/s");
        return;

    // ---- RC servo ----------------------------------------------------
    } else if (cmd.startsWith("servo ") || cmd.startsWith("SERVO ")) {
        String args = cmd.substring(6);
        args.trim();
        int sp = args.indexOf(' ');
        if (sp <= 0) { Serial.println("[ERR] Usage: servo <idx> <angle 0-180>"); return; }
        uint8_t idx   = (uint8_t)args.toInt();
        float   angle = args.substring(sp + 1).toFloat();
        if (idx >= NUM_SERVOS) {
            Serial.print("[ERR] servo: invalid index (0–"); Serial.print(NUM_SERVOS - 1); Serial.println(")");
            return;
        }
        int deg = constrain((int)angle, 0, 180);
        g_servoTarget[idx] = (float)deg;
        Serial.print("[SERVO"); Serial.print(idx); Serial.print("] target "); Serial.print(deg);
        Serial.print(" deg  ramp "); Serial.print(g_servoRampRate[idx], 1); Serial.println(" deg/s");
        return;

    // ---- Unknown command ---------------------------------------------
    } else {
        Serial.print("[ERR] Unknown: "); Serial.println(cmd);
        Serial.println("[ERR] Motor cmds (need index m): v va p pv pmode plim r stop gains kp ki kd pkp pki pkd zero");
        Serial.println("[ERR] Global cmds: freeze resume save load defaults");
        Serial.println("[ERR] Servo cmds: servo <idx> <angle 0-180>");
    }
}

// ---------------------------------------------------------------------------
// Console input handler — assembles characters into lines.
// ---------------------------------------------------------------------------
static String  g_serialBuf;

static void handleConsoleInput() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            processCommand(g_serialBuf);
            g_serialBuf = "";
        } else if (g_serialBuf.length() < 32) {
            g_serialBuf += c;
        }
    }
}

// ---------------------------------------------------------------------------
// Core 0 — main loop: control only, no Serial output.
// ---------------------------------------------------------------------------
void loop() {
    uint32_t now = millis();
    handleConsoleInput();
    controlLoop(now);
}
