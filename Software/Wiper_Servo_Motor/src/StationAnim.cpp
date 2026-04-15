#include "StationAnim.h"
#include <Arduino.h>

// =============================================================================
//  Station 7 — Meter
//
//  A large analog gauge whose needle is driven by Motor 0 in POSITION mode.
//
//  showIntensity interpretation:
//    0%   → 0 deg   (needle at bottom of scale)
//    100% → 180 deg (needle at top of scale)
//    Values above 100% are mapped linearly but clamped by posMax[0] from NVM.
//
//  Live oscillation:
//    A sinusoidal wobble of ±METER_OSCILLATION_AMP degrees at a period of
//    METER_OSCILLATION_PERIOD_MS ms is superimposed on the setpoint to give
//    the illusion of a live measurement with visible noise.  The final target
//    is clamped to [posMin[0], posMax[0]] so the needle never over-travels.
//
//  Standalone default (no animation controller):
//    showIntensity defaults to METER_DEFAULT_SHOW_INTENSITY (50%) so the needle
//    rests at 90 deg with oscillation active.
//
//  Motor 1 and both servos are not used by this station and are left unchanged.
// =============================================================================

static constexpr uint8_t  STATION_METER_ID             = 7;
static constexpr float    METER_DEG_PER_PERCENT         = 1.8f;    // 180 deg / 100%
static constexpr float    METER_OSCILLATION_AMP         = 15.0f;    // +/- degrees
static constexpr float    METER_OSCILLATION_PERIOD_MS   = 5000.0f; // 5 second period
static constexpr uint8_t  METER_DEFAULT_SHOW_INTENSITY  = 50;      // 50% = 90 deg default

static void updateMeter(uint32_t nowMs,
                        uint8_t  showIntensity,
                        const float posMin[],
                        const float posMax[],
                        uint8_t  outMode[],
                        float    outTargetPos[],
                        float    outTargetVel[],
                        float    outServoTarget[])
{
    // Centre setpoint from the commanded show intensity scalar.
    float setpoint = (float)showIntensity * METER_DEG_PER_PERCENT;

    // Slow oscillation superimposed to simulate a live measurement.
    float osc = METER_OSCILLATION_AMP *
                sinf(2.0f * PI * (float)nowMs / METER_OSCILLATION_PERIOD_MS);

    // Final position target, clamped to the motor's configured travel limits.
    float target = constrain(setpoint + osc, posMin[0], posMax[0]);

    // Motor 0 — POSITION mode, needle target.
    outMode[0]      = SANIM_POSITION;
    outTargetPos[0] = target;
    outTargetVel[0] = 0.0f;   // unused in POSITION mode

    // Motor 1 and servos — not controlled by this station; leave as-is.
    (void)outMode[1];
    (void)outTargetPos[1];
    (void)outTargetVel[1];
    (void)outServoTarget;
}

// =============================================================================
//  Station 6 — Robot
//
//  A robot prop whose head (Motor 0) and arms (Motor 1) perform a synchronised
//  dance move.  Both motors oscillate between -ROBOT_AMP_DEG and +ROBOT_AMP_DEG
//  in POSITION mode with a ROBOT_DWELL_MS hold at each extreme before reversing.
//  Servo 0 and Servo 1 mirror the same deflection, centred at 90°.
//
//  showIntensity scaling:
//    0%   → 0 deg amplitude  (all outputs frozen at zero / 90° servo centre)
//    100% → ±ROBOT_AMP_DEG  (full dance swing)
//    Values above 100% are clamped at ±ROBOT_AMP_DEG.
//
//  Standalone default (no animation controller):
//    showIntensity defaults to ROBOT_DEFAULT_SHOW_INTENSITY (100%) for full swing.
//
//  State machine (time-based, does not require position feedback):
//    Phase 0 — move to  +amp  (ROBOT_MOVE_MS)
//    Phase 1 — dwell at +amp  (ROBOT_DWELL_MS)
//    Phase 2 — move to  -amp  (ROBOT_MOVE_MS)
//    Phase 3 — dwell at -amp  (ROBOT_DWELL_MS)
// =============================================================================

static constexpr uint8_t  STATION_ROBOT_ID              = 6;
static constexpr float    ROBOT_AMP_DEG                 = 45.0f;   // fixed travel limits +/- degrees
static constexpr float    ROBOT_FULL_TRAVERSE_VEL       = 180.0f;  // traverse velocity at 100 %
static constexpr float    ROBOT_TRAVERSE_PER_PERCENT    = 1.8f;    // deg/s per intensity percent
static constexpr float    ROBOT_MIN_TRAVERSE_VEL        = 5.0f;    // floor to prevent div-by-zero
static constexpr uint32_t ROBOT_DWELL_MS                = 500;     // hold time at each extreme (unscaled)
static constexpr float    ROBOT_MOVE_SAFETY_FACTOR      = 1.5f;    // margin multiplier for move phase
static constexpr uint32_t ROBOT_MOVE_MARGIN_MS          = 300;     // flat margin added to move phase
static constexpr uint8_t  ROBOT_DEFAULT_SHOW_INTENSITY  = 100;     // 100 % = full speed, ±45 deg

static void updateRobot(uint32_t nowMs,
                        uint8_t  showIntensity,
                        const float posMin[],
                        const float posMax[],
                        uint8_t  outMode[],
                        float    outTargetPos[],
                        float    outTargetVel[],
                        float    outServoTarget[])
{
    // 4-phase state machine (persists across calls).
    //   Phase 0 — move to  +AMP  (duration adapts to traverse velocity)
    //   Phase 1 — dwell at +AMP  (ROBOT_DWELL_MS, unaffected by intensity)
    //   Phase 2 — move to  -AMP  (duration adapts to traverse velocity)
    //   Phase 3 — dwell at -AMP  (ROBOT_DWELL_MS, unaffected by intensity)
    static uint32_t phaseStartMs = 0;
    static uint8_t  phase        = 0;
    static uint32_t moveDurMs    = 1500;   // recalculated at start of each move phase

    // Traverse velocity scales with intensity; targets always reach full amplitude.
    float traverseVel = max(ROBOT_MIN_TRAVERSE_VEL,
                            (float)showIntensity * ROBOT_TRAVERSE_PER_PERCENT);

    // Advance phase when its duration has elapsed.
    uint32_t phaseDur = (phase & 1u) ? ROBOT_DWELL_MS : moveDurMs;
    if (nowMs - phaseStartMs >= phaseDur) {
        phase = (phase + 1u) & 3u;
        phaseStartMs = nowMs;
        if (!(phase & 1u)) {
            // Entering a new move phase — lock in duration for this leg.
            // Worst-case travel is the full 2×AMP crossing; safety factor
            // and flat margin ensure the motor arrives before the phase ends.
            moveDurMs = (uint32_t)(2.0f * ROBOT_AMP_DEG / traverseVel
                                   * 1000.0f * ROBOT_MOVE_SAFETY_FACTOR)
                        + ROBOT_MOVE_MARGIN_MS;
        }
    }

    // Targets are always the full limits — intensity only controls speed.
    float target = (phase < 2u) ? ROBOT_AMP_DEG : -ROBOT_AMP_DEG;

    // Motor 0 (head) — POSITION mode.
    // outTargetVel in POSITION mode is interpreted as traverse velocity by main.cpp.
    outMode[0]      = SANIM_POSITION;
    outTargetPos[0] = constrain(target, posMin[0], posMax[0]);
    outTargetVel[0] = traverseVel;

    // Motor 1 (arms) — identical motion.
    outMode[1]      = SANIM_POSITION;
    outTargetPos[1] = constrain(target, posMin[1], posMax[1]);
    outTargetVel[1] = traverseVel;

    // Servos 0 and 1 — same deflection centred at 90°.
    float servoAngle = constrain(90.0f + target, 0.0f, 180.0f);
    outServoTarget[0] = servoAngle;
    outServoTarget[1] = servoAngle;
}

// =============================================================================
//  Public API
// =============================================================================

bool stationAnim_hasHandler(uint8_t nodeId)
{
    switch (nodeId) {
    case STATION_ROBOT_ID:
    case STATION_METER_ID:
        return true;
    default:
        return false;
    }
}

uint8_t stationAnim_defaultShowIntensity(uint8_t nodeId)
{
    switch (nodeId) {
    case STATION_ROBOT_ID:  return ROBOT_DEFAULT_SHOW_INTENSITY;
    case STATION_METER_ID:  return METER_DEFAULT_SHOW_INTENSITY;
    default:                return 100;
    }
}

void stationAnim_update(uint8_t   nodeId,
                        uint32_t  nowMs,
                        uint8_t   showIntensity,
                        const float posMin[],
                        const float posMax[],
                        uint8_t   outMode[],
                        float     outTargetPos[],
                        float     outTargetVel[],
                        float     outServoTarget[])
{
    switch (nodeId) {
    case STATION_ROBOT_ID:
        updateRobot(nowMs, showIntensity, posMin, posMax,
                    outMode, outTargetPos, outTargetVel, outServoTarget);
        break;
    case STATION_METER_ID:
        updateMeter(nowMs, showIntensity, posMin, posMax,
                    outMode, outTargetPos, outTargetVel, outServoTarget);
        break;
    default:
        break;
    }
}
