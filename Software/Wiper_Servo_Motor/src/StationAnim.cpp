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
//  Public API
// =============================================================================

bool stationAnim_hasHandler(uint8_t nodeId)
{
    switch (nodeId) {
    case STATION_METER_ID:
        return true;
    default:
        return false;
    }
}

uint8_t stationAnim_defaultShowIntensity(uint8_t nodeId)
{
    switch (nodeId) {
    case STATION_METER_ID:
        return METER_DEFAULT_SHOW_INTENSITY;
    default:
        return 100;
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
    case STATION_METER_ID:
        updateMeter(nowMs, showIntensity, posMin, posMax,
                    outMode, outTargetPos, outTargetVel, outServoTarget);
        break;
    default:
        break;
    }
}
