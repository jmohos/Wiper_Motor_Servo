#pragma once
// =============================================================================
//  StationAnim.h — Station-specific RUN_AUTO animation patterns
//
//  Each physical installation of the Quad Motor Controller has a unique role
//  in the animated show, identified by its RS485 station ID (configured via
//  NVM and the "nodeid" console command).
//
//  When the animation controller sends CONTROL_STATE RUN_AUTO, the control
//  loop calls stationAnim_hasHandler() to check whether the configured nodeId
//  has a dedicated handler.  If so, stationAnim_update() is called in place of
//  the generic sine-wave patterns, and the results are applied to the motor and
//  servo targets.
//
//  Station registry (mirrors Animation Controller endpoint table):
//    Station 7  — Meter: large analog gauge, 0-180 deg needle deflection
//
//  How to add a new station:
//    1. Add a case to stationAnim_hasHandler() returning true for the new ID.
//    2. Add the default showIntensity to stationAnim_defaultShowIntensity().
//    3. Add a static handler function (e.g. updateMyStation()) and dispatch it
//       inside stationAnim_update().
//    4. Document the new station in README.md.
// =============================================================================
#include <Arduino.h>
#include "Config.h"

// Motor mode constants — must match ControlMode enum values in main.cpp.
static constexpr uint8_t SANIM_MANUAL   = 0;
static constexpr uint8_t SANIM_VELOCITY = 1;
static constexpr uint8_t SANIM_POSITION = 2;

// Returns true if nodeId has a dedicated station animation handler.
// When true, applyAnimRunPattern() delegates to stationAnim_update() instead
// of running the generic sine-wave patterns.
bool stationAnim_hasHandler(uint8_t nodeId);

// Default showIntensity (0-200) to use at power-up and after watchdog recovery
// when no animation controller is present on the bus.
// Returns 100 for any nodeId that has no dedicated handler.
uint8_t stationAnim_defaultShowIntensity(uint8_t nodeId);

// Station-specific animation update — called from the 50 Hz control loop when
// g_animState == ANIMCOM_STATE_RUN_AUTO and stationAnim_hasHandler() is true.
//
// Parameters:
//   nodeId         — configured RS485 station ID (from Settings::s.nodeId)
//   nowMs          — current millis() timestamp
//   showIntensity  — animation intensity scalar, 0-200 (percent)
//   posMin[]       — per-motor minimum position limit from NVM (NUM_MOTORS elements)
//   posMax[]       — per-motor maximum position limit from NVM (NUM_MOTORS elements)
//   outMode[]      — write SANIM_* mode for each motor (NUM_MOTORS elements);
//                    initialised to current mode before call — only write channels
//                    this station actually controls
//   outTargetPos[] — write position setpoint (deg) for each motor (NUM_MOTORS elements)
//   outTargetVel[] — write velocity setpoint (deg/s) for each motor (NUM_MOTORS elements)
//   outServoTarget[] — write servo target angle (deg) for each servo (NUM_SERVOS elements);
//                    initialised to current targets before call — only write if used
void stationAnim_update(uint8_t   nodeId,
                        uint32_t  nowMs,
                        uint8_t   showIntensity,
                        const float posMin[],
                        const float posMax[],
                        uint8_t   outMode[],
                        float     outTargetPos[],
                        float     outTargetVel[],
                        float     outServoTarget[]);
