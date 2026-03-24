#pragma once
// =============================================================================
//  PIDController.h — Generic discrete PID controller
//
//  Features:
//    - Derivative-on-measurement (no kick when setpoint changes)
//    - Integral clamping anti-windup (integral bounded to output limits)
//    - Output clamped to [outMin, outMax]
//
//  Usage:
//    PIDController pid(kp, ki, kd, -127.0f, 127.0f);
//    pid.setSetpoint(targetVelocity);
//    float out = pid.update(measuredVelocity, dt_seconds);
// =============================================================================
#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outMin, float outMax);

    // Compute next output.  measurement: current process value.  dt: seconds since last call.
    float update(float measurement, float dt);

    void  setSetpoint(float sp);
    float getSetpoint() const { return _setpoint; }

    // Update gains without resetting state (safe to call while running).
    void setGains(float kp, float ki, float kd);
    void setLimits(float outMin, float outMax);

    // Clear integrator and derivative history.  Call after setpoint step changes
    // or when switching into active control.
    void reset();

    float output() const { return _output; }

    // Public gains for easy console inspection / tuning.
    float kp, ki, kd;

private:
    float _setpoint;
    float _integral;
    float _prevMeasurement;
    float _output;
    float _outMin, _outMax;
    bool  _firstUpdate;
};
