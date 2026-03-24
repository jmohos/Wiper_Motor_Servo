#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float outMin, float outMax)
    : kp(kp), ki(ki), kd(kd),
      _setpoint(0.0f), _integral(0.0f), _prevMeasurement(0.0f),
      _output(0.0f), _outMin(outMin), _outMax(outMax), _firstUpdate(true)
{}

float PIDController::update(float measurement, float dt) {
    if (dt <= 0.0f) return _output;

    float error = _setpoint - measurement;

    // Proportional term
    float pTerm = kp * error;

    // Integral term with conditional anti-windup:
    // Only accumulate when doing so would not worsen an already-saturated output.
    // This prevents windup while the motor is running at a limit, and allows
    // the integrator to discharge naturally when the error reverses.
    bool saturatedHigh = (_output >= _outMax);
    bool saturatedLow  = (_output <= _outMin);
    bool wouldWorsen   = (error > 0.0f && saturatedHigh) ||
                         (error < 0.0f && saturatedLow);
    if (!wouldWorsen) {
        _integral += ki * error * dt;
        if      (_integral >  _outMax) _integral =  _outMax;
        else if (_integral <  _outMin) _integral =  _outMin;
    }

    // Derivative term — applied on measurement (not error) to avoid
    // derivative kick when the setpoint steps.
    float dTerm = 0.0f;
    if (!_firstUpdate) {
        dTerm = -kd * (measurement - _prevMeasurement) / dt;
    }
    _firstUpdate = false;
    _prevMeasurement = measurement;

    _output = pTerm + _integral + dTerm;
    if      (_output >  _outMax) _output =  _outMax;
    else if (_output <  _outMin) _output =  _outMin;

    return _output;
}

void PIDController::setSetpoint(float sp) {
    _setpoint = sp;
}

void PIDController::setGains(float kp_, float ki_, float kd_) {
    kp = kp_;  ki = ki_;  kd = kd_;
}

void PIDController::setLimits(float outMin, float outMax) {
    _outMin = outMin;  _outMax = outMax;
}

void PIDController::reset() {
    _integral        = 0.0f;
    _prevMeasurement = 0.0f;
    _output          = 0.0f;
    _firstUpdate     = true;
}
