#pragma once
// =============================================================================
//  DataLogger.h — Circular buffer data logger for PID tuning
//
//  Records one entry per control loop cycle (50 Hz).
//  Capacity: 500 records ≈ 10 seconds of history.
//
//  Workflow:
//    1. Logger runs continuously in the background (record() each cycle).
//    2. Type "freeze" in the console — recording stops and the buffer is
//       dumped to Serial as CSV.
//    3. Copy/paste from the terminal into a .csv file and open in Excel.
//    4. Type "resume" to clear the buffer and restart recording.
//
//  CSV columns:
//    time_ms, setpoint_dps, velocity_dps, error_dps, pid_out, angle_raw, angle_deg, pwm
// =============================================================================
#include <Arduino.h>

class DataLogger {
public:
    struct Record {
        uint32_t timeMs;      // millis() at sample time
        float    setpoint;    // velocity setpoint (deg/s)
        float    velocity;    // measured velocity  (deg/s)
        float    error;       // setpoint − velocity (deg/s)
        float    pidOutput;   // PID output (−127 … +127)
        uint16_t angle;       // AS5600 raw angle (0–4095)
        int16_t  duty;        // signed PWM duty (−PWM_WRAP … +PWM_WRAP)
    };

    // Number of records stored.  At 50 Hz: 500 = ~10 seconds.
    static constexpr uint16_t CAPACITY = 500;

    // Initialise (clears buffer, starts in RUNNING state).
    static void begin();

    // Push one record.  No-op when frozen.
    // error is passed explicitly so each mode can supply the meaningful error:
    //   VELOCITY mode : setpoint(deg/s) − velocity(deg/s)
    //   POSITION mode : target_angle(deg) − current_angle(deg)
    static void record(uint32_t timeMs,
                       float setpoint, float velocity, float error,
                       float pidOutput, uint16_t angle, int16_t duty);

    // Stop recording.  Call before dump().
    static void freeze();

    // Print all buffered records to Serial as CSV, oldest first.
    // Blocks until complete — only call when motor is safely coasted.
    static void dump();

    // Clear buffer and resume recording.
    static void resume();

    static bool isFrozen() { return _frozen; }
    static uint16_t count() { return _count; }

private:
    static Record   _buf[CAPACITY];
    static uint16_t _head;    // index of next write slot
    static uint16_t _count;   // valid records currently stored (0–CAPACITY)
    static bool     _frozen;
};
