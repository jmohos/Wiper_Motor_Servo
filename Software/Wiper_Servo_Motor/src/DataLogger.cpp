#include "DataLogger.h"

// Static member storage
DataLogger::Record DataLogger::_buf[DataLogger::CAPACITY];
uint16_t DataLogger::_head   = 0;
uint16_t DataLogger::_count  = 0;
bool     DataLogger::_frozen = false;

void DataLogger::begin() {
    _head   = 0;
    _count  = 0;
    _frozen = false;
}

void DataLogger::record(uint32_t timeMs,
                        float setpoint, float velocity, float error,
                        float pidOutput, uint16_t angle, int16_t duty) {
    if (_frozen) return;

    Record& r   = _buf[_head];
    r.timeMs    = timeMs;
    r.setpoint  = setpoint;
    r.velocity  = velocity;
    r.error     = error;
    r.pidOutput = pidOutput;
    r.angle     = angle;
    r.duty      = duty;

    _head = (_head + 1) % CAPACITY;
    if (_count < CAPACITY) _count++;
}

void DataLogger::freeze() {
    _frozen = true;
}

void DataLogger::resume() {
    _head   = 0;
    _count  = 0;
    _frozen = false;
}

void DataLogger::dump() {
    Serial.println("time_ms,setpoint,velocity_dps,error,pid_out,angle_raw,angle_deg,duty");

    if (_count == 0) {
        Serial.println("# (no data)");
        return;
    }

    uint16_t start = (_head + CAPACITY - _count) % CAPACITY;

    char line[80];
    for (uint16_t i = 0; i < _count; i++) {
        const Record& r = _buf[(start + i) % CAPACITY];
        float angleDeg  = (float)r.angle * 360.0f / 4096.0f;

        snprintf(line, sizeof(line),
                 "%lu,%.2f,%.2f,%.2f,%.2f,%u,%.2f,%d",
                 (unsigned long)r.timeMs,
                 r.setpoint, r.velocity, r.error,
                 r.pidOutput, r.angle, angleDeg, (int)r.duty);
        Serial.println(line);
    }

    Serial.print("# ");
    Serial.print(_count);
    Serial.println(" records");
}
