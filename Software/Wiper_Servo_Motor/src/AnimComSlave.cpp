#include "AnimComSlave.h"

#include <Arduino.h>

void AnimComSlave::begin(const AnimComCallbacks& cb)
{
    _cb = cb;

    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);

    Serial1.setTX(RS485_TX_PIN);
    Serial1.setRX(RS485_RX_PIN);
    Serial1.begin(RS485_BAUD);

    _lastFrameMs = millis();
    _watchdogFired = false;

    Serial.printf("[AnimCom] RS485 slave ready  station=0x%02X  baud=%lu\n",
                  _nodeId,
                  (unsigned long)RS485_BAUD);
}

void AnimComSlave::poll()
{
    while (Serial1.available()) {
        _processByte((uint8_t)Serial1.read());
    }

    if (!_watchdogFired && (millis() - _lastFrameMs) >= WATCHDOG_TIMEOUT_MS) {
        _watchdogFired = true;
        Serial.println("[AnimCom] Watchdog timeout -- coasting all motors");
        if (_cb.onWatchdog) {
            _cb.onWatchdog();
        }
    }
    if (_watchdogFired && isLinked()) {
        _watchdogFired = false;
    }
}

void AnimComSlave::logStats() const
{
    Serial.printf("[AnimCom] rx=%lu crcErr=%lu gaps=%lu linked=%s\n",
                  _statRxFrames,
                  _statCrcErrors,
                  _statGaps,
                  isLinked() ? "YES" : "NO");
}

void AnimComSlave::_processByte(uint8_t b)
{
    switch (_ps) {
    case PS_SYNC0:
        if (b == ANIMCOM_SYNC0) _ps = PS_SYNC1;
        break;

    case PS_SYNC1:
        if (b == ANIMCOM_SYNC1) _ps = PS_STATION_ID;
        else if (b == ANIMCOM_SYNC0) _ps = PS_SYNC1;
        else _ps = PS_SYNC0;
        break;

    case PS_STATION_ID:
        if (b == _nodeId || b == ANIMCOM_STATION_BCAST) {
            _frame.station_id = b;
            _ps = PS_SEQ;
        } else {
            _ps = PS_SYNC0;
        }
        break;

    case PS_SEQ:
        _frame.seq = b;
        _ps = PS_MSG_TYPE;
        break;

    case PS_MSG_TYPE:
        _frame.msg_type = b;
        _ps = PS_PAYLOAD_LEN;
        break;

    case PS_PAYLOAD_LEN:
        if (b > ANIMCOM_MAX_PAYLOAD) {
            _ps = PS_SYNC0;
            break;
        }
        _frame.payload_len = b;
        _payloadPos = 0;
        _ps = (b == 0) ? PS_CRC_HI : PS_PAYLOAD;
        break;

    case PS_PAYLOAD:
        _frame.payload[_payloadPos++] = b;
        if (_payloadPos >= _frame.payload_len) _ps = PS_CRC_HI;
        break;

    case PS_CRC_HI:
        _crcHi = b;
        _ps = PS_CRC_LO;
        break;

    case PS_CRC_LO: {
        uint8_t crcBuf[4 + ANIMCOM_MAX_PAYLOAD];
        crcBuf[0] = _frame.station_id;
        crcBuf[1] = _frame.seq;
        crcBuf[2] = _frame.msg_type;
        crcBuf[3] = _frame.payload_len;
        for (uint8_t i = 0; i < _frame.payload_len; i++) {
            crcBuf[4 + i] = _frame.payload[i];
        }

        uint16_t expected = animcom_crc16(crcBuf, (uint8_t)(4u + _frame.payload_len));
        uint16_t received = ((uint16_t)_crcHi << 8u) | b;

        if (received == expected) {
            if (!_firstFrame) {
                uint8_t gap = (uint8_t)(_frame.seq - _lastSeq - 1u);
                if (gap > 0) _statGaps += gap;
            }
            _lastSeq = _frame.seq;
            _firstFrame = false;
            _dispatchFrame();
        } else {
            _statCrcErrors++;
        }
        _ps = PS_SYNC0;
        break;
    }

    default:
        _ps = PS_SYNC0;
        break;
    }
}

void AnimComSlave::_dispatchFrame()
{
    _lastFrameMs = millis();
    _statRxFrames++;

    switch (_frame.msg_type) {
    case ANIMCOM_MSG_CONTROL_STATE:
        if (_frame.payload_len < 3) {
            break;
        }
        if (_cb.onControlState) {
            _cb.onControlState(_frame.payload[0], _frame.payload[1], _frame.payload[2]);
        }
        break;

    case ANIMCOM_MSG_TRIGGER_EFFECT:
        if (_frame.payload_len < 4) {
            break;
        }
        if (_cb.onTriggerEffect) {
            uint16_t param = (uint16_t)_frame.payload[2] |
                             ((uint16_t)_frame.payload[3] << 8u);
            _cb.onTriggerEffect(_frame.payload[0], _frame.payload[1], param);
        }
        break;

    case ANIMCOM_MSG_MANUAL_SINGLE: {
        if (_frame.payload_len < 6) {
            break;
        }
        uint8_t ch = _frame.payload[0];
        uint8_t cmd_type = _frame.payload[1];
        int32_t value = (int32_t)_frame.payload[2]
                      | ((int32_t)_frame.payload[3] << 8)
                      | ((int32_t)_frame.payload[4] << 16)
                      | ((int32_t)_frame.payload[5] << 24);
        if (_cb.onManualSingle) {
            _cb.onManualSingle(ch, cmd_type, value);
        }
        break;
    }

    case ANIMCOM_MSG_MANUAL_BULK:
        break;

    default:
        break;
    }
}

