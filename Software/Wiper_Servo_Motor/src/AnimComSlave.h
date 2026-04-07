#pragma once
// =============================================================================
//  AnimComSlave.h -- AnimCom RS485 slave receiver (RP2040, Quad Motor Controller)
// =============================================================================

#include <Arduino.h>
#include "Config.h"
#include "AnimComProtocol.h"

struct AnimComCallbacks {
    void (*onControlState)(uint8_t state, uint8_t pattern, uint8_t speed_scale);
    void (*onManualSingle)(uint8_t ch, uint8_t cmd_type, int32_t value);
    void (*onTriggerEffect)(uint8_t effect_type, uint8_t effect_id, uint16_t effect_param);
    void (*onWatchdog)();
};

class AnimComSlave {
public:
    void begin(const AnimComCallbacks& cb);
    void poll();

    // Update the station ID checked by the frame parser at runtime.
    // Call after begin() whenever the NVM nodeId is loaded or changed.
    void setNodeId(uint8_t id) { _nodeId = id; }
    uint8_t getNodeId() const  { return _nodeId; }

    bool isLinked() const { return (millis() - _lastFrameMs) < LINK_TIMEOUT_MS; }
    uint32_t getRxFrames() const { return _statRxFrames; }
    uint32_t getCrcErrors() const { return _statCrcErrors; }
    uint32_t getGapCount() const { return _statGaps; }
    void logStats() const;

private:
    static constexpr uint32_t LINK_TIMEOUT_MS = 3000;

    enum ParseState : uint8_t {
        PS_SYNC0, PS_SYNC1, PS_STATION_ID, PS_SEQ,
        PS_MSG_TYPE, PS_PAYLOAD_LEN, PS_PAYLOAD, PS_CRC_HI, PS_CRC_LO
    };

    uint8_t          _nodeId = NODE_ID;
    AnimComCallbacks _cb = {};
    ParseState _ps = PS_SYNC0;
    AnimComFrame _frame = {};
    uint8_t _payloadPos = 0;
    uint8_t _crcHi = 0;
    uint8_t _lastSeq = 0;
    bool _firstFrame = true;
    bool _watchdogFired = false;

    uint32_t _lastFrameMs = 0;
    uint32_t _statRxFrames = 0;
    uint32_t _statCrcErrors = 0;
    uint32_t _statGaps = 0;

    void _processByte(uint8_t b);
    void _dispatchFrame();
};
