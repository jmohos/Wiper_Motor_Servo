#pragma once
/*
 * AnimComProtocol.h -- shared AnimCom RS485 wire format
 *
 * Frame layout:
 *   byte 0   sync0        0x55
 *   byte 1   sync1        0xAA
 *   byte 2   station_id   0x00 master, 0x01-0xFE endpoint, 0xFF broadcast
 *   byte 3   seq          rolling sender sequence
 *   byte 4   msg_type     ANIMCOM_MSG_*
 *   byte 5   payload_len  0-32
 *   byte 6+  payload      message-specific bytes
 *   final 2  crc16        CRC-16/CCITT over bytes 2 through payload end
 *
 * Physical layer:
 *   RS485 half duplex, 115200 baud, 8N1, master to endpoints.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define ANIMCOM_SYNC0            0x55u
#define ANIMCOM_SYNC1            0xAAu
#define ANIMCOM_STATION_MASTER   0x00u
#define ANIMCOM_STATION_BCAST    0xFFu
#define ANIMCOM_MAX_PAYLOAD      32u
#define ANIMCOM_FRAME_OVERHEAD   8u
#define ANIMCOM_MAX_FRAME        (ANIMCOM_FRAME_OVERHEAD + ANIMCOM_MAX_PAYLOAD)

#define ANIMCOM_MSG_CONTROL_STATE   0x01u
#define ANIMCOM_MSG_TRIGGER_EFFECT  0x02u
#define ANIMCOM_MSG_MANUAL_SINGLE   0x03u
#define ANIMCOM_MSG_MANUAL_BULK     0x04u

/*
 * CONTROL_STATE payload (0x01) -- 3 bytes
 *   [0] state       0=STOP, 1=RUN_AUTO, 2=MANUAL
 *   [1] pattern     animation pattern number, used in RUN_AUTO
 *   [2] show_intensity 0-200 percent, used in RUN_AUTO
 */
#define ANIMCOM_STATE_STOP      0u
#define ANIMCOM_STATE_RUN_AUTO  1u
#define ANIMCOM_STATE_MANUAL    2u

typedef struct {
    uint8_t state;
    uint8_t pattern;
    uint8_t show_intensity;
} AnimComPayloadControlState;

/*
 * TRIGGER_EFFECT payload (0x02) -- 4 bytes
 *   [0] effect_type
 *   [1] effect_id
 *   [2] effect_param low byte
 *   [3] effect_param high byte
 */
#define ANIMCOM_EFFECT_AUDIO_PLAY   0x01u
#define ANIMCOM_EFFECT_MOTION_SPEC  0x02u

typedef struct {
    uint8_t  effect_type;
    uint8_t  effect_id;
    uint16_t effect_param;
} AnimComPayloadTriggerEffect;

/*
 * MANUAL_SINGLE payload (0x03) -- 6 bytes
 *   [0] channel_index
 *   [1] cmd_type
 *   [2..5] value int32 little-endian
 *
 * Capability table:
 *   cmd_type              Octal PWM   Quad DC   Quad Servo
 *   SPEED_PERCENT         yes         yes       yes
 *   SPEED_DEG_PER_SEC     ignore      yes       ignore
 *   POSITION_DEG          ignore      yes       yes
 */
#define ANIMCOM_CMD_SPEED_PERCENT      0x01u
#define ANIMCOM_CMD_SPEED_DEG_PER_SEC  0x02u
#define ANIMCOM_CMD_POSITION_DEG       0x03u

typedef struct {
    uint8_t channel_index;
    uint8_t cmd_type;
    int32_t value;
} AnimComPayloadManualSingle;

/*
 * MANUAL_BULK payload (0x04) -- 8 bytes
 *   [0..7] signed speed percent for octal motors
 */
#define ANIMCOM_BULK_MOTORS  8u

typedef struct {
    int8_t speeds[ANIMCOM_BULK_MOTORS];
} AnimComPayloadManualBulk;

typedef struct {
    uint8_t station_id;
    uint8_t seq;
    uint8_t msg_type;
    uint8_t payload_len;
    uint8_t payload[ANIMCOM_MAX_PAYLOAD];
} AnimComFrame;

static inline uint16_t animcom_crc16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= ((uint16_t)(*data++) << 8u);
        uint8_t i;
        for (i = 0u; i < 8u; i++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1u) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1u);
            }
        }
    }
    return crc;
}

static inline uint8_t animcom_build_frame(uint8_t *buf,
                                          uint8_t buf_size,
                                          const AnimComFrame *f)
{
    uint8_t total = (uint8_t)(ANIMCOM_FRAME_OVERHEAD + f->payload_len);
    if (buf_size < total) {
        return 0u;
    }

    buf[0] = ANIMCOM_SYNC0;
    buf[1] = ANIMCOM_SYNC1;
    buf[2] = f->station_id;
    buf[3] = f->seq;
    buf[4] = f->msg_type;
    buf[5] = f->payload_len;

    uint8_t i;
    for (i = 0u; i < f->payload_len; i++) {
        buf[6u + i] = f->payload[i];
    }

    {
        uint16_t crc = animcom_crc16(&buf[2], (uint8_t)(4u + f->payload_len));
        buf[6u + f->payload_len] = (uint8_t)(crc >> 8u);
        buf[7u + f->payload_len] = (uint8_t)(crc & 0xFFu);
    }
    return total;
}

static inline uint8_t animcom_pct_to_motor_byte(int8_t pct)
{
    if (pct > 100) {
        pct = 100;
    }
    if (pct < -100) {
        pct = -100;
    }
    if (pct == 0) {
        return 127u;
    }
    if (pct > 0) {
        return (uint8_t)(127 + (int16_t)pct * 128 / 100);
    }
    return (uint8_t)(127 + (int16_t)pct * 127 / 100);
}

static inline int8_t animcom_motor_byte_to_pct(uint8_t motorByte)
{
    int16_t delta = (int16_t)motorByte - 127;
    if (delta == 0) {
        return 0;
    }
    if (delta > 0) {
        return (int8_t)(delta * 100 / 128);
    }
    return (int8_t)(delta * 100 / 127);
}

#ifdef __cplusplus
}
#endif

