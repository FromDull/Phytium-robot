#include "rpmsg_protocol.h"

uint8_t rpmsg_checksum(const uint8_t *data, size_t size)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < size; ++i) {
        sum = (uint8_t)(sum + data[i]);
    }
    return (uint8_t)(~sum + 1);
}

size_t rpmsg_encode(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t length,
                    uint8_t *out, size_t out_size)
{
    size_t frame_size = (size_t)length + 5;
    if (length > RPMSG_MAX_PAYLOAD || out_size < frame_size) {
        return 0;
    }

    out[0] = RPMSG_FRAME_MAGIC;
    out[1] = type;
    out[2] = seq;
    out[3] = length;

    for (uint8_t i = 0; i < length; ++i) {
        out[4 + i] = payload ? payload[i] : 0;
    }

    out[4 + length] = rpmsg_checksum(out, (size_t)length + 4);
    return frame_size;
}

bool rpmsg_decode(const uint8_t *data, size_t size, RpmsgFrame *frame)
{
    if (!data || !frame || size < 5 || data[0] != RPMSG_FRAME_MAGIC) {
        return false;
    }

    uint8_t length = data[3];
    if (length > RPMSG_MAX_PAYLOAD || size != (size_t)length + 5) {
        return false;
    }

    uint8_t checksum = rpmsg_checksum(data, (size_t)length + 4);
    if (checksum != data[4 + length]) {
        return false;
    }

    frame->type = data[1];
    frame->seq = data[2];
    frame->length = length;
    for (uint8_t i = 0; i < length; ++i) {
        frame->payload[i] = data[4 + i];
    }

    return true;
}

