#include "rpmsg_protocol.h"
#include <assert.h>
#include <stdio.h>

static void test_encode_decode(void)
{
    uint8_t payload[1] = {1};
    uint8_t buffer[32];
    RpmsgFrame frame;

    size_t size = rpmsg_encode(CMD_CAN_ENABLE, 7, payload, 1, buffer, sizeof(buffer));
    assert(size == 6);
    assert(rpmsg_decode(buffer, size, &frame));
    assert(frame.type == CMD_CAN_ENABLE);
    assert(frame.seq == 7);
    assert(frame.length == 1);
    assert(frame.payload[0] == 1);
}

static void test_bad_checksum(void)
{
    uint8_t payload[1] = {1};
    uint8_t buffer[32];
    RpmsgFrame frame;

    size_t size = rpmsg_encode(CMD_HEARTBEAT, 1, payload, 1, buffer, sizeof(buffer));
    buffer[size - 1] ^= 0x55;
    assert(!rpmsg_decode(buffer, size, &frame));
}

static void test_balance_gain_frame(void)
{
    uint8_t payload[16] = {0};
    uint8_t buffer[32];
    RpmsgFrame frame;
    size_t size = rpmsg_encode(CMD_BALANCE_SET_GAINS, 3, payload,
                               sizeof(payload), buffer, sizeof(buffer));

    assert(size == 21);
    assert(rpmsg_decode(buffer, size, &frame));
    assert(frame.type == CMD_BALANCE_SET_GAINS);
    assert(frame.length == sizeof(payload));
}

static void test_balance_speed_limit_frame(void)
{
    uint8_t payload[4] = {0, 18, 79, 128};
    uint8_t buffer[16];
    RpmsgFrame frame;
    size_t size = rpmsg_encode(CMD_BALANCE_SET_SPEED_LIMIT, 4, payload,
                               sizeof(payload), buffer, sizeof(buffer));

    assert(size == 9);
    assert(rpmsg_decode(buffer, size, &frame));
    assert(frame.type == CMD_BALANCE_SET_SPEED_LIMIT);
    assert(frame.length == sizeof(payload));
}

static void test_motor_speed_diag_frame(void)
{
    uint8_t payload[5] = {1, 0, 5, 3, 232};
    uint8_t buffer[16];
    RpmsgFrame frame;
    size_t size = rpmsg_encode(CMD_CAN_SPEED_DIAG, 5, payload,
                               sizeof(payload), buffer, sizeof(buffer));

    assert(size == 10);
    assert(rpmsg_decode(buffer, size, &frame));
    assert(frame.type == CMD_CAN_SPEED_DIAG);
    assert(frame.length == sizeof(payload));
}

static void test_balance_telemetry_frame(void)
{
    uint8_t buffer[16];
    RpmsgFrame frame;
    size_t size = rpmsg_encode(CMD_BALANCE_TELEMETRY, 6, NULL, 0,
                               buffer, sizeof(buffer));

    assert(size == 5);
    assert(rpmsg_decode(buffer, size, &frame));
    assert(frame.type == CMD_BALANCE_TELEMETRY);
    assert(frame.length == 0U);
    assert(BALANCE_TELEMETRY_VERSION == 2U);
    assert(BALANCE_TELEMETRY_PAYLOAD_SIZE == 60U);
}

static void test_balance_runtime_tuning_frames(void)
{
    const uint8_t commands[] = {
        CMD_BALANCE_SET_FILTER,
        CMD_BALANCE_SET_POSTURE_PRIORITY,
        CMD_BALANCE_SET_TORQUE_LIMIT
    };
    uint8_t payload[4] = {0, 0, 0, 0};
    uint8_t buffer[16];
    RpmsgFrame frame;

    for (size_t i = 0; i < sizeof(commands); ++i) {
        size_t size = rpmsg_encode(commands[i], (uint8_t)(7U + i), payload,
                                   sizeof(payload), buffer, sizeof(buffer));
        assert(size == 9U);
        assert(rpmsg_decode(buffer, size, &frame));
        assert(frame.type == commands[i]);
        assert(frame.length == sizeof(payload));
    }
}

static void test_balance_position_hold_frame(void)
{
    uint8_t payload[13] = {1U};
    uint8_t buffer[24];
    RpmsgFrame frame;
    size_t size = rpmsg_encode(CMD_BALANCE_SET_POSITION_HOLD, 10U, payload,
                               sizeof(payload), buffer, sizeof(buffer));

    assert(size == 18U);
    assert(rpmsg_decode(buffer, size, &frame));
    assert(frame.type == CMD_BALANCE_SET_POSITION_HOLD);
    assert(frame.length == sizeof(payload));
}

int main(void)
{
    test_encode_decode();
    test_bad_checksum();
    test_balance_gain_frame();
    test_balance_speed_limit_frame();
    test_motor_speed_diag_frame();
    test_balance_telemetry_frame();
    test_balance_runtime_tuning_frames();
    test_balance_position_hold_frame();
    printf("rpmsg protocol tests passed\n");
    return 0;
}

