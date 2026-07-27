#include "motor_can.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    MotorCanFrame frame;
    MotorFeedback feedback = {0};
    MotorRegisterValue register_value = {0};

    motor_build_torque(2U, -25, &frame);
    assert(frame.id == 0x602U);
    assert(frame.data[0] == 0x2bU);
    assert(frame.data[1] == 0x00U && frame.data[2] == 0x20U);
    assert(frame.data[4] == 0xffU && frame.data[5] == 0xe7U);

    frame.id = 0x581U;
    frame.dlc = 8U;
    frame.data[0] = 0x2aU;
    frame.data[1] = 0xffU;
    frame.data[2] = 0xffU;
    frame.data[3] = 0x9cU;
    frame.data[4] = 0xffU;
    frame.data[5] = 0x9cU;
    frame.data[6] = 0x00U;
    frame.data[7] = 0x7bU;
    assert(motor_parse_feedback(&frame, &feedback) == 0);
    assert(feedback.motor_id == 1U);
    assert(feedback.position_x100_deg == -100);
    assert(feedback.speed_rpm == -100);
    assert(feedback.current_x100_a == 123);
    assert(feedback.valid == 1U);

    motor_build_read_u32(2U, 0x000cU, &frame);
    assert(frame.id == 0x602U);
    assert(frame.data[0] == 0x43U);
    assert(frame.data[1] == 0x00U && frame.data[2] == 0x0cU);

    frame.id = 0x582U;
    frame.data[0] = 0x43U;
    frame.data[1] = 0x00U;
    frame.data[2] = 0x0cU;
    frame.data[4] = 0x01U;
    frame.data[5] = 0x00U;
    frame.data[6] = 0x08U;
    frame.data[7] = 0x00U;
    assert(motor_parse_register_u32(&frame, &register_value) == 0);
    assert(register_value.motor_id == 2U);
    assert(register_value.address == 0x000cU);
    assert(register_value.value == 0x01000800U);
    assert(register_value.valid == 1U);

    frame.data[1] = 0x00U;
    frame.data[2] = 0x06U;
    frame.data[4] = 0xffU;
    frame.data[5] = 0xffU;
    frame.data[6] = 0xd8U;
    frame.data[7] = 0xf0U;
    assert(motor_parse_register_u32(&frame, &register_value) == 0);
    assert(register_value.address == 0x0006U);
    assert((int32_t)register_value.value == -10000);

    puts("motor_balance_protocol_test: PASS");
    return 0;
}
