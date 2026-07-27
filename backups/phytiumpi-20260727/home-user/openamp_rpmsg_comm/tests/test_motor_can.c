#include "motor_can.h"

#include <assert.h>
#include <stdio.h>

static void test_pvt_frame(void)
{
    MotorCanFrame frame;
    motor_build_pvt(1, 36000, 200, 50, &frame);

    assert(frame.id == 0x601);
    assert(frame.dlc == 8);
    assert(frame.data[0] == 0x25);
    assert(frame.data[1] == 0x00);
    assert(frame.data[2] == 0x00);
    assert(frame.data[3] == 0x8C);
    assert(frame.data[4] == 0xA0);
    assert(frame.data[5] == 0x00);
    assert(frame.data[6] == 0xC8);
    assert(frame.data[7] == 50);
}

static void test_known_linux_cansend_frame(void)
{
    MotorCanFrame frame;
    motor_build_pvt(1, 500, 50, 10, &frame);

    assert(frame.id == 0x601);
    assert(frame.dlc == 8);
    assert(frame.data[0] == 0x25);
    assert(frame.data[1] == 0x00);
    assert(frame.data[2] == 0x00);
    assert(frame.data[3] == 0x01);
    assert(frame.data[4] == 0xF4);
    assert(frame.data[5] == 0x00);
    assert(frame.data[6] == 0x32);
    assert(frame.data[7] == 0x0A);
}

static void test_origin_frames(void)
{
    MotorCanFrame frame;

    motor_build_set_origin(3, &frame);
    assert(frame.id == 0x603);
    assert(frame.data[0] == 0x2B);
    assert(frame.data[1] == 0x00);
    assert(frame.data[2] == 0xA6);
    assert(frame.data[4] == 0x00);
    assert(frame.data[5] == 0x01);

    motor_build_set_temporary_origin(4, &frame);
    assert(frame.id == 0x604);
    assert(frame.data[0] == 0x2B);
    assert(frame.data[1] == 0x00);
    assert(frame.data[2] == 0xA7);
    assert(frame.data[4] == 0x00);
    assert(frame.data[5] == 0x01);
}

int main(void)
{
    test_pvt_frame();
    test_known_linux_cansend_frame();
    test_origin_frames();
    printf("motor CAN protocol tests passed\n");
    return 0;
}
