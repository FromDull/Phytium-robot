#include "leg_joint_mapping.h"

#include <assert.h>
#include <stdio.h>

static void test_stable_pose_mapping(void)
{
    const uint16_t effective[LEG_JOINT_COUNT] = {450U, 450U, 450U, 450U};
    const uint16_t expected_raw[LEG_JOINT_COUNT] = {450U, 1350U, 450U, 1350U};
    uint16_t raw[LEG_JOINT_COUNT];
    uint16_t round_trip[LEG_JOINT_COUNT];

    assert(leg_effective_to_servo_raw_x10(effective, raw) == 0);
    for (uint8_t i = 0; i < LEG_JOINT_COUNT; ++i) {
        assert(raw[i] == expected_raw[i]);
    }
    assert(leg_servo_raw_to_effective_x10(raw, round_trip) == 0);
    for (uint8_t i = 0; i < LEG_JOINT_COUNT; ++i) {
        assert(round_trip[i] == effective[i]);
    }
}

static void test_asymmetric_mapping_and_validation(void)
{
    const uint16_t effective[LEG_JOINT_COUNT] = {300U, 400U, 500U, 600U};
    const uint16_t invalid[LEG_JOINT_COUNT] = {300U, 1801U, 500U, 600U};
    uint16_t raw[LEG_JOINT_COUNT];

    assert(leg_effective_to_servo_raw_x10(effective, raw) == 0);
    assert(raw[0] == 300U);
    assert(raw[1] == 1400U);
    assert(raw[2] == 600U);
    assert(raw[3] == 1300U);
    assert(leg_effective_to_servo_raw_x10(invalid, raw) != 0);
    assert(leg_effective_to_servo_raw_x10(NULL, raw) != 0);
}

int main(void)
{
    test_stable_pose_mapping();
    test_asymmetric_mapping_and_validation();
    puts("leg_joint_mapping_test: PASS");
    return 0;
}
