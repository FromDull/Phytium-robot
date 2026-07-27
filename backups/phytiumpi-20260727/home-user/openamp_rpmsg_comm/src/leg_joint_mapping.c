#include "leg_joint_mapping.h"

#include <stddef.h>

static int angles_are_valid(const uint16_t angles[LEG_JOINT_COUNT])
{
    if (angles == NULL) {
        return 0;
    }
    for (uint8_t i = 0; i < LEG_JOINT_COUNT; ++i) {
        if (angles[i] > LEG_ANGLE_FULL_SCALE_X10) {
            return 0;
        }
    }
    return 1;
}

int leg_effective_to_servo_raw_x10(
    const uint16_t effective_x10[LEG_JOINT_COUNT],
    uint16_t servo_raw_x10[LEG_JOINT_COUNT])
{
    if (!angles_are_valid(effective_x10) || servo_raw_x10 == NULL) {
        return -1;
    }

    /* Servo order: right-front, right-rear, left-rear, left-front. */
    servo_raw_x10[0] = effective_x10[LEG_JOINT_RIGHT_FRONT];
    servo_raw_x10[1] = LEG_ANGLE_FULL_SCALE_X10 -
        effective_x10[LEG_JOINT_RIGHT_REAR];
    servo_raw_x10[2] = effective_x10[LEG_JOINT_LEFT_REAR];
    servo_raw_x10[3] = LEG_ANGLE_FULL_SCALE_X10 -
        effective_x10[LEG_JOINT_LEFT_FRONT];
    return 0;
}

int leg_servo_raw_to_effective_x10(
    const uint16_t servo_raw_x10[LEG_JOINT_COUNT],
    uint16_t effective_x10[LEG_JOINT_COUNT])
{
    if (!angles_are_valid(servo_raw_x10) || effective_x10 == NULL) {
        return -1;
    }

    effective_x10[LEG_JOINT_RIGHT_FRONT] = servo_raw_x10[0];
    effective_x10[LEG_JOINT_RIGHT_REAR] =
        LEG_ANGLE_FULL_SCALE_X10 - servo_raw_x10[1];
    effective_x10[LEG_JOINT_LEFT_REAR] = servo_raw_x10[2];
    effective_x10[LEG_JOINT_LEFT_FRONT] =
        LEG_ANGLE_FULL_SCALE_X10 - servo_raw_x10[3];
    return 0;
}
