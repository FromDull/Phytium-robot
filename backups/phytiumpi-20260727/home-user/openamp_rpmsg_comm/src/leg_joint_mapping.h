#ifndef LEG_JOINT_MAPPING_H
#define LEG_JOINT_MAPPING_H

#include <stdint.h>

#define LEG_JOINT_COUNT 4U
#define LEG_ANGLE_FULL_SCALE_X10 1800U
#define LEG_STABLE_EFFECTIVE_ANGLE_X10 450U

typedef enum {
    LEG_JOINT_RIGHT_FRONT = 0,
    LEG_JOINT_RIGHT_REAR = 1,
    LEG_JOINT_LEFT_FRONT = 2,
    LEG_JOINT_LEFT_REAR = 3
} LegJointId;

int leg_effective_to_servo_raw_x10(
    const uint16_t effective_x10[LEG_JOINT_COUNT],
    uint16_t servo_raw_x10[LEG_JOINT_COUNT]);
int leg_servo_raw_to_effective_x10(
    const uint16_t servo_raw_x10[LEG_JOINT_COUNT],
    uint16_t effective_x10[LEG_JOINT_COUNT]);

#endif
