#ifndef SERVO_MOTION_CONTROLLER_H
#define SERVO_MOTION_CONTROLLER_H

#include <stdint.h>

#include "phytium_servo_port.h"

typedef enum {
    SERVO_MOTION_IDLE = 0,
    SERVO_MOTION_MOVING = 1,
    SERVO_MOTION_FAULT = 2,
    SERVO_MOTION_UNARMED = 3,
    SERVO_MOTION_STARTING = 4,
    SERVO_MOTION_TESTING = 5
} ServoMotionState;

enum {
    SERVO_MOTION_OK = 0,
    SERVO_MOTION_INVALID = 1,
    SERVO_MOTION_BUSY = 2,
    SERVO_MOTION_BALANCE_ACTIVE = 3,
    SERVO_MOTION_HARDWARE_ERROR = 4,
    SERVO_MOTION_NOT_ARMED = 5
};

typedef struct {
    uint8_t state;
    int8_t last_error;
    uint16_t current_angle_x10_deg[PHYTIUM_SERVO_NUM];
    uint16_t target_angle_x10_deg[PHYTIUM_SERVO_NUM];
    uint16_t pulse_us[PHYTIUM_SERVO_NUM];
    uint32_t remaining_ms;
} ServoMotionTelemetry;

int servo_motion_init(void);
int servo_motion_enable_at_target(
    const uint16_t target_angle_x10_deg[PHYTIUM_SERVO_NUM]);
int servo_motion_start(
    const uint16_t target_angle_x10_deg[PHYTIUM_SERVO_NUM],
    uint16_t duration_ms);
int servo_motion_test_one(uint8_t servo_id, uint16_t angle_x10_deg,
                          uint16_t duration_ms);
void servo_motion_stop(void);
void servo_motion_poll(void);
const ServoMotionTelemetry *servo_motion_get_telemetry(void);

#endif
