#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <stdint.h>

#define GIMBAL_YAW_MOTOR_ID 3U
#define GIMBAL_PITCH_MOTOR_ID 4U

typedef enum {
    GIMBAL_STATE_DISABLED = 0,
    GIMBAL_STATE_STARTING = 1,
    GIMBAL_STATE_HOMING = 2,
    GIMBAL_STATE_ACTIVE = 3,
    GIMBAL_STATE_RETURNING = 4,
    GIMBAL_STATE_STOPPING = 5,
    GIMBAL_STATE_FAULT = 6
} GimbalState;

typedef enum {
    GIMBAL_AXIS_YAW = 0,
    GIMBAL_AXIS_PITCH = 1
} GimbalAxis;

typedef enum {
    GIMBAL_LIMIT_MIN = 0,
    GIMBAL_LIMIT_MAX = 1
} GimbalLimitSide;

enum {
    GIMBAL_LIMIT_YAW_MIN_VALID = 1U << 0,
    GIMBAL_LIMIT_YAW_MAX_VALID = 1U << 1,
    GIMBAL_LIMIT_PITCH_MIN_VALID = 1U << 2,
    GIMBAL_LIMIT_PITCH_MAX_VALID = 1U << 3,
    GIMBAL_LIMIT_ALL_VALID = 0x0fU
};

enum {
    GIMBAL_FAULT_NONE = 0,
    GIMBAL_FAULT_YAW_FEEDBACK = 1U << 0,
    GIMBAL_FAULT_PITCH_FEEDBACK = 1U << 1,
    GIMBAL_FAULT_CAN = 1U << 2,
    GIMBAL_FAULT_LIMIT_CONFIG = 1U << 3,
    GIMBAL_FAULT_LIMIT_EXCEEDED = 1U << 4,
    GIMBAL_FAULT_HOME_TIMEOUT = 1U << 5,
    GIMBAL_FAULT_COMMAND_TIMEOUT = 1U << 6,
    GIMBAL_FAULT_MOTION_TIMEOUT = 1U << 7
};

enum {
    GIMBAL_STATUS_OK = 0,
    GIMBAL_STATUS_INVALID = 1,
    GIMBAL_STATUS_BUSY = 2,
    GIMBAL_STATUS_NO_FEEDBACK = 3,
    GIMBAL_STATUS_LIMITS_NOT_READY = 4,
    GIMBAL_STATUS_CAN_ERROR = 5
};

typedef struct {
    int32_t yaw_min_x100_deg;
    int32_t yaw_max_x100_deg;
    int32_t pitch_min_x100_deg;
    int32_t pitch_max_x100_deg;
    uint8_t valid_mask;
} GimbalLimits;

typedef struct {
    uint8_t state;
    uint8_t fault;
    uint8_t limits_valid_mask;
    uint8_t feedback_valid_mask;
    int32_t yaw_position_x100_deg;
    int32_t pitch_position_x100_deg;
    int16_t yaw_speed_rpm;
    int16_t pitch_speed_rpm;
    int16_t yaw_current_x100_a;
    int16_t pitch_current_x100_a;
    int32_t yaw_target_x100_deg;
    int32_t pitch_target_x100_deg;
    GimbalLimits limits;
    uint16_t command_speed_rpm;
    uint8_t command_torque_percent;
    uint32_t yaw_feedback_age_ms;
    uint32_t pitch_feedback_age_ms;
    uint32_t command_timeout_remaining_ms;
    int32_t startup_pitch_x100_deg;
} GimbalTelemetry;

int gimbal_control_init(void);
void gimbal_control_poll(void);
void gimbal_control_shutdown(void);
int gimbal_control_enable(uint8_t home_torque_percent,
                          uint16_t home_speed_rpm,
                          uint8_t return_torque_percent,
                          uint16_t return_speed_rpm);
int gimbal_control_disable(void);
void gimbal_control_emergency_stop(void);
int gimbal_control_set_target(int32_t yaw_x100_deg,
                              int32_t pitch_x100_deg,
                              uint16_t speed_rpm,
                              uint8_t torque_percent,
                              uint16_t timeout_ms);
int gimbal_control_calibrate_limit(uint8_t axis, uint8_t side);
int gimbal_control_set_limits(const GimbalLimits *limits);
int gimbal_control_reset_limits(void);
const GimbalTelemetry *gimbal_control_get_telemetry(void);

#endif
