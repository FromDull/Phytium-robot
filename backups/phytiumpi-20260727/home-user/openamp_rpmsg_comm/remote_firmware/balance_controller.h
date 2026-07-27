#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

typedef enum {
    BALANCE_STATE_DISABLED = 0,
    BALANCE_STATE_ARMING = 1,
    BALANCE_STATE_ACTIVE = 2,
    BALANCE_STATE_FAULT = 3
} BalanceState;

enum {
    BALANCE_FAULT_NONE = 0,
    BALANCE_FAULT_IMU = 1U << 0,
    BALANCE_FAULT_LEFT_MOTOR = 1U << 1,
    BALANCE_FAULT_RIGHT_MOTOR = 1U << 2,
    BALANCE_FAULT_CAN = 1U << 3,
    BALANCE_FAULT_FALL = 1U << 4,
    BALANCE_FAULT_OVERRUN = 1U << 5,
    BALANCE_FAULT_ARM_TIMEOUT = 1U << 6,
    BALANCE_FAULT_SPEED = 1U << 7,
    BALANCE_FAULT_ARM_CONDITION = BALANCE_FAULT_SPEED,
    BALANCE_FAULT_CONFIG = BALANCE_FAULT_SPEED
};

enum {
    BALANCE_MOTION_OK = 0,
    BALANCE_MOTION_INVALID = 1,
    BALANCE_MOTION_NOT_ACTIVE = 2
};

typedef struct {
    uint8_t state;
    uint8_t fault;
    uint16_t control_hz;
    float pitch_rad;
    float pitch_rate_rad_s;
    float wheel_position_m;
    float wheel_velocity_m_s;
    float left_torque_nm;
    float right_torque_nm;
    float pitch_target_rad;
    float position_target_m;
    float position_error_m;
    float velocity_error_m_s;
    uint32_t loop_count;
    uint8_t position_hold_enabled;
} BalanceTelemetry;

typedef struct {
    float pitch_trim_rad;
    float k_theta;
    float k_theta_rate;
    float k_position;
    float k_velocity;
    float posture_priority_angle_rad;
    float max_wheel_speed_m_s;
    float motor_feedback_speed_scale;
    float pitch_rate_filter_hz;
    float torque_limit_nm;
    float position_hold_kp_rad_per_m;
    float position_hold_kd_rad_per_m_s;
    float position_hold_limit_rad;
    uint8_t position_hold_enabled;
} BalanceRuntimeConfig;

typedef struct {
    float target_linear_m_s;
    float target_angular_rad_s;
    float applied_linear_m_s;
    float applied_angular_rad_s;
    float measured_linear_m_s;
    float measured_angular_rad_s;
    float wheel_position_m;
    float yaw_position_rad;
    float wheel_track_m;
    uint32_t command_age_ms;
} BalanceMotionTelemetry;

enum {
    BALANCE_CONFIG_OK = 0,
    BALANCE_CONFIG_INVALID = 1,
    BALANCE_CONFIG_BUSY = 2
};

int balance_control_init(void);
typedef void (*BalanceCalibrationService)(void *context);

int balance_control_enable(void);
int balance_control_enable_serviced(BalanceCalibrationService service,
                                    void *context);
void balance_control_disable(void);
void balance_control_poll(void);
const BalanceTelemetry *balance_control_get_telemetry(void);
void balance_control_get_runtime_config(BalanceRuntimeConfig *config);
int balance_control_set_pitch_trim(float pitch_trim_rad);
int balance_control_set_gains(float k_theta, float k_theta_rate,
                              float k_position, float k_velocity);
int balance_control_reset_runtime_config(void);
int balance_control_set_speed_limit(float max_wheel_speed_m_s);
int balance_control_set_pitch_rate_filter(float cutoff_hz);
int balance_control_set_posture_priority(float angle_rad);
int balance_control_set_torque_limit(float torque_limit_nm);
int balance_control_set_position_hold(uint8_t enabled, float kp_rad_per_m,
                                      float kd_rad_per_m_s,
                                      float pitch_limit_rad);
int balance_control_set_motion_command(float linear_m_s, float angular_rad_s,
                                       uint16_t timeout_ms);
int balance_control_set_wheel_track(float wheel_track_m);
void balance_control_get_motion_telemetry(BalanceMotionTelemetry *telemetry);

#endif
