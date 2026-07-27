#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <stdint.h>

typedef struct {
    /* Discrete LQR gain for total wheel torque, tau_left + tau_right. */
    float k_theta;
    float k_theta_rate;
    float k_position;
    float k_velocity;
    float wheel_radius_m;
    float torque_limit_nm;
    float fall_angle_rad;
    float pitch_offset_rad;
    float posture_priority_angle_rad;
    float left_motor_direction;
    float right_motor_direction;
} LqrConfig;

typedef struct {
    float pitch_rad;
    float pitch_rate_rad_s;
    float left_position_rad;
    float right_position_rad;
    float left_velocity_rad_s;
    float right_velocity_rad_s;
    uint8_t valid;
} LqrSensorData;

typedef struct {
    float left_torque_nm;
    float right_torque_nm;
    float wheel_position_m;
    float wheel_velocity_m_s;
    float total_torque_command_nm;
    uint8_t enabled;
    uint8_t fault;
} LqrOutput;

typedef struct {
    LqrConfig config;
    float wheel_zero_position_m;
    float pitch_target_rad;
    float position_target_m;
    float velocity_target_m_s;
    uint8_t initialized;
    uint8_t enabled;
} LqrController;

int lqr_init(LqrController *controller, const LqrConfig *config);
int lqr_enable(LqrController *controller, const LqrSensorData *sensor);
void lqr_disable(LqrController *controller);
int lqr_set_targets(LqrController *controller, float pitch_target_rad,
                    float position_target_m, float velocity_target_m_s);
LqrOutput lqr_update(LqrController *controller, const LqrSensorData *sensor);

#endif
