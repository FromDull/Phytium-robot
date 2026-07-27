#include "lqr_controller.h"

#include <math.h>
#include <string.h>

static float clampf(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static int lqr_sensor_is_finite(const LqrSensorData *sensor)
{
    return isfinite(sensor->pitch_rad) &&
           isfinite(sensor->pitch_rate_rad_s) &&
           isfinite(sensor->left_position_rad) &&
           isfinite(sensor->right_position_rad) &&
           isfinite(sensor->left_velocity_rad_s) &&
           isfinite(sensor->right_velocity_rad_s);
}

static float wheel_position(const LqrConfig *cfg,
                            const LqrSensorData *sensor)
{
    return 0.5f * cfg->wheel_radius_m *
           (cfg->left_motor_direction * sensor->left_position_rad +
            cfg->right_motor_direction * sensor->right_position_rad);
}

int lqr_init(LqrController *controller, const LqrConfig *config)
{
    if (controller == NULL || config == NULL ||
        !(config->wheel_radius_m > 0.0f) ||
        !(config->torque_limit_nm > 0.0f) ||
        !(config->fall_angle_rad > 0.0f) ||
        !(config->posture_priority_angle_rad > 0.0f) ||
        config->posture_priority_angle_rad >= config->fall_angle_rad) {
        return -1;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    controller->initialized = 1U;
    return 0;
}

int lqr_enable(LqrController *controller, const LqrSensorData *sensor)
{
    if (controller == NULL || sensor == NULL || !controller->initialized ||
        !sensor->valid || !lqr_sensor_is_finite(sensor)) {
        return -1;
    }

    controller->wheel_zero_position_m =
        wheel_position(&controller->config, sensor);
    controller->pitch_target_rad = 0.0f;
    controller->position_target_m = 0.0f;
    controller->velocity_target_m_s = 0.0f;
    controller->enabled = 1U;
    return 0;
}

void lqr_disable(LqrController *controller)
{
    if (controller != NULL) {
        controller->enabled = 0U;
    }
}

int lqr_set_targets(LqrController *controller, float pitch_target_rad,
                    float position_target_m, float velocity_target_m_s)
{
    if (controller == NULL || !controller->initialized ||
        !isfinite(pitch_target_rad) || !isfinite(position_target_m) ||
        !isfinite(velocity_target_m_s)) {
        return -1;
    }
    controller->pitch_target_rad = pitch_target_rad;
    controller->position_target_m = position_target_m;
    controller->velocity_target_m_s = velocity_target_m_s;
    return 0;
}

LqrOutput lqr_update(LqrController *controller, const LqrSensorData *sensor)
{
    LqrOutput output;
    const LqrConfig *cfg;
    float pitch;
    float wheel_pos;
    float wheel_vel;
    float posture_torque;
    float travel_torque;
    float total_torque;
    float single_torque;

    memset(&output, 0, sizeof(output));
    if (controller == NULL || sensor == NULL || !controller->initialized ||
        !sensor->valid || !lqr_sensor_is_finite(sensor)) {
        output.fault = 1U;
        return output;
    }
    if (!controller->enabled) {
        return output;
    }

    cfg = &controller->config;
    pitch = sensor->pitch_rad - cfg->pitch_offset_rad;
    if (fabsf(pitch) > cfg->fall_angle_rad) {
        controller->enabled = 0U;
        output.fault = 1U;
        return output;
    }

    wheel_pos = wheel_position(cfg, sensor) - controller->wheel_zero_position_m;
    wheel_vel = 0.5f * cfg->wheel_radius_m *
                (cfg->left_motor_direction * sensor->left_velocity_rad_s +
                 cfg->right_motor_direction * sensor->right_velocity_rad_s);
    posture_torque = -(cfg->k_theta *
                           (pitch - controller->pitch_target_rad) +
                       cfg->k_theta_rate * sensor->pitch_rate_rad_s);
    travel_torque = -(cfg->k_position *
                          (wheel_pos - controller->position_target_m) +
                      cfg->k_velocity *
                          (wheel_vel - controller->velocity_target_m_s));

    /* Preserve all available actuator authority for catching a fall. */
    if (fabsf(pitch - controller->pitch_target_rad) >=
            cfg->posture_priority_angle_rad &&
        posture_torque * travel_torque < 0.0f) {
        travel_torque = 0.0f;
    }
    total_torque = posture_torque + travel_torque;
    single_torque = clampf(0.5f * total_torque, cfg->torque_limit_nm);

    output.left_torque_nm = cfg->left_motor_direction * single_torque;
    output.right_torque_nm = cfg->right_motor_direction * single_torque;
    output.wheel_position_m = wheel_pos;
    output.wheel_velocity_m_s = wheel_vel;
    output.total_torque_command_nm = 2.0f * single_torque;
    output.enabled = 1U;
    return output;
}
