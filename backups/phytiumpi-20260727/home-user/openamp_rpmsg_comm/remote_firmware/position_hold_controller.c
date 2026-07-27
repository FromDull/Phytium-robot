#include "position_hold_controller.h"

#include <math.h>
#include <stddef.h>

int position_hold_config_is_valid(const PositionHoldConfig *config)
{
    return config != NULL &&
           (config->enabled == 0U || config->enabled == 1U) &&
           isfinite(config->kp_rad_per_m) &&
           isfinite(config->kd_rad_per_m_s) &&
           isfinite(config->pitch_limit_rad) &&
           config->kp_rad_per_m >= 0.0f &&
           config->kd_rad_per_m_s >= 0.0f &&
           config->pitch_limit_rad > 0.0f;
}

float position_hold_pitch_target(const PositionHoldConfig *config,
                                 float position_error_m,
                                 float velocity_error_m_s)
{
    float target;

    if (!position_hold_config_is_valid(config) || !config->enabled ||
        !isfinite(position_error_m) || !isfinite(velocity_error_m_s)) {
        return 0.0f;
    }

    target = -(config->kp_rad_per_m * position_error_m +
               config->kd_rad_per_m_s * velocity_error_m_s);
    if (target > config->pitch_limit_rad) {
        return config->pitch_limit_rad;
    }
    if (target < -config->pitch_limit_rad) {
        return -config->pitch_limit_rad;
    }
    return target;
}
