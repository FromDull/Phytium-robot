#ifndef POSITION_HOLD_CONTROLLER_H
#define POSITION_HOLD_CONTROLLER_H

#include <stdint.h>

typedef struct {
    float kp_rad_per_m;
    float kd_rad_per_m_s;
    float pitch_limit_rad;
    uint8_t enabled;
} PositionHoldConfig;

int position_hold_config_is_valid(const PositionHoldConfig *config);
float position_hold_pitch_target(const PositionHoldConfig *config,
                                 float position_error_m,
                                 float velocity_error_m_s);

#endif
