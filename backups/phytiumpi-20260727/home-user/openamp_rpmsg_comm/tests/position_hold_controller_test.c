#include "position_hold_controller.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static int nearly_equal(float left, float right)
{
    return fabsf(left - right) < 1.0e-6f;
}

int main(void)
{
    PositionHoldConfig config = {
        .kp_rad_per_m = 0.10f,
        .kd_rad_per_m_s = 0.20f,
        .pitch_limit_rad = 0.05f,
        .enabled = 0U,
    };

    assert(position_hold_config_is_valid(&config));
    assert(nearly_equal(position_hold_pitch_target(&config, 0.2f, 0.1f),
                        0.0f));

    config.enabled = 1U;
    assert(nearly_equal(position_hold_pitch_target(&config, 0.2f, 0.1f),
                        -0.04f));
    assert(nearly_equal(position_hold_pitch_target(&config, -0.2f, -0.1f),
                        0.04f));
    assert(nearly_equal(position_hold_pitch_target(&config, 1.0f, 0.0f),
                        -0.05f));
    assert(nearly_equal(position_hold_pitch_target(&config, -1.0f, 0.0f),
                        0.05f));
    assert(nearly_equal(position_hold_pitch_target(&config, 0.0f, 0.1f),
                        -0.02f));
    assert(nearly_equal(position_hold_pitch_target(&config, NAN, 0.0f),
                        0.0f));

    config.pitch_limit_rad = 0.0f;
    assert(!position_hold_config_is_valid(&config));
    assert(nearly_equal(position_hold_pitch_target(&config, 1.0f, 1.0f),
                        0.0f));
    assert(!position_hold_config_is_valid(NULL));

    printf("position hold controller tests passed\n");
    return 0;
}
