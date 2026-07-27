#include "phytium_servo_port.h"

#include "fio_mux.h"
#include "fpwm.h"
#include "ftypes.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#define SERVO_PERIOD_US 20000U
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U
#define SERVO_MAX_ANGLE_DEG 180U
#define SERVO_MAX_ANGLE_X10_DEG 1800U

/*
 * One FPWM controller has two independent channels when dead-zone output is
 * bypassed. Adjust this table after confirming the actual pins used on the
 * carrier board.
 */
typedef struct {
    u32 pwm_id;
    u32 channel;
    u8 enabled;
} ServoPwmMap;

static const ServoPwmMap g_servo_map[PHYTIUM_SERVO_NUM] = {
    {1, 0, TRUE},  /* servo0 -> PWM2_OUT -> AG57 -> 40Pin Pin32 */
    {2, 1, TRUE},  /* servo1 -> PWM5_OUT -> C39  -> 40Pin Pin33 */
    {3, 0, TRUE},  /* servo2 -> PWM6_OUT -> A37  -> 40Pin Pin7 */
    {3, 1, TRUE},  /* servo3 -> PWM7_OUT -> A43 -> released after CAN moves to CAN1 */
};

static FPwmCtrl g_pwm_ctrl[FPWM_NUM];
static int g_servo_ready = 0;
static int g_servo_outputs_enabled = 0;
static uint8_t g_servo_polarity = FPWM_POLARITY_NORMAL;
static const uint16_t g_safe_reference_x10_deg[PHYTIUM_SERVO_NUM] = {
    450U, 1350U, 450U, 1350U
};
static PhytiumServoDebugState g_servo_debug = {
    .init_ret = -99,
    .last_ret = -99,
    .last_servo_id = 0xff,
    .last_pwm_id = 0xff,
    .last_channel = 0xff,
    .angle_deg = {45, 135, 45, 135},
    .angle_x10_deg = {450, 1350, 450, 1350},
    .pulse_us = {0, 0, 0, 0},
};

static uint16_t servo_clamp_angle(uint16_t angle_deg)
{
    return angle_deg > SERVO_MAX_ANGLE_DEG ? SERVO_MAX_ANGLE_DEG : angle_deg;
}

static uint16_t servo_clamp_angle_x10(uint16_t angle_x10_deg)
{
    return angle_x10_deg > SERVO_MAX_ANGLE_X10_DEG ?
        SERVO_MAX_ANGLE_X10_DEG : angle_x10_deg;
}

static uint16_t servo_angle_x10_to_pulse_us(uint16_t angle_x10_deg)
{
    angle_x10_deg = servo_clamp_angle_x10(angle_x10_deg);
    return (uint16_t)(SERVO_MIN_US +
        ((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle_x10_deg) /
        SERVO_MAX_ANGLE_X10_DEG);
}

static uint16_t servo_pulse_to_ccr_us(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_US) {
        pulse_us = SERVO_MIN_US;
    }
    if (pulse_us > SERVO_MAX_US) {
        pulse_us = SERVO_MAX_US;
    }

    /*
     * With 50 Hz PWM, using period=20000 lets the CCR value correspond to us.
     * If a specific SDK/platform interprets period in clock ticks, keep the
     * mapping here as the only place to adjust.
     */
    return pulse_us;
}

static void servo_fill_default_cfg(FPwmDbVariableConfig *db_cfg, FPwmVariableConfig *pwm_cfg, uint16_t pulse_us)
{
    memset(db_cfg, 0, sizeof(*db_cfg));
    memset(pwm_cfg, 0, sizeof(*pwm_cfg));

    db_cfg->db_out_mode = FPWM_DB_OUT_MODE_BYPASS;

    pwm_cfg->tim_ctrl_mode = 0;
    pwm_cfg->tim_ctrl_div = 49;
    pwm_cfg->pwm_period = SERVO_PERIOD_US;
    pwm_cfg->pwm_mode = FPWM_OUTPUT_COMPARE;
    pwm_cfg->pwm_polarity = g_servo_polarity;
    pwm_cfg->pwm_duty_source_mode = FPWM_DUTY_CCR;
    pwm_cfg->pwm_pulse = pulse_us;
}

const PhytiumServoDebugState *phytium_servo_get_debug_state(void)
{
    return &g_servo_debug;
}

void phytium_servo_set_polarity(uint8_t polarity)
{
    if (polarity >= FPWM_POLARITY_NUM) {
        g_servo_debug.last_ret = -4;
        return;
    }

    g_servo_polarity = polarity;
    g_servo_ready = 0;
    g_servo_debug.last_ret = 0;
}

int phytium_servo_init(void)
{
    FError ret;
    FPwmDbVariableConfig db_cfg;
    FPwmVariableConfig pwm_cfg;
    u8 pwm_inited[FPWM_NUM];

    if (g_servo_ready) {
        g_servo_debug.init_ret = 0;
        return 0;
    }

    memset(g_pwm_ctrl, 0, sizeof(g_pwm_ctrl));
    memset(pwm_inited, 0, sizeof(pwm_inited));
    servo_fill_default_cfg(&db_cfg, &pwm_cfg,
                           servo_angle_x10_to_pulse_us(
                               g_safe_reference_x10_deg[0]));

    FIOMuxInit();

    for (u32 i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        const ServoPwmMap *map = &g_servo_map[i];
        u32 pwm_id = map->pwm_id;
        const FPwmConfig *cfg;

        if (!map->enabled) {
            continue;
        }

        if (pwm_id >= FPWM_NUM) {
            g_servo_debug.init_ret = -1;
            printf("servo_init: pwm id out of range servo=%u pwm=%u\r\n",
                   (unsigned)i, (unsigned)pwm_id);
            return -1;
        }

        if (pwm_inited[pwm_id]) {
            continue;
        }

        cfg = FPwmLookupConfig(pwm_id);
        if (!cfg) {
            g_servo_debug.init_ret = -1;
            printf("servo_init: FPwmLookupConfig(%u) failed\r\n", (unsigned)pwm_id);
            return -1;
        }

        ret = FPwmCfgInitialize(&g_pwm_ctrl[pwm_id], cfg);
        if (ret != FPWM_SUCCESS) {
            g_servo_debug.init_ret = -2;
            printf("servo_init: FPwmCfgInitialize(%u) failed ret=%d\r\n", (unsigned)pwm_id, ret);
            return -2;
        }

        ret = FPwmDbVariableSet(&g_pwm_ctrl[pwm_id], &db_cfg);
        if (ret != FPWM_SUCCESS) {
            g_servo_debug.init_ret = -3;
            printf("servo_init: FPwmDbVariableSet(%u) failed ret=%d\r\n", (unsigned)pwm_id, ret);
            return -3;
        }

        pwm_inited[pwm_id] = TRUE;
    }

    for (u32 i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        const ServoPwmMap *map = &g_servo_map[i];
        if (!map->enabled) {
            continue;
        }

        servo_fill_default_cfg(&db_cfg, &pwm_cfg,
                               servo_angle_x10_to_pulse_us(
                                   g_safe_reference_x10_deg[i]));
        FIOPadSetPwmMux(map->pwm_id, map->channel);
        ret = FPwmVariableSet(&g_pwm_ctrl[map->pwm_id], map->channel, &pwm_cfg);
        if (ret != FPWM_SUCCESS) {
            g_servo_debug.init_ret = -4;
            g_servo_debug.last_servo_id = (uint8_t)i;
            g_servo_debug.last_pwm_id = (uint8_t)map->pwm_id;
            g_servo_debug.last_channel = (uint8_t)map->channel;
            printf("servo_init: FPwmVariableSet servo=%u pwm=%u ch=%u ret=%d\r\n",
                   (unsigned)i, (unsigned)map->pwm_id, (unsigned)map->channel, ret);
            return -4;
        }
    }

    g_servo_ready = 1;
    g_servo_outputs_enabled = 0;
    g_servo_debug.init_ret = 0;
    g_servo_debug.last_ret = 0;
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_servo_debug.pulse_us[i] = 0U;
    }
    return 0;
}

int phytium_servo_set_angle(uint8_t servo_id, uint16_t angle_deg)
{
    angle_deg = servo_clamp_angle(angle_deg);
    return phytium_servo_set_angle_x10(servo_id,
                                       (uint16_t)(angle_deg * 10U));
}

int phytium_servo_set_angle_x10(uint8_t servo_id, uint16_t angle_x10_deg)
{
    FError ret;
    uint16_t pulse_us;
    uint16_t ccr;
    const ServoPwmMap *map;

    if (servo_id >= PHYTIUM_SERVO_NUM) {
        g_servo_debug.last_ret = -1;
        return -1;
    }

    if (!g_servo_ready) {
        int init_ret = phytium_servo_init();
        if (init_ret != 0) {
            g_servo_debug.last_ret = -2;
            return -2;
        }
    }

    if (!g_servo_outputs_enabled) {
        g_servo_debug.last_ret = -4;
        return -4;
    }

    angle_x10_deg = servo_clamp_angle_x10(angle_x10_deg);
    pulse_us = servo_angle_x10_to_pulse_us(angle_x10_deg);
    ccr = servo_pulse_to_ccr_us(pulse_us);
    map = &g_servo_map[servo_id];

    if (!map->enabled) {
        g_servo_debug.angle_deg[servo_id] = angle_x10_deg / 10U;
        g_servo_debug.angle_x10_deg[servo_id] = angle_x10_deg;
        g_servo_debug.pulse_us[servo_id] = pulse_us;
        g_servo_debug.last_servo_id = servo_id;
        g_servo_debug.last_pwm_id = (uint8_t)map->pwm_id;
        g_servo_debug.last_channel = (uint8_t)map->channel;
        g_servo_debug.last_ret = 0;
        return 0;
    }

    ret = FPwmPulseSet(&g_pwm_ctrl[map->pwm_id], map->channel, ccr);
    if (ret != FPWM_SUCCESS) {
        g_servo_debug.last_ret = -3;
        g_servo_debug.last_servo_id = servo_id;
        g_servo_debug.last_pwm_id = (uint8_t)map->pwm_id;
        g_servo_debug.last_channel = (uint8_t)map->channel;
        printf("servo_set: FPwmPulseSet servo=%u pwm=%u ch=%u ccr=%u ret=%d\r\n",
               servo_id, (unsigned)map->pwm_id, (unsigned)map->channel, ccr, ret);
        return -3;
    }

    g_servo_debug.angle_deg[servo_id] = angle_x10_deg / 10U;
    g_servo_debug.angle_x10_deg[servo_id] = angle_x10_deg;
    g_servo_debug.pulse_us[servo_id] = pulse_us;
    g_servo_debug.last_servo_id = servo_id;
    g_servo_debug.last_pwm_id = (uint8_t)map->pwm_id;
    g_servo_debug.last_channel = (uint8_t)map->channel;
    g_servo_debug.last_ret = 0;
    return 0;
}

int phytium_servo_set_all(const uint16_t angle_deg[PHYTIUM_SERVO_NUM])
{
    uint16_t angle_x10_deg[PHYTIUM_SERVO_NUM];

    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        angle_x10_deg[i] = (uint16_t)(servo_clamp_angle(angle_deg[i]) * 10U);
    }
    return phytium_servo_set_all_x10(angle_x10_deg);
}

int phytium_servo_set_all_x10(
    const uint16_t angle_x10_deg[PHYTIUM_SERVO_NUM])
{
    int ret = 0;

    if (angle_x10_deg == NULL) {
        return -1;
    }
    if (!g_servo_ready && phytium_servo_init() != 0) {
        return -2;
    }
    if (!g_servo_outputs_enabled) {
        /* Program every disabled channel before enabling any channel. This
         * prevents the linkage from briefly seeing the 90-degree defaults. */
        for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
            const ServoPwmMap *map = &g_servo_map[i];
            uint16_t angle = servo_clamp_angle_x10(angle_x10_deg[i]);
            uint16_t pulse_us = servo_angle_x10_to_pulse_us(angle);
            FError one_ret = FPWM_SUCCESS;

            if (map->enabled) {
                one_ret = FPwmPulseSet(&g_pwm_ctrl[map->pwm_id], map->channel,
                                       servo_pulse_to_ccr_us(pulse_us));
            }
            if (one_ret != FPWM_SUCCESS) {
                g_servo_debug.last_ret = -3;
                return -3;
            }
            g_servo_debug.angle_deg[i] = angle / 10U;
            g_servo_debug.angle_x10_deg[i] = angle;
            g_servo_debug.pulse_us[i] = pulse_us;
        }
        for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
            const ServoPwmMap *map = &g_servo_map[i];

            if (map->enabled) {
                FPwmEnable(&g_pwm_ctrl[map->pwm_id], map->channel);
            }
        }
        g_servo_outputs_enabled = 1;
        g_servo_debug.last_ret = 0;
        return 0;
    }

    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        int one_ret = phytium_servo_set_angle_x10(i, angle_x10_deg[i]);
        if (one_ret != 0) {
            ret = one_ret;
        }
    }
    return ret;
}

int phytium_servo_enable_single_x10(uint8_t servo_id,
                                    uint16_t angle_x10_deg)
{
    const ServoPwmMap *map;
    uint16_t pulse_us;
    FError ret;

    if (servo_id >= PHYTIUM_SERVO_NUM ||
        angle_x10_deg > SERVO_MAX_ANGLE_X10_DEG) {
        g_servo_debug.last_ret = -1;
        return -1;
    }
    if (!g_servo_ready && phytium_servo_init() != 0) {
        g_servo_debug.last_ret = -2;
        return -2;
    }

    /* A commissioning test must never leave another linkage energized. */
    phytium_servo_disable_outputs();
    map = &g_servo_map[servo_id];
    pulse_us = servo_angle_x10_to_pulse_us(angle_x10_deg);
    if (map->enabled) {
        ret = FPwmPulseSet(&g_pwm_ctrl[map->pwm_id], map->channel,
                           servo_pulse_to_ccr_us(pulse_us));
        if (ret != FPWM_SUCCESS) {
            g_servo_debug.last_ret = -3;
            return -3;
        }
        FPwmEnable(&g_pwm_ctrl[map->pwm_id], map->channel);
    }
    g_servo_outputs_enabled = 1;
    g_servo_debug.angle_deg[servo_id] = angle_x10_deg / 10U;
    g_servo_debug.angle_x10_deg[servo_id] = angle_x10_deg;
    g_servo_debug.pulse_us[servo_id] = pulse_us;
    g_servo_debug.last_servo_id = servo_id;
    g_servo_debug.last_pwm_id = (uint8_t)map->pwm_id;
    g_servo_debug.last_channel = (uint8_t)map->channel;
    g_servo_debug.last_ret = 0;
    return 0;
}

void phytium_servo_disable_outputs(void)
{
    if (g_servo_ready && g_servo_outputs_enabled) {
        for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
            const ServoPwmMap *map = &g_servo_map[i];

            if (map->enabled) {
                FPwmDisable(&g_pwm_ctrl[map->pwm_id], map->channel);
            }
        }
    }
    g_servo_outputs_enabled = 0;
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_servo_debug.pulse_us[i] = 0U;
    }
    g_servo_debug.last_ret = 0;
}
