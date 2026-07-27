#include "servo_motion_controller.h"

#include "fgeneric_timer.h"
#include "fparameters.h"

#include <stdint.h>
#include <string.h>

#define SERVO_MOTION_UPDATE_HZ 50U
#define SERVO_MOTION_MIN_DURATION_MS 100U
#define SERVO_MOTION_MAX_DURATION_MS 10000U
#define SERVO_MOTION_MAX_ANGLE_X10_DEG 1800U
#define SERVO_MOTION_STARTUP_SETTLE_MS 3000U

static ServoMotionTelemetry g_telemetry;
static uint16_t g_start_angle_x10_deg[PHYTIUM_SERVO_NUM];
static uint64_t g_start_tick;
static uint64_t g_next_update_tick;
static uint64_t g_timer_frequency;
static uint32_t g_duration_ms;

static uint32_t elapsed_ms(uint64_t now)
{
    if (g_timer_frequency == 0U) {
        return 0U;
    }
    return (uint32_t)(((now - g_start_tick) * 1000U) / g_timer_frequency);
}

static void update_debug_pulses(void)
{
    const PhytiumServoDebugState *debug = phytium_servo_get_debug_state();

    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_telemetry.pulse_us[i] = debug->pulse_us[i];
    }
}

int servo_motion_init(void)
{
    const PhytiumServoDebugState *debug = phytium_servo_get_debug_state();

    memset(&g_telemetry, 0, sizeof(g_telemetry));
    g_timer_frequency = GenericTimerFrequecy();
    if (g_timer_frequency == 0U) {
        g_telemetry.state = SERVO_MOTION_FAULT;
        g_telemetry.last_error = -1;
        return -1;
    }

    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_telemetry.current_angle_x10_deg[i] = debug->angle_x10_deg[i];
        g_telemetry.target_angle_x10_deg[i] = debug->angle_x10_deg[i];
    }
    update_debug_pulses();
    /* PWM servos have no position feedback. Debug values after reset are not
     * proof of the physical pose, so motion must remain blocked until an
     * operator explicitly enables a supported startup at the safe target. */
    g_telemetry.state = SERVO_MOTION_UNARMED;
    return 0;
}

int servo_motion_enable_at_target(
    const uint16_t target_angle_x10_deg[PHYTIUM_SERVO_NUM])
{
    uint64_t now;

    if (target_angle_x10_deg == NULL) {
        return SERVO_MOTION_INVALID;
    }
    if (g_telemetry.state == SERVO_MOTION_MOVING ||
        g_telemetry.state == SERVO_MOTION_STARTING) {
        return SERVO_MOTION_BUSY;
    }
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        if (target_angle_x10_deg[i] > SERVO_MOTION_MAX_ANGLE_X10_DEG) {
            return SERVO_MOTION_INVALID;
        }
    }
    if (phytium_servo_set_all_x10(target_angle_x10_deg) != 0) {
        g_telemetry.state = SERVO_MOTION_FAULT;
        g_telemetry.last_error = -2;
        return SERVO_MOTION_HARDWARE_ERROR;
    }
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        /* These are commanded coordinates. PWM servos provide no measured
         * joint position, especially during the first movement after enable. */
        g_telemetry.current_angle_x10_deg[i] = target_angle_x10_deg[i];
        g_telemetry.target_angle_x10_deg[i] = target_angle_x10_deg[i];
    }
    now = GenericTimerRead(GENERIC_TIMER_ID0);
    g_start_tick = now;
    g_duration_ms = SERVO_MOTION_STARTUP_SETTLE_MS;
    g_telemetry.remaining_ms = SERVO_MOTION_STARTUP_SETTLE_MS;
    g_telemetry.last_error = 0;
    g_telemetry.state = SERVO_MOTION_STARTING;
    update_debug_pulses();
    return SERVO_MOTION_OK;
}

int servo_motion_start(
    const uint16_t target_angle_x10_deg[PHYTIUM_SERVO_NUM],
    uint16_t duration_ms)
{
    uint64_t now;

    if (target_angle_x10_deg == NULL ||
        duration_ms < SERVO_MOTION_MIN_DURATION_MS ||
        duration_ms > SERVO_MOTION_MAX_DURATION_MS) {
        return SERVO_MOTION_INVALID;
    }
    if (g_telemetry.state == SERVO_MOTION_MOVING ||
        g_telemetry.state == SERVO_MOTION_STARTING) {
        return SERVO_MOTION_BUSY;
    }
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        if (target_angle_x10_deg[i] > SERVO_MOTION_MAX_ANGLE_X10_DEG) {
            return SERVO_MOTION_INVALID;
        }
    }
    if (g_telemetry.state != SERVO_MOTION_IDLE) {
        return SERVO_MOTION_NOT_ARMED;
    }

    now = GenericTimerRead(GENERIC_TIMER_ID0);
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_start_angle_x10_deg[i] = g_telemetry.current_angle_x10_deg[i];
        g_telemetry.target_angle_x10_deg[i] = target_angle_x10_deg[i];
    }
    g_start_tick = now;
    g_next_update_tick = now;
    g_duration_ms = duration_ms;
    g_telemetry.remaining_ms = duration_ms;
    g_telemetry.last_error = 0;
    g_telemetry.state = SERVO_MOTION_MOVING;
    return SERVO_MOTION_OK;
}

int servo_motion_test_one(uint8_t servo_id, uint16_t angle_x10_deg,
                          uint16_t duration_ms)
{
    uint64_t now;

    if (servo_id >= PHYTIUM_SERVO_NUM ||
        angle_x10_deg > SERVO_MOTION_MAX_ANGLE_X10_DEG ||
        duration_ms < SERVO_MOTION_MIN_DURATION_MS ||
        duration_ms > SERVO_MOTION_MAX_DURATION_MS) {
        return SERVO_MOTION_INVALID;
    }
    if (g_telemetry.state == SERVO_MOTION_MOVING ||
        g_telemetry.state == SERVO_MOTION_STARTING ||
        g_telemetry.state == SERVO_MOTION_TESTING) {
        return SERVO_MOTION_BUSY;
    }
    if (phytium_servo_enable_single_x10(servo_id, angle_x10_deg) != 0) {
        g_telemetry.state = SERVO_MOTION_FAULT;
        g_telemetry.last_error = -2;
        return SERVO_MOTION_HARDWARE_ERROR;
    }

    g_telemetry.current_angle_x10_deg[servo_id] = angle_x10_deg;
    g_telemetry.target_angle_x10_deg[servo_id] = angle_x10_deg;
    now = GenericTimerRead(GENERIC_TIMER_ID0);
    g_start_tick = now;
    g_duration_ms = duration_ms;
    g_telemetry.remaining_ms = duration_ms;
    g_telemetry.last_error = 0;
    g_telemetry.state = SERVO_MOTION_TESTING;
    update_debug_pulses();
    return SERVO_MOTION_OK;
}

void servo_motion_stop(void)
{
    if (g_telemetry.state == SERVO_MOTION_MOVING) {
        for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
            g_telemetry.target_angle_x10_deg[i] =
                g_telemetry.current_angle_x10_deg[i];
        }
    }
    g_telemetry.remaining_ms = 0U;
    g_telemetry.state = SERVO_MOTION_UNARMED;
    phytium_servo_disable_outputs();
    update_debug_pulses();
}

void servo_motion_poll(void)
{
    uint16_t command[PHYTIUM_SERVO_NUM];
    uint64_t now;
    uint32_t passed_ms;

    if (g_telemetry.state != SERVO_MOTION_MOVING &&
        g_telemetry.state != SERVO_MOTION_STARTING &&
        g_telemetry.state != SERVO_MOTION_TESTING) {
        return;
    }
    now = GenericTimerRead(GENERIC_TIMER_ID0);
    if (g_telemetry.state == SERVO_MOTION_TESTING) {
        passed_ms = elapsed_ms(now);
        if (passed_ms >= g_duration_ms) {
            servo_motion_stop();
        } else {
            g_telemetry.remaining_ms = g_duration_ms - passed_ms;
        }
        return;
    }
    if (g_telemetry.state == SERVO_MOTION_STARTING) {
        passed_ms = elapsed_ms(now);
        if (passed_ms >= g_duration_ms) {
            g_telemetry.remaining_ms = 0U;
            g_telemetry.state = SERVO_MOTION_IDLE;
        } else {
            g_telemetry.remaining_ms = g_duration_ms - passed_ms;
        }
        return;
    }
    if ((int64_t)(now - g_next_update_tick) < 0) {
        return;
    }
    g_next_update_tick = now + g_timer_frequency / SERVO_MOTION_UPDATE_HZ;
    passed_ms = elapsed_ms(now);
    if (passed_ms > g_duration_ms) {
        passed_ms = g_duration_ms;
    }

    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        int32_t start = g_start_angle_x10_deg[i];
        int32_t delta = (int32_t)g_telemetry.target_angle_x10_deg[i] - start;
        int32_t interpolated = start +
            (delta * (int32_t)passed_ms) / (int32_t)g_duration_ms;
        command[i] = (uint16_t)interpolated;
    }

    if (phytium_servo_set_all_x10(command) != 0) {
        g_telemetry.state = SERVO_MOTION_FAULT;
        g_telemetry.last_error = -2;
        g_telemetry.remaining_ms = 0U;
        return;
    }
    for (uint8_t i = 0; i < PHYTIUM_SERVO_NUM; ++i) {
        g_telemetry.current_angle_x10_deg[i] = command[i];
    }
    update_debug_pulses();
    g_telemetry.remaining_ms = g_duration_ms - passed_ms;
    if (passed_ms == g_duration_ms) {
        g_telemetry.state = SERVO_MOTION_IDLE;
    }
}

const ServoMotionTelemetry *servo_motion_get_telemetry(void)
{
    return &g_telemetry;
}
