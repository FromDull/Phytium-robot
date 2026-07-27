#ifndef PHYTIUM_SERVO_PORT_H
#define PHYTIUM_SERVO_PORT_H

#include <stdint.h>

#define PHYTIUM_SERVO_NUM 4

typedef struct {
    int init_ret;
    int last_ret;
    uint8_t last_servo_id;
    uint8_t last_pwm_id;
    uint8_t last_channel;
    uint16_t angle_deg[PHYTIUM_SERVO_NUM];
    uint16_t angle_x10_deg[PHYTIUM_SERVO_NUM];
    uint16_t pulse_us[PHYTIUM_SERVO_NUM];
} PhytiumServoDebugState;

int phytium_servo_init(void);
void phytium_servo_set_polarity(uint8_t polarity);
int phytium_servo_set_angle(uint8_t servo_id, uint16_t angle_deg);
int phytium_servo_set_all(const uint16_t angle_deg[PHYTIUM_SERVO_NUM]);
int phytium_servo_set_angle_x10(uint8_t servo_id, uint16_t angle_x10_deg);
int phytium_servo_set_all_x10(
    const uint16_t angle_x10_deg[PHYTIUM_SERVO_NUM]);
int phytium_servo_enable_single_x10(uint8_t servo_id,
                                    uint16_t angle_x10_deg);
void phytium_servo_disable_outputs(void);
const PhytiumServoDebugState *phytium_servo_get_debug_state(void);

#endif
