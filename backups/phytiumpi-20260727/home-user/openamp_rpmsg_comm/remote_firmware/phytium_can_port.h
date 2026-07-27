#ifndef PHYTIUM_CAN_PORT_H
#define PHYTIUM_CAN_PORT_H

#include "motor_can.h"

typedef struct {
    int init_ret;
    int last_send_ret;
    uint32_t can_id;
    uint32_t baudrate;
    uint32_t send_count;
    uint32_t receive_count;
    uint32_t feedback_count;
    int last_receive_ret;
    uint32_t last_frame_id;
    uint8_t last_frame_dlc;
    uint8_t last_frame_data[8];
    uint32_t reg_ctrl;
    uint32_t reg_intr;
    uint32_t reg_xfer_sts;
    uint32_t reg_err_cnt;
    uint32_t reg_fifo_cnt;
    uint32_t reg_xfer_en;
} PhytiumCanDebugState;

/* Platform adapter.
 *
 * The motor protocol code is platform-independent. This file is the only
 * place that should call the Phytium Standalone SDK CAN driver.
 *
 * Return 0 on success, negative value on failure.
 */
int phytium_can_init(void);
int phytium_can_send(const MotorCanFrame *frame);
int phytium_can_poll(void);
int phytium_can_get_motor_feedback(uint8_t motor_id, MotorFeedback *feedback);
void phytium_can_clear_motor_feedback(uint8_t motor_id);
void phytium_can_clear_register_value(uint8_t motor_id);
int phytium_can_get_register_value(uint8_t motor_id,
                                   MotorRegisterValue *value);
int phytium_can_bus_ok(void);
const PhytiumCanDebugState *phytium_can_get_debug_state(void);

#endif
