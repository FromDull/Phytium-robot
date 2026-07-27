#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include <stdint.h>

#define MOTOR_CAN_DLC 8

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[MOTOR_CAN_DLC];
} MotorCanFrame;

typedef struct {
    uint8_t motor_id;
    int32_t position_x100_deg;
    int16_t speed_rpm;
    int16_t current_x100_a;
    uint64_t update_tick;
    uint8_t valid;
} MotorFeedback;

typedef struct {
    uint8_t motor_id;
    uint16_t address;
    uint32_t value;
    uint64_t update_tick;
    uint8_t valid;
} MotorRegisterValue;

void motor_build_pvt(uint8_t motor_id, int32_t position_x100_deg,
                     uint16_t speed_rpm, uint8_t torque_percent,
                     MotorCanFrame *frame);
void motor_build_enable(uint8_t motor_id, MotorCanFrame *frame);
void motor_build_set_mode(uint8_t motor_id, uint16_t mode, MotorCanFrame *frame);
void motor_build_set_origin(uint8_t motor_id, MotorCanFrame *frame);
void motor_build_set_temporary_origin(uint8_t motor_id, MotorCanFrame *frame);
void motor_build_safe_stop(uint8_t motor_id, MotorCanFrame *frame);
void motor_build_torque(uint8_t motor_id, int16_t torque_x100_nm,
                        MotorCanFrame *frame);
void motor_build_read_u32(uint8_t motor_id, uint16_t address,
                          MotorCanFrame *frame);
void motor_build_idle(uint8_t motor_id, MotorCanFrame *frame);
int motor_parse_feedback(const MotorCanFrame *frame, MotorFeedback *feedback);
int motor_parse_register_u32(const MotorCanFrame *frame,
                             MotorRegisterValue *value);

#endif
