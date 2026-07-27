#include "motor_can.h"

static void motor_clear_frame(uint8_t motor_id, MotorCanFrame *frame)
{
    frame->id = 0x600u + motor_id;
    frame->dlc = MOTOR_CAN_DLC;
    for (int i = 0; i < MOTOR_CAN_DLC; ++i) {
        frame->data[i] = 0;
    }
}

static void motor_build_register_write(uint8_t motor_id, uint16_t reg_addr,
                                       uint16_t value, MotorCanFrame *frame)
{
    motor_clear_frame(motor_id, frame);

    frame->data[0] = 0x2B;
    frame->data[1] = (uint8_t)(reg_addr >> 8);
    frame->data[2] = (uint8_t)(reg_addr & 0xff);
    frame->data[3] = 0x00;

    frame->data[4] = (uint8_t)(value >> 8);
    frame->data[5] = (uint8_t)(value & 0xff);
}

void motor_build_pvt(uint8_t motor_id, int32_t position_x100_deg,
                     uint16_t speed_rpm, uint8_t torque_percent,
                     MotorCanFrame *frame)
{
    motor_clear_frame(motor_id, frame);

    frame->data[0] = 0x25;
    frame->data[1] = (uint8_t)(position_x100_deg >> 24);
    frame->data[2] = (uint8_t)(position_x100_deg >> 16);
    frame->data[3] = (uint8_t)(position_x100_deg >> 8);
    frame->data[4] = (uint8_t)(position_x100_deg & 0xff);
    frame->data[5] = (uint8_t)(speed_rpm >> 8);
    frame->data[6] = (uint8_t)(speed_rpm & 0xff);
    frame->data[7] = torque_percent;
}

void motor_build_enable(uint8_t motor_id, MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x00A2, 1, frame);
}

void motor_build_set_mode(uint8_t motor_id, uint16_t mode, MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x0060, mode, frame);
}

void motor_build_set_origin(uint8_t motor_id, MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x00A6, 1, frame);
}

void motor_build_set_temporary_origin(uint8_t motor_id, MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x00A7, 1, frame);
}

void motor_build_safe_stop(uint8_t motor_id, MotorCanFrame *frame)
{
    motor_build_pvt(motor_id, 0, 0, 0, frame);
}

void motor_build_torque(uint8_t motor_id, int16_t torque_x100_nm,
                        MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x0020,
                               (uint16_t)torque_x100_nm, frame);
}

void motor_build_read_u32(uint8_t motor_id, uint16_t address,
                          MotorCanFrame *frame)
{
    motor_clear_frame(motor_id, frame);
    frame->data[0] = 0x43;
    frame->data[1] = (uint8_t)(address >> 8);
    frame->data[2] = (uint8_t)(address & 0xff);
}

void motor_build_idle(uint8_t motor_id, MotorCanFrame *frame)
{
    motor_build_register_write(motor_id, 0x00A0, 1, frame);
}

int motor_parse_feedback(const MotorCanFrame *frame, MotorFeedback *feedback)
{
    int32_t position;

    if (frame == 0 || feedback == 0 || frame->dlc != MOTOR_CAN_DLC ||
        frame->id < 0x581U || frame->id > 0x5ffU || frame->data[0] != 0x2aU) {
        return -1;
    }

    position = ((int32_t)frame->data[1] << 16) |
               ((int32_t)frame->data[2] << 8) |
               (int32_t)frame->data[3];
    if ((position & 0x00800000L) != 0) {
        position |= (int32_t)0xff000000L;
    }

    feedback->motor_id = (uint8_t)(frame->id - 0x580U);
    feedback->position_x100_deg = position;
    feedback->speed_rpm = (int16_t)(((uint16_t)frame->data[4] << 8) |
                                    frame->data[5]);
    feedback->current_x100_a = (int16_t)(((uint16_t)frame->data[6] << 8) |
                                         frame->data[7]);
    feedback->valid = 1;
    return 0;
}

int motor_parse_register_u32(const MotorCanFrame *frame,
                             MotorRegisterValue *value)
{
    if (frame == 0 || value == 0 || frame->dlc != MOTOR_CAN_DLC ||
        frame->id < 0x581U || frame->id > 0x5ffU || frame->data[0] != 0x43U) {
        return -1;
    }

    value->motor_id = (uint8_t)(frame->id - 0x580U);
    value->address = (uint16_t)(((uint16_t)frame->data[1] << 8) |
                                frame->data[2]);
    value->value = ((uint32_t)frame->data[4] << 24) |
                   ((uint32_t)frame->data[5] << 16) |
                   ((uint32_t)frame->data[6] << 8) |
                   (uint32_t)frame->data[7];
    value->valid = 1U;
    return 0;
}
