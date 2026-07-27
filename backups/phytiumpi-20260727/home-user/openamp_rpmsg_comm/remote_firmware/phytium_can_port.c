#include "phytium_can_port.h"

#include "sdkconfig.h"

#if defined(CONFIG_USE_CAN) && defined(CONFIG_USE_FCAN)

#include "fcan.h"
#include "fio_mux.h"
#include "fcan_hw.h"
#include "fparameters.h"
#include "fgeneric_timer.h"
#include "ftypes.h"
#include <stdio.h>
#include <string.h>

#ifndef PHYTIUM_CAN_ID
#define PHYTIUM_CAN_ID FCAN1_ID
#endif

#ifndef PHYTIUM_CAN_BAUDRATE
#define PHYTIUM_CAN_BAUDRATE 1000000U
#endif

#define PHYTIUM_CAN0_BASE 0x2800A000U
#define PHYTIUM_CAN1_BASE 0x2800B000U
#define PHYTIUM_CAN_CTRL_OFFSET 0x000U
#define PHYTIUM_CAN_INTR_OFFSET 0x004U
#define PHYTIUM_CAN_XFER_STS_OFFSET 0x030U
#define PHYTIUM_CAN_ERR_CNT_OFFSET 0x034U
#define PHYTIUM_CAN_FIFO_CNT_OFFSET 0x038U
#define PHYTIUM_CAN_XFER_EN_OFFSET 0x040U

static FCanCtrl g_can;
static int g_can_ready = 0;
static MotorFeedback g_motor_feedback[128];
static MotorRegisterValue g_register_value[128];
static PhytiumCanDebugState g_can_debug = {
    .init_ret = -99,
    .last_send_ret = -99,
    .last_receive_ret = -99,
    .can_id = PHYTIUM_CAN_ID,
    .baudrate = PHYTIUM_CAN_BAUDRATE,
};

static uintptr phytium_can_base(void)
{
    return PHYTIUM_CAN_ID == FCAN1_ID ? PHYTIUM_CAN1_BASE : PHYTIUM_CAN0_BASE;
}

static uint32_t phytium_can_read_reg(uint32_t offset)
{
    return *(volatile uint32_t *)(phytium_can_base() + offset);
}

static void phytium_can_update_regs(void)
{
    g_can_debug.reg_ctrl = phytium_can_read_reg(PHYTIUM_CAN_CTRL_OFFSET);
    g_can_debug.reg_intr = phytium_can_read_reg(PHYTIUM_CAN_INTR_OFFSET);
    g_can_debug.reg_xfer_sts = phytium_can_read_reg(PHYTIUM_CAN_XFER_STS_OFFSET);
    g_can_debug.reg_err_cnt = phytium_can_read_reg(PHYTIUM_CAN_ERR_CNT_OFFSET);
    g_can_debug.reg_fifo_cnt = phytium_can_read_reg(PHYTIUM_CAN_FIFO_CNT_OFFSET);
    g_can_debug.reg_xfer_en = phytium_can_read_reg(PHYTIUM_CAN_XFER_EN_OFFSET);
}

const PhytiumCanDebugState *phytium_can_get_debug_state(void)
{
    if (g_can_ready) {
        phytium_can_update_regs();
    }
    return &g_can_debug;
}

int phytium_can_init(void)
{
    FError ret;
    FCanBaudrateConfig arb_segment_config;
    FCanIdMaskConfig id_mask;

    if (g_can_ready) {
        g_can_debug.init_ret = 0;
        return 0;
    }

    memset(&g_can, 0, sizeof(g_can));
    g_can_debug.can_id = PHYTIUM_CAN_ID;
    g_can_debug.baudrate = PHYTIUM_CAN_BAUDRATE;

    printf("phytium_can_init: can_id=%u baudrate=%u\r\n",
           (unsigned)PHYTIUM_CAN_ID,
           (unsigned)PHYTIUM_CAN_BAUDRATE);

    FIOMuxInit();
    printf("phytium_can_init: FIOMuxInit done\r\n");

    FIOPadSetCanMux(PHYTIUM_CAN_ID);
    printf("phytium_can_init: FIOPadSetCanMux done\r\n");

    ret = FCanCfgInitialize(&g_can, FCanLookupConfig(PHYTIUM_CAN_ID));
    if (ret != FCAN_SUCCESS) {
        g_can_debug.init_ret = -1;
        printf("phytium_can_init: FCanCfgInitialize failed ret=%d\r\n", ret);
        return -1;
    }
    printf("phytium_can_init: FCanCfgInitialize ok\r\n");

    FCanFdEnable(&g_can, FALSE);
    FCanSetMode(&g_can, FCAN_PROBE_NORMAL_MODE);

    memset(&arb_segment_config, 0, sizeof(arb_segment_config));

    arb_segment_config.baudrate = PHYTIUM_CAN_BAUDRATE;
    /*
     * Match the Linux SocketCAN timing that has already moved the motor:
     * bitrate 1000000, sample-point 0.750, brp 10, prop 7,
     * phase_seg1 7, phase_seg2 5, sjw 2, CAN clock 200MHz.
     */
    arb_segment_config.auto_calc = FALSE;
    arb_segment_config.segment = FCAN_ARB_SEGMENT;
    arb_segment_config.sample_point = 750;
    arb_segment_config.prop_seg = 7;
    arb_segment_config.phase_seg1 = 7;
    arb_segment_config.phase_seg2 = 5;
    arb_segment_config.sjw = 2;
    arb_segment_config.brp = 10;

    ret = FCanBaudrateSet(&g_can, &arb_segment_config);
    if (ret != FCAN_SUCCESS) {
        g_can_debug.init_ret = -2;
        printf("phytium_can_init: FCanBaudrateSet arb failed ret=%d\r\n", ret);
        return -2;
    }
    printf("phytium_can_init: FCanBaudrateSet arb ok\r\n");

    memset(&id_mask, 0, sizeof(id_mask));
    for (int i = 0; i < FCAN_ACC_ID_REG_NUM; ++i) {
        id_mask.filter_index = i;
        id_mask.id = 0U;
        id_mask.mask = FCAN_ACC_IDN_MASK;
        id_mask.type = STANDARD_FRAME;
        ret = FCanIdMaskFilterSet(&g_can, &id_mask);
        if (ret != FCAN_SUCCESS) {
            g_can_debug.init_ret = -3;
            printf("phytium_can_init: filter setup failed ret=%d\r\n", ret);
            return -3;
        }
    }
    FCanIdMaskFilterEnable(&g_can);

    FCanEnable(&g_can, TRUE);
    printf("phytium_can_init: FCanEnable done\r\n");

    g_can_ready = 1;
    g_can_debug.init_ret = 0;
    phytium_can_update_regs();
    return 0;
}

int phytium_can_send(const MotorCanFrame *frame)
{
    FError ret;
    FCanFrame send_frame;

    if (!frame) {
        g_can_debug.last_send_ret = -1;
        return -1;
    }

    if (!g_can_ready) {
        ret = phytium_can_init();
        if (ret != 0) {
            g_can_debug.last_send_ret = -2;
            return -2;
        }
    }

    memset(&send_frame, 0, sizeof(send_frame));

    send_frame.canid = frame->id & CAN_SFF_MASK;
    send_frame.candlc = frame->dlc;
    for (int i = 0; i < frame->dlc && i < 8; ++i) {
        send_frame.data[i] = frame->data[i];
    }

    g_can_debug.last_frame_id = send_frame.canid;
    g_can_debug.last_frame_dlc = send_frame.candlc;
    for (int i = 0; i < 8; ++i) {
        g_can_debug.last_frame_data[i] = send_frame.data[i];
    }

    ret = FCanSend(&g_can, &send_frame);
    if (ret != FCAN_SUCCESS) {
        g_can_debug.last_send_ret = -3;
        phytium_can_update_regs();
        printf("phytium_can_send: FCanSend failed ret=%d\r\n", ret);
        return -3;
    }

    g_can_debug.last_send_ret = 0;
    g_can_debug.send_count++;
    phytium_can_update_regs();
    return 0;
}

int phytium_can_poll(void)
{
    int received = 0;

    if (!g_can_ready && phytium_can_init() != 0) {
        g_can_debug.last_receive_ret = -2;
        return -2;
    }

    while (received < 16 && !FCAN_RX_FIFO_EMPTY(g_can.config.base_address)) {
        FCanFrame rx_frame;
        MotorCanFrame frame;
        MotorFeedback feedback;
        MotorRegisterValue register_value;
        FError ret = FCanRecv(&g_can, &rx_frame);

        if (ret != FCAN_SUCCESS) {
            g_can_debug.last_receive_ret = -3;
            return received > 0 ? received : -3;
        }

        frame.id = rx_frame.canid & CAN_SFF_MASK;
        frame.dlc = rx_frame.candlc > MOTOR_CAN_DLC ? MOTOR_CAN_DLC : rx_frame.candlc;
        memset(frame.data, 0, sizeof(frame.data));
        memcpy(frame.data, rx_frame.data, frame.dlc);
        if (motor_parse_feedback(&frame, &feedback) == 0 &&
            feedback.motor_id < (uint8_t)(sizeof(g_motor_feedback) / sizeof(g_motor_feedback[0]))) {
            feedback.update_tick = GenericTimerRead(GENERIC_TIMER_ID0);
            g_motor_feedback[feedback.motor_id] = feedback;
            g_can_debug.feedback_count++;
        }
        if (motor_parse_register_u32(&frame, &register_value) == 0 &&
            register_value.motor_id <
                (uint8_t)(sizeof(g_register_value) /
                          sizeof(g_register_value[0]))) {
            register_value.update_tick = GenericTimerRead(GENERIC_TIMER_ID0);
            g_register_value[register_value.motor_id] = register_value;
        }
        received++;
        g_can_debug.receive_count++;
    }

    g_can_debug.last_receive_ret = 0;
    return received;
}

int phytium_can_get_motor_feedback(uint8_t motor_id, MotorFeedback *feedback)
{
    if (feedback == NULL || motor_id >=
        (uint8_t)(sizeof(g_motor_feedback) / sizeof(g_motor_feedback[0])) ||
        !g_motor_feedback[motor_id].valid) {
        return -1;
    }

    *feedback = g_motor_feedback[motor_id];
    return 0;
}

void phytium_can_clear_motor_feedback(uint8_t motor_id)
{
    if (motor_id < (uint8_t)(sizeof(g_motor_feedback) /
                             sizeof(g_motor_feedback[0]))) {
        memset(&g_motor_feedback[motor_id], 0,
               sizeof(g_motor_feedback[motor_id]));
    }
}

void phytium_can_clear_register_value(uint8_t motor_id)
{
    if (motor_id < (uint8_t)(sizeof(g_register_value) /
                             sizeof(g_register_value[0]))) {
        memset(&g_register_value[motor_id], 0,
               sizeof(g_register_value[motor_id]));
    }
}

int phytium_can_get_register_value(uint8_t motor_id,
                                   MotorRegisterValue *value)
{
    if (value == NULL || motor_id >=
        (uint8_t)(sizeof(g_register_value) / sizeof(g_register_value[0])) ||
        !g_register_value[motor_id].valid) {
        return -1;
    }

    *value = g_register_value[motor_id];
    return 0;
}

int phytium_can_bus_ok(void)
{
    uint32_t tx_err;

    if (!g_can_ready) {
        return 0;
    }

    phytium_can_update_regs();
    tx_err = FCAN_ERR_CNT_TFN_GET(g_can_debug.reg_err_cnt);

    return tx_err < 128U && (g_can_debug.reg_ctrl & 0x1U) != 0U;
}

#else

static PhytiumCanDebugState g_can_debug = {
    .init_ret = -98,
    .last_send_ret = -98,
    .last_receive_ret = -98,
    .can_id = 1,
    .baudrate = 1000000U,
};

int phytium_can_init(void)
{
    g_can_debug.init_ret = -98;
    return -98;
}

int phytium_can_send(const MotorCanFrame *frame)
{
    if (frame) {
        g_can_debug.last_frame_id = frame->id;
        g_can_debug.last_frame_dlc = frame->dlc;
        for (int i = 0; i < 8; ++i) {
            g_can_debug.last_frame_data[i] = frame->data[i];
        }
    }
    g_can_debug.last_send_ret = -98;
    return -98;
}

int phytium_can_poll(void)
{
    g_can_debug.last_receive_ret = -98;
    return -98;
}

int phytium_can_get_motor_feedback(uint8_t motor_id, MotorFeedback *feedback)
{
    (void)motor_id;
    (void)feedback;
    return -98;
}

void phytium_can_clear_motor_feedback(uint8_t motor_id)
{
    (void)motor_id;
}

void phytium_can_clear_register_value(uint8_t motor_id)
{
    (void)motor_id;
}

int phytium_can_get_register_value(uint8_t motor_id,
                                   MotorRegisterValue *value)
{
    (void)motor_id;
    (void)value;
    return -98;
}

const PhytiumCanDebugState *phytium_can_get_debug_state(void)
{
    return &g_can_debug;
}

int phytium_can_bus_ok(void)
{
    return 0;
}

#endif
