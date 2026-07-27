#include "phytium_bmi088_port.h"

#include "sdkconfig.h"

#if defined(CONFIG_USE_SPI) && defined(CONFIG_USE_FSPIM) && defined(CONFIG_USE_GPIO) && defined(CONFIG_ENABLE_FGPIO)

#include "fio_mux.h"
#include "fsleep.h"
#include "fspim.h"
#include "fgpio.h"
#include "fgeneric_timer.h"
#include "ftypes.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifndef PHYTIUM_BMI088_SPI_ID
#define PHYTIUM_BMI088_SPI_ID FSPI0_ID
#endif

#ifndef PHYTIUM_BMI088_SPI_HZ
#define PHYTIUM_BMI088_SPI_HZ 1000000U
#endif

#ifndef PHYTIUM_BMI088_ACCEL_CS_GPIO_ID
#define PHYTIUM_BMI088_ACCEL_CS_GPIO_ID FGPIO_ID(FGPIO_CTRL_1, FGPIO_PIN_12)
#endif

#ifndef PHYTIUM_BMI088_GYRO_CS_GPIO_ID
#define PHYTIUM_BMI088_GYRO_CS_GPIO_ID FGPIO_ID(FGPIO_CTRL_4, FGPIO_PIN_12)
#endif

#ifndef PHYTIUM_BMI088_ACCEL_CS_GPIO_CTRL
#define PHYTIUM_BMI088_ACCEL_CS_GPIO_CTRL FGPIO_CTRL_1
#endif

#ifndef PHYTIUM_BMI088_ACCEL_CS_GPIO_PIN
#define PHYTIUM_BMI088_ACCEL_CS_GPIO_PIN FGPIO_PIN_12
#endif

#ifndef PHYTIUM_BMI088_GYRO_CS_GPIO_CTRL
#define PHYTIUM_BMI088_GYRO_CS_GPIO_CTRL FGPIO_CTRL_4
#endif

#ifndef PHYTIUM_BMI088_GYRO_CS_GPIO_PIN
#define PHYTIUM_BMI088_GYRO_CS_GPIO_PIN FGPIO_PIN_12
#endif

#define BMI088_ACCEL_CHIP_ID_REG 0x00U
#define BMI088_GYRO_CHIP_ID_REG  0x00U
#define BMI088_ACCEL_DATA_REG    0x12U
#define BMI088_GYRO_DATA_REG     0x02U
#define BMI088_ACCEL_CHIP_ID     0x1EU
#define BMI088_GYRO_CHIP_ID      0x0FU

#define BMI088_ACCEL_CONF_REG     0x40U
#define BMI088_ACCEL_RANGE_REG    0x41U
#define BMI088_ACCEL_PWR_CONF_REG 0x7CU
#define BMI088_ACCEL_PWR_CTRL_REG 0x7DU
#define BMI088_GYRO_RANGE_REG     0x0FU
#define BMI088_GYRO_BW_REG        0x10U
#define BMI088_GYRO_LPM1_REG      0x11U

#define BMI088_ACCEL_PWR_ENABLE   0x04U
#define BMI088_ACCEL_PM_ACTIVE    0x00U
#define BMI088_ACCEL_CONF_100HZ   0xA8U
#define BMI088_ACCEL_RANGE_6G     0x01U
#define BMI088_GYRO_PM_NORMAL     0x00U
#define BMI088_GYRO_BW_100HZ_32HZ 0x07U
#define BMI088_GYRO_RANGE_1000DPS 0x01U

#define BMI088_GRAVITY_M_S2 9.80665f
#define BMI088_PI 3.14159265358979323846f
#define BMI088_ACCEL_SCALE (6.0f * BMI088_GRAVITY_M_S2 / 32768.0f)
#define BMI088_GYRO_SCALE (1000.0f * BMI088_PI / 180.0f / 32768.0f)
#define BMI088_COMPLEMENTARY_ALPHA 0.98f

#ifndef PHYTIUM_BMI088_PITCH_DIRECTION
#define PHYTIUM_BMI088_PITCH_DIRECTION 1.0f
#endif

#ifndef PHYTIUM_BMI088_ROLL_DIRECTION
#define PHYTIUM_BMI088_ROLL_DIRECTION 1.0f
#endif

typedef enum {
    BMI088_DEV_ACCEL = 0,
    BMI088_DEV_GYRO = 1
} Bmi088Dev;

static FSpim g_spim;
static FGpio g_accel_cs;
static FGpio g_gyro_cs;
static uint8_t g_ready;
static PhytiumBmi088Sample g_sample;
static float g_gyro_bias[3];
static PhytiumBmi088DebugState g_dbg = {
    .init_ret = -99,
    .last_ret = -99,
};

static void bmi088_select(Bmi088Dev dev, boolean on)
{
    FGpio *cs = (dev == BMI088_DEV_ACCEL) ? &g_accel_cs : &g_gyro_cs;
    (void)FGpioSetOutputValue(cs, on ? FGPIO_PIN_LOW : FGPIO_PIN_HIGH);
}

static int bmi088_transfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    FError ret = FSpimTransferPollFifo(&g_spim, tx, rx, len);
    if (ret != FSPIM_SUCCESS) {
        g_dbg.last_ret = -2;
        return -2;
    }

    return 0;
}

static int bmi088_read_reg(Bmi088Dev dev, uint8_t reg, uint8_t *data, uint32_t len)
{
    uint8_t tx[16];
    uint8_t rx[16];
    uint32_t dummy = (dev == BMI088_DEV_ACCEL) ? 1U : 0U;
    uint32_t total = len + 1U + dummy;

    if (len == 0U || total > sizeof(tx)) {
        g_dbg.last_ret = -3;
        return -3;
    }

    memset(tx, 0xff, total);
    memset(rx, 0, total);
    tx[0] = (uint8_t)(reg | 0x80U);

    bmi088_select(dev, TRUE);
    int ret = bmi088_transfer(tx, rx, total);
    bmi088_select(dev, FALSE);

    if (ret != 0) {
        return ret;
    }

    memcpy(data, &rx[1U + dummy], len);
    g_dbg.last_ret = 0;
    return 0;
}

static int bmi088_write_reg(Bmi088Dev dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)(reg & 0x7fU);
    tx[1] = value;
    memset(rx, 0, sizeof(rx));

    bmi088_select(dev, TRUE);
    int ret = bmi088_transfer(tx, rx, sizeof(tx));
    bmi088_select(dev, FALSE);

    if (ret != 0) {
        return ret;
    }

    g_dbg.last_ret = 0;
    return 0;
}

static int bmi088_configure_sensors(void)
{
    int ret;

    ret = bmi088_write_reg(BMI088_DEV_ACCEL, BMI088_ACCEL_PWR_CTRL_REG, BMI088_ACCEL_PWR_ENABLE);
    if (ret != 0) return ret;
    fsleep_millisec(5);

    ret = bmi088_write_reg(BMI088_DEV_ACCEL, BMI088_ACCEL_PWR_CONF_REG, BMI088_ACCEL_PM_ACTIVE);
    if (ret != 0) return ret;
    fsleep_millisec(5);

    ret = bmi088_write_reg(BMI088_DEV_ACCEL, BMI088_ACCEL_CONF_REG, BMI088_ACCEL_CONF_100HZ);
    if (ret != 0) return ret;

    ret = bmi088_write_reg(BMI088_DEV_ACCEL, BMI088_ACCEL_RANGE_REG, BMI088_ACCEL_RANGE_6G);
    if (ret != 0) return ret;

    ret = bmi088_write_reg(BMI088_DEV_GYRO, BMI088_GYRO_LPM1_REG, BMI088_GYRO_PM_NORMAL);
    if (ret != 0) return ret;
    fsleep_millisec(30);

    ret = bmi088_write_reg(BMI088_DEV_GYRO, BMI088_GYRO_BW_REG, BMI088_GYRO_BW_100HZ_32HZ);
    if (ret != 0) return ret;

    ret = bmi088_write_reg(BMI088_DEV_GYRO, BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_1000DPS);
    if (ret != 0) return ret;

    fsleep_millisec(10);
    return 0;
}

int phytium_bmi088_init(void)
{
    FSpimConfig spim_cfg;
    FError ret;

    if (g_ready) {
        g_dbg.init_ret = 0;
        return 0;
    }

    /*
     * Do not call FIOMuxInit() here. Servo/CAN may already have configured
     * their pads, and reinitializing the global IOMUX late can disturb PWM
     * output. Only claim the SPI and CS pads used by BMI088.
     */
    FIOPadSetSpimMux(PHYTIUM_BMI088_SPI_ID);
    FIOPadSetGpioMux(PHYTIUM_BMI088_ACCEL_CS_GPIO_CTRL, PHYTIUM_BMI088_ACCEL_CS_GPIO_PIN);
    FIOPadSetGpioMux(PHYTIUM_BMI088_GYRO_CS_GPIO_CTRL, PHYTIUM_BMI088_GYRO_CS_GPIO_PIN);

    const FGpioConfig *accel_cs_cfg = FGpioLookupConfig(PHYTIUM_BMI088_ACCEL_CS_GPIO_ID);
    const FGpioConfig *gyro_cs_cfg = FGpioLookupConfig(PHYTIUM_BMI088_GYRO_CS_GPIO_ID);
    if (accel_cs_cfg == NULL || gyro_cs_cfg == NULL) {
        g_dbg.init_ret = -5;
        return -5;
    }

    if (FGpioCfgInitialize(&g_accel_cs, accel_cs_cfg) != FGPIO_SUCCESS ||
        FGpioCfgInitialize(&g_gyro_cs, gyro_cs_cfg) != FGPIO_SUCCESS) {
        g_dbg.init_ret = -6;
        return -6;
    }

    FGpioSetDirection(&g_accel_cs, FGPIO_DIR_OUTPUT);
    FGpioSetDirection(&g_gyro_cs, FGPIO_DIR_OUTPUT);
    (void)FGpioSetOutputValue(&g_accel_cs, FGPIO_PIN_HIGH);
    (void)FGpioSetOutputValue(&g_gyro_cs, FGPIO_PIN_HIGH);

    const FSpimConfig *base_cfg = FSpimLookupConfig(PHYTIUM_BMI088_SPI_ID);
    if (base_cfg == NULL) {
        g_dbg.init_ret = -1;
        return -1;
    }

    spim_cfg = *base_cfg;
    spim_cfg.en_test = FALSE;
    spim_cfg.en_dma = FALSE;
    spim_cfg.slave_dev_id = FSPIM_SLAVE_DEV_0;
    spim_cfg.cpol = FSPIM_CPOL_HIGH;
    spim_cfg.cpha = FSPIM_CPHA_2_EDGE;
    spim_cfg.n_bytes = FSPIM_1_BYTE;
    spim_cfg.sclk_hz = PHYTIUM_BMI088_SPI_HZ;
    spim_cfg.trans_way = TRANS_WAY_POLL;

    ret = FSpimCfgInitialize(&g_spim, &spim_cfg);
    if (ret != FSPIM_SUCCESS) {
        g_dbg.init_ret = -2;
        return -2;
    }

    ret = FSpimSetOption(&g_spim, FSPIM_FREQUENCY_OPTION, PHYTIUM_BMI088_SPI_HZ);
    if (ret != FSPIM_SUCCESS) {
        g_dbg.init_ret = -3;
        return -3;
    }

    FSpimSetChipSelection(&g_spim, FALSE);
    fsleep_millisec(10);

    (void)bmi088_read_reg(BMI088_DEV_ACCEL, BMI088_ACCEL_CHIP_ID_REG, &g_dbg.accel_chip_id, 1);
    (void)bmi088_read_reg(BMI088_DEV_GYRO, BMI088_GYRO_CHIP_ID_REG, &g_dbg.gyro_chip_id, 1);

    if (g_dbg.accel_chip_id != BMI088_ACCEL_CHIP_ID ||
        g_dbg.gyro_chip_id != BMI088_GYRO_CHIP_ID) {
        g_dbg.init_ret = -4;
        return -4;
    }

    ret = bmi088_configure_sensors();
    if (ret != 0) {
        g_dbg.init_ret = -7;
        return -7;
    }

    g_ready = 1;
    g_dbg.init_ret = 0;
    return 0;
}

int phytium_bmi088_read_sample(void)
{
    uint8_t buf[6];
    int ret = phytium_bmi088_init();
    if (ret != 0) {
        g_dbg.last_ret = (int8_t)ret;
        return ret;
    }

    ret = bmi088_read_reg(BMI088_DEV_ACCEL, BMI088_ACCEL_DATA_REG, buf, sizeof(buf));
    if (ret != 0) {
        return ret;
    }
    g_dbg.accel_raw[0] = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    g_dbg.accel_raw[1] = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    g_dbg.accel_raw[2] = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

    ret = bmi088_read_reg(BMI088_DEV_GYRO, BMI088_GYRO_DATA_REG, buf, sizeof(buf));
    if (ret != 0) {
        return ret;
    }
    g_dbg.gyro_raw[0] = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    g_dbg.gyro_raw[1] = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    g_dbg.gyro_raw[2] = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    g_dbg.read_count++;
    g_dbg.last_ret = 0;

    return 0;
}

int phytium_bmi088_calibrate_gyro_serviced(
    uint16_t sample_count,
    PhytiumBmi088CalibrationService service,
    void *context)
{
    float sum[3] = {0.0f, 0.0f, 0.0f};

    if (sample_count == 0U) {
        return -1;
    }

    memset(&g_sample, 0, sizeof(g_sample));
    for (uint16_t i = 0; i < sample_count; ++i) {
        int ret = phytium_bmi088_read_sample();
        if (ret != 0) {
            return ret;
        }
        for (int axis = 0; axis < 3; ++axis) {
            sum[axis] += (float)g_dbg.gyro_raw[axis] * BMI088_GYRO_SCALE;
        }
        if (service != NULL) {
            service(context);
        }
        fsleep_millisec(10);
    }

    for (int axis = 0; axis < 3; ++axis) {
        g_gyro_bias[axis] = sum[axis] / (float)sample_count;
    }
    g_sample.calibrated = 1U;
    return 0;
}

int phytium_bmi088_calibrate_gyro(uint16_t sample_count)
{
    return phytium_bmi088_calibrate_gyro_serviced(sample_count, NULL, NULL);
}

int phytium_bmi088_update(float dt_s)
{
    float roll_acc;
    float roll_gyro;
    float pitch_acc;
    float pitch_gyro;
    uint8_t was_valid = g_sample.valid;
    uint8_t was_calibrated = g_sample.calibrated;
    int ret;

    if (!(dt_s > 0.0f) || dt_s > 0.1f) {
        return -1;
    }

    ret = phytium_bmi088_read_sample();
    if (ret != 0) {
        g_sample.valid = 0U;
        return ret;
    }

    for (int i = 0; i < 3; ++i) {
        g_sample.accel_m_s2[i] = (float)g_dbg.accel_raw[i] * BMI088_ACCEL_SCALE;
        g_sample.gyro_rad_s[i] = (float)g_dbg.gyro_raw[i] * BMI088_GYRO_SCALE;
    }

    g_sample.roll_rate_rad_s = PHYTIUM_BMI088_ROLL_DIRECTION *
                               (g_sample.gyro_rad_s[0] - g_gyro_bias[0]);
    roll_acc = PHYTIUM_BMI088_ROLL_DIRECTION *
               atan2f(g_sample.accel_m_s2[1], g_sample.accel_m_s2[2]);
    roll_gyro = g_sample.roll_rad + g_sample.roll_rate_rad_s * dt_s;
    g_sample.roll_rad = was_valid ?
        BMI088_COMPLEMENTARY_ALPHA * roll_gyro +
        (1.0f - BMI088_COMPLEMENTARY_ALPHA) * roll_acc : roll_acc;

    g_sample.pitch_rate_rad_s = PHYTIUM_BMI088_PITCH_DIRECTION *
                                (g_sample.gyro_rad_s[1] - g_gyro_bias[1]);
    pitch_acc = PHYTIUM_BMI088_PITCH_DIRECTION *
                atan2f(-g_sample.accel_m_s2[0],
                       sqrtf(g_sample.accel_m_s2[1] * g_sample.accel_m_s2[1] +
                             g_sample.accel_m_s2[2] * g_sample.accel_m_s2[2]));
    pitch_gyro = g_sample.pitch_rad + g_sample.pitch_rate_rad_s * dt_s;
    g_sample.pitch_rad = was_valid ?
        BMI088_COMPLEMENTARY_ALPHA * pitch_gyro +
        (1.0f - BMI088_COMPLEMENTARY_ALPHA) * pitch_acc : pitch_acc;
    g_sample.update_tick = GenericTimerRead(GENERIC_TIMER_ID0);
    g_sample.calibrated = was_calibrated;
    g_sample.valid = 1U;
    return 0;
}

int phytium_bmi088_get_sample(PhytiumBmi088Sample *sample)
{
    if (sample == NULL || !g_sample.valid) {
        return -1;
    }
    *sample = g_sample;
    return 0;
}

const PhytiumBmi088DebugState *phytium_bmi088_get_debug_state(void)
{
    return &g_dbg;
}

#else

static PhytiumBmi088DebugState g_dbg = {
    .init_ret = -98,
    .last_ret = -98,
};

int phytium_bmi088_init(void)
{
    g_dbg.init_ret = -98;
    g_dbg.last_ret = -98;
    return -98;
}

int phytium_bmi088_read_sample(void)
{
    g_dbg.last_ret = -98;
    return -98;
}

int phytium_bmi088_calibrate_gyro(uint16_t sample_count)
{
    (void)sample_count;
    return -98;
}

int phytium_bmi088_calibrate_gyro_serviced(
    uint16_t sample_count,
    PhytiumBmi088CalibrationService service,
    void *context)
{
    (void)sample_count;
    (void)service;
    (void)context;
    return -98;
}

int phytium_bmi088_update(float dt_s)
{
    (void)dt_s;
    return -98;
}

int phytium_bmi088_get_sample(PhytiumBmi088Sample *sample)
{
    (void)sample;
    return -98;
}

const PhytiumBmi088DebugState *phytium_bmi088_get_debug_state(void)
{
    return &g_dbg;
}

#endif
