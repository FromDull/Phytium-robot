#ifndef PHYTIUM_BMI088_PORT_H
#define PHYTIUM_BMI088_PORT_H

#include <stdint.h>

typedef struct {
    int8_t init_ret;
    int8_t last_ret;
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    uint32_t read_count;
} PhytiumBmi088DebugState;

typedef struct {
    float accel_m_s2[3];
    float gyro_rad_s[3];
    float roll_rad;
    float roll_rate_rad_s;
    float pitch_rad;
    float pitch_rate_rad_s;
    uint64_t update_tick;
    uint8_t valid;
    uint8_t calibrated;
} PhytiumBmi088Sample;

typedef void (*PhytiumBmi088CalibrationService)(void *context);

int phytium_bmi088_init(void);
int phytium_bmi088_read_sample(void);
int phytium_bmi088_calibrate_gyro(uint16_t sample_count);
int phytium_bmi088_calibrate_gyro_serviced(
    uint16_t sample_count,
    PhytiumBmi088CalibrationService service,
    void *context);
int phytium_bmi088_update(float dt_s);
int phytium_bmi088_get_sample(PhytiumBmi088Sample *sample);
const PhytiumBmi088DebugState *phytium_bmi088_get_debug_state(void);

#endif
