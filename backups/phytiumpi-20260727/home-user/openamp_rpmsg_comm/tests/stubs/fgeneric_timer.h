#ifndef FGENERIC_TIMER_H
#define FGENERIC_TIMER_H

#include <stdint.h>

#define GENERIC_TIMER_ID0 0U

uint64_t GenericTimerRead(uint32_t timer_id);
uint64_t GenericTimerFrequecy(void);

#endif
