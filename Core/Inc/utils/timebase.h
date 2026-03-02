#pragma once
#include <stdint.h>

void timebase_init(void);
uint32_t timebase_cycles(void);                 // raw CYCCNT
uint32_t timebase_cycles_to_us(uint32_t cyc);   // convert delta cycles -> us
