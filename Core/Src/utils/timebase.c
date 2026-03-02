#include "utils/timebase.h"
#include "stm32f4xx.h"

void timebase_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t timebase_cycles(void)
{
  return DWT->CYCCNT;
}

uint32_t timebase_cycles_to_us(uint32_t cyc)
{
  // 84 MHz => 84 cycles per microsecond
  return cyc / 84u;
}
