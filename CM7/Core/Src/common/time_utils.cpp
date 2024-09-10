#include "time_utils.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
}
#endif



// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms)
{
  HAL_Delay(ms);
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){

//return 1000 * HAL_GetTick(); // TODO: setup another timer to get microsec

//uint32_t cycleCount = DWT->CYCCNT;
//uint32_t clockFrequency = 84000000;
//uint32_t microseconds = (cycleCount * 1000000) / clockFrequency;

// HAL_RCC_GetHCLKFreq()


return DWT->CYCCNT / 84;

}
