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


/**
 * @brief  Initializes the Data Watchpoint and Trace (DWT) unit to enable cycle counting.
 * 
 * This function enables the DWT cycle counter, which is part of the ARM Cortex-M's 
 * debugging and tracing features. The DWT cycle counter can be used to measure the 
 * number of CPU cycles elapsed, which is helpful for profiling code performance 
 * and timing analysis.
 */
void DWT_Init(void) 
{
    // Check if the DWT (Data Watchpoint and Trace) unit is already enabled
    // CoreDebug->DEMCR contains the Debug Exception and Monitor Control Register
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) 
    {
        // Enable the DWT unit by setting the TRCENA (Trace Enable) bit in DEMCR
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

        // Reset the cycle counter (CYCCNT) to 0 to start counting from zero
        DWT->CYCCNT = 0;

        // Enable the DWT cycle counter by setting the CYCCNTENA bit in the DWT control register
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}


uint32_t micros(void)
{
    return DWT->CYCCNT / (CORE_CLOCK_HZ / 1000000U);
}




