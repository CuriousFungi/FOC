#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <stdint.h>

#include "foc_utils.hpp"

#define CORE_CLOCK_HZ 400000000U  // 400 MHz for STM32H755

#ifdef __cplusplus
 extern "C" {
#endif

     // Function implementing delay() function in milliseconds 
     // - blocking function
     // - hardware specific
     //
     // @param ms number of milliseconds to wait
    void _delay(unsigned long ms);

     // Function implementing timestamp getting function in microseconds
     // hardware specific
    unsigned long _micros();


    void DWT_Init(void);
    uint32_t micros(void);

#ifdef __cplusplus
 } // extern "C" {
#endif



#endif
