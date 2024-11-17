#ifndef CRITICAL_REGION_HPP
#define CRITICAL_REGION_HPP

#include <stdint.h>
#include "core_cm7.h"

class CriticalRegion
{
public:
    explicit CriticalRegion()
    :  m_prime_mask(0UL)
    {
        ;
    }

    ~CriticalRegion()
    {
        ;
    }

    void enter()
    {
        m_prime_mask = __get_PRIMASK();
        __disable_irq();
    }

    void exit()
    {
        __set_PRIMASK(m_prime_mask);
        __enable_irq();
    }

private:
    uint32_t m_prime_mask;

};


#endif // Inclusion guard
