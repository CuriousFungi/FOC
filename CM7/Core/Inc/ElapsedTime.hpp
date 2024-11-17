#ifndef ELAPSED_TIME_HPP
#define ELAPSED_TIME_HPP

#include <stdio.h>

#include "time_utils.hpp"


class ElapsedTime
{
public:
    explicit ElapsedTime()
    :  m_prev_time(micros())
    {
    }

    ~ElapsedTime(){}

    uint32_t get()
    {
         uint32_t curr_time = micros();
         uint32_t diff_time = 0;

         if(curr_time >= m_prev_time)
         {
            diff_time = curr_time - m_prev_time;
         }
         else
         {
            diff_time = (0xFFFFFFFF - m_prev_time) + curr_time + 1;
         }

         m_prev_time = curr_time;

         return diff_time;
    }

private:

    uint32_t m_prev_time;
};

#endif // Inclusion guard
