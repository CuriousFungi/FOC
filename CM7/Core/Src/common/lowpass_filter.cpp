#include "lowpass_filter.hpp"
#include <cmath>

LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
    , initialized(false)
{
    timestamp_prev = _micros();
}


float LowPassFilter::operator() (float x)
{
    const float SECONDS_PER_MICROSECOND(0.000001f);
    unsigned long timestamp = _micros();
 
    float dt = static_cast<float>(timestamp - timestamp_prev)
             * SECONDS_PER_MICROSECOND;
#if 0
    if (dt < 0.0f )
    {
        dt = 1e-3f;
    }
    else if(dt > 0.3f)
    {
        y_prev         = x;
        timestamp_prev = timestamp;
        return x;
    }
#endif

    // Check if y_prev is uninitialized (NAN) and initialize it with the first value
     if (!initialized)
     {
         y_prev = x;  // Set the previous value to the first input value
         initialized = true;
     }


    // enforce a minimum dt
    float alpha = Tf/(Tf + fmax(dt, 1e-6f));
    float y     = (alpha * y_prev) + (1.0f - alpha) * x;
    
    y_prev         = y;
    timestamp_prev = timestamp;
    
    return y;
}
