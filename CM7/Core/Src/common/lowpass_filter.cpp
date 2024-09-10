#include "lowpass_filter.hpp"

LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
{
    timestamp_prev = _micros();
}


float LowPassFilter::operator() (float x)
{
    const float SECONDS_PER_MICROSECOND(0.000001f);
    unsigned long timestamp = _micros();
 
    float dt = static_cast<float>(timestamp - timestamp_prev)
             * SECONDS_PER_MICROSECOND;

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

    float alpha = Tf/(Tf + dt);
    float y     = (alpha * y_prev) + (1.0f - alpha) * x;
    
    y_prev         = y;
    timestamp_prev = timestamp;
    
    return y;
}
