  
#ifndef PID_ALGORITHM_HPP
#define PID_ALGORITHM_HPP

// Based on CMSIS arm_pid_f32
// https://www.keil.com/pack/doc/CMSIS/DSP/html/group__PID.html#ga5a6865ed706b7dd969ef0bd58a61f306
  

#include <stdint.h>
#include <string.h>
#include <math.h>



//=============================================================================
//                             PID_algorithm
//=============================================================================
template <typename VAR_TYPE, int NUM_PID_STATES>
class PID_algorithm
{
public:

    explicit PID_algorithm( VAR_TYPE Kp,
                            VAR_TYPE Ki,
                            VAR_TYPE Kd,
                            VAR_TYPE rate_limit,
                            VAR_TYPE integral_limit)
             :   KP(Kp)
             ,   KI(Ki)
             ,   KD(Kd)
             ,   A0( KP + KI + KD )
             ,   A1( -KP -(2.0f * KD))
             ,   A2( KD)
             ,   INTEGRAL_LIMIT(integral_limit)
             {
                 reset();
             }

            ~PID_algorithm(){};           

    //------------------------------------------------------------------------- 
    //                            reset
    //------------------------------------------------------------------------- 
    void reset()
    {
        memset( m_state, 0, NUM_PID_STATES * sizeof(VAR_TYPE) );
    } 
 
  
    //------------------------------------------------------------------------- 
    //                            update
    //-------------------------------------------------------------------------
    VAR_TYPE update(float ref_error)
    {
        // Algorithm:  y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
    
        VAR_TYPE output = ref_error  * A0
                        + m_state[0] * A1
                        + m_state[1] * A2
                        + m_state[2];
        
         m_state[1] = m_state[0];
         m_state[0] = ref_error;
         m_state[2] = output; 
             
     return output;
}


private:
  
    const VAR_TYPE      KP;      /// The proportional gain
    const VAR_TYPE      KI;      /// The integral gain
    const VAR_TYPE      KD;      //  The derivative gain

    const VAR_TYPE      A0;      /// The derived gain, A0 = Kp + Ki + Kd
    const VAR_TYPE      A1;      /// The derived gain, A1 = -Kp - 2Kd
    const VAR_TYPE      A2;      /// The derived gain, A2 = Kd

    const VAR_TYPE      INTEGRAL_LIMIT;

    VAR_TYPE            m_state[NUM_PID_STATES];
};

#endif // inclusion guard
