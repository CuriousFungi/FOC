// TODO: move  determine_sensor_direction and align sensor into higher level classes



#include <limits>
#include <math.h>

#include "foc_utils.hpp"
#include "AS5048A.hpp"
#include "EnumValue.hpp"
#include "clamp.hpp"
#include "ElapsedTime.hpp"

#include "StepperMotor.hpp"


extern "C" 
{
    extern void update_buffers(uint16_t new_angle, uint32_t new_timestamp);
    extern float calculate_velocity_from_buffer(void);
}


volatile float g_shaft_angle = 0.0f;
volatile float g_mag_flux_linkage_q = 0.0f;

extern volatile float g_as5048_angle;
extern volatile float g_as5048_velocity;
volatile float g_electrical_a(0.0f);
volatile float g_electrical_b(0.0f);
volatile float g_electrical_c(0.0f);
volatile float g_electrical_rad_ref(0.0f);
volatile float g_electrical_rad_cmd(0.0f);
volatile float g_velocity_correction(0.0f);
volatile float g_back_emf_q_axis(0.0);

volatile float g_dutycycle_1A(0.0f);
volatile float g_dutycycle_1B(0.0f);
volatile float g_dutycycle_2A(0.0f);
volatile float g_dutycycle_2b(0.0f);

// Debug variables for D6 (PC7, TIM8_CH2) issue
volatile uint32_t g_debug_ccr2b_written = 0;  // CCR value being written for phase_2B (D6)
volatile uint32_t g_debug_ccr2b_actual = 0;   // Actual TIM8->CCR2 register value
volatile float g_debug_duty2b = 0.0f;         // Duty cycle parameter for phase_2B (D6)
volatile bool g_debug_cc2ne_enabled = false;  // Complementary channel enabled flag
volatile uint32_t g_debug_ccer_value = 0;     // Full CCER register value
volatile uint32_t g_debug_trigger = 0;        // Set to 1 to trigger inspection point
volatile float g_debug_U_beta = 0.0f;         // U_beta value before clamping
volatile float g_debug_U_beta_clamped = 0.0f; // U_beta value after clamping

// Debug variables for D7 (PC6, TIM8_CH1) for comparison
volatile uint32_t g_debug_ccr2a_written = 0;  // CCR value being written for phase_2A (D7)
volatile uint32_t g_debug_ccr2a_actual = 0;   // Actual TIM8->CCR1 register value
volatile float g_debug_duty2a = 0.0f;

// Additional debug variables for duty cycle calculation
volatile bool g_debug_U_beta_positive = false;  // Whether U_beta >= 0 (true) or < 0 (false)
volatile float g_debug_duty_cycle_beta = 0.0f;   // duty_cycle_beta value before factors
volatile float g_debug_hifactor_2 = 0.0f;        // hifactor_2 value
volatile float g_debug_lofactor_2 = 0.0f;         // lofactor_2 value         // Duty cycle parameter for phase_2A (D7)

volatile float g_winding_amps_a(0.0f);
volatile float g_winding_amps_b(0.0f);
volatile float g_current_offset_a(0.0f);
volatile float g_current_offset_b(0.0f);

volatile float g_computed_inductance(0.0f);
volatile float g_amperage_q(0.0f);

volatile float g_pre_clamp_v(0.0f);
volatile float g_post_clamp_v(0.0f);
volatile float g_radian_advance_electrical(0.0f);
//volatile unsigned long g_us(0);

volatile float g_target_rad_per_sec(0.0f);
volatile float g_ramped_target_rps(0.0f);
volatile float g_accumulated_mech_rad(0.0f);
volatile float g_angle_before_add(0.0f);
volatile float g_angle_increment(0.0f);
volatile float g_angle_after_add(0.0f);
volatile float g_mech_angle_for_foc(0.0f);
volatile float g_elec_angle_for_foc(0.0f);
volatile uint32_t g_loop_counter(0);
volatile uint32_t g_loopfoc_counter(0);
volatile uint32_t g_speed_update_counter(0);

volatile float g_filtered_angle(0.0f);
volatile float g_filtered_velocity(0.0f);
volatile float g_filtered_back_emf(0.0f);
volatile unsigned long g_microseconds(0);
volatile float g_raw_velocity(0.0f);
volatile float g_raw_angle(0.0f);
volatile float g_delta_angle(0.0f);
volatile float g_new_shaft_angle(0.0f);
volatile float g_new_rad_per_sec(0.0f);
volatile int32_t g_new_int_velocity(0.0f);
volatile uint16_t g_count(0);
volatile float g_new_velocity(0.0f);



volatile uint16_t g_sample_count(0);
volatile float    g_sample_radians(0.0f);
volatile float    g_sample_radians_per_second(0.0f);

volatile float    g_voltage_q(0.0f);
volatile float    g_voltage_d(0.0f);
volatile float    g_target_elec_rad(0.0f);
volatile uint16_t g_sensor_offset_u16(0);
volatile float    g_U_alpha(0.0f);
volatile float    g_U_beta(0.0f);
volatile float    g_debug_Uq_input(0.0f);          // Uq input to inverse park transform
volatile float    g_debug_Ud_input(0.0f);          // Ud input to inverse park transform
volatile float    g_debug_U_beta_calc_sa(0.0f);    // _sa value used in U_beta calculation
volatile float    g_debug_U_beta_calc_ca(0.0f);    // _ca value used in U_beta calculation
volatile float    g_debug_U_beta_term1(0.0f);      // _sa * Ud term
volatile float    g_debug_U_beta_term2(0.0f);      // _ca * Uq term
volatile float    g_debug_m_U_beta_after_calc(0.0f); // m_U_beta immediately after calculation
volatile float    g_debug_m_U_beta_before_assign(0.0f); // m_U_beta before assignment to g_U_beta
volatile float    g_debug_m_U_beta_at_assign(0.0f); // m_U_beta at exact moment of assignment to g_U_beta
volatile float    g_clean_U_beta_for_plot(0.0f);    // Clean U_beta value (only updated when angle != 0) for debug plots
volatile float    g_debug_ca_at_calc(0.0f); // _ca value at moment of m_U_beta calculation
volatile float    g_debug_sa_at_calc(0.0f); // _sa value at moment of m_U_beta calculation
volatile float    g_debug_Ud_at_calc(0.0f); // Ud value at moment of m_U_beta calculation
volatile float    g_debug_Uq_at_calc(0.0f); // Uq value at moment of m_U_beta calculation
volatile float    g_debug_term1_direct(0.0f); // _sa * Ud calculated directly
volatile float    g_debug_term2_direct(0.0f); // _ca * Uq calculated directly
volatile float    g_debug_m_U_beta_calc_direct(0.0f); // m_U_beta calculated as term1 + term2
volatile float    g_debug_m_U_beta_diff(0.0f); // Difference between m_U_beta and direct calc
volatile float    g_debug_sa_for_term1(0.0f); // _sa value used in term1 calculation
volatile float    g_debug_Ud_for_term1(0.0f); // Ud value used in term1 calculation
volatile float    g_debug_ca_for_term2(0.0f); // _ca value used in term2 calculation
volatile float    g_debug_Uq_for_term2(0.0f); // Uq value used in term2 calculation
volatile float    g_debug_term1_verify(0.0f); // term1 recalculated from captured values
volatile float    g_debug_term2_verify(0.0f); // term2 recalculated from captured values
volatile float    g_debug_m_U_beta_calc_verify(0.0f); // m_U_beta recalculated from verified terms
volatile bool     g_motor_enabled(false);
volatile uint32_t g_park_transform_counter(0);
volatile uint32_t g_angle_zero_skip_counter(0);  // Counts how many times we skip due to angle==0
volatile float    g_park_sin(0.0f);
volatile float    g_park_cos(0.0f);
volatile float    g_debug_angle_when_zero(0.0f);  // Captures the input angle when it's detected as 0
volatile float    g_debug_ramped_speed(0.0f);     // Track ramped_speed value
volatile float    g_debug_actual_target_rps(0.0f); // Track actual_target_rps after ramping
volatile uint32_t g_debug_epsilon_trigger_count(0); // Count how many times epsilon check triggers
volatile float    g_debug_electric_angle_for_sincos(0.0f);  // Angle actually passed to sin/cos (validated)
volatile float    g_debug_electric_angle_input(0.0f);       // Angle input to compute_inverse_park_transform
volatile float    g_debug_cosf_result(0.0f);                 // Raw result from cosf() before assignment
volatile float    g_debug_sinf_result(0.0f);                // Raw result from sinf() before assignment
volatile float    g_debug_validated_angle_at_cosf(0.0f);     // Angle value at the moment cosf() is called
volatile float    g_debug_cos_angle_before_wrap(0.0f);        // cos_angle value BEFORE wrapping
volatile float    g_debug_cos_angle_before_sinf(0.0f);        // cos_angle value AFTER wrapping, before sinf()
volatile float    g_debug_cos_angle_before_wrap_check(0.0f);  // cos_angle value before wrap check
volatile bool     g_debug_cos_angle_needs_wrap(false);         // Whether cos_angle needs wrapping
volatile float    g_debug_cos_angle_after_wrap(0.0f);          // cos_angle value AFTER wrapping
volatile float    g_debug_angle_before_cos(0.0f);              // Angle input to _cos function
volatile float    g_debug_a_sin_before_wrap(0.0f);            // a_sin value before wrapping in _cos
volatile float    g_debug_a_sin_after_wrap(0.0f);              // a_sin value after wrapping in _cos
volatile float    g_debug_cos_via_sinf(0.0f);                  // Cosine computed via sinf(a_sin)

//=============================================================================

//-----------------------------------------------------------------------------
//                              normalize_angle
//-----------------------------------------------------------------------------
float normalize_angle(float angle)
{
    //const float MAX_ANGLE = 2.0f * M_PI;
    //while (angle >= MAX_ANGLE) angle -= MAX_ANGLE;
    //while (angle < 0.0f) angle += MAX_ANGLE;
    //return angle;
    return _normalizeAngle(angle);;
}

//-----------------------------------------------------------------------------
//                          smooth
//-----------------------------------------------------------------------------
float smooth(float current_value, float previous_value, float alpha)
{
    // Ensure alpha is between 0 and 1; 0 means no smoothing, 1 means full smoothing (no change)
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    // Apply smoothing: blend current and previous values
    return (alpha * current_value) + ((1.0f - alpha) * previous_value);
}

//-----------------------------------------------------------------------------
//                        compute_inverse_park_transform
//
// Function uses sine approximation
//    regular  sin +  cos ~300us    (no memory usaage)
//    approx  _sin + _cos ~110us    (400Byte ~ 20% of memory)
//
//! @param Uq
//! @param Ud
//! @param electric_angle
//-----------------------------------------------------------------------------
#include <cmath>
void sincos(float a, float* s, float* c)
{
  *s = sinf(a);
  *c = cosf(a);
}


unsigned long prev_us = 0;


//=============================================================================
//                          StepperMotor Class
//=============================================================================
StepperMotor::StepperMotor(SPI_HandleTypeDef* hspi, 
                                int                pole_pairs, 
                                float              phase_resistance, 
                                float              KV, 
                                float              phase_inductance,
                                float              voltage_limit,
                                float              power_supply_voltage,
                                TIM_HandleTypeDef* p_htim_phase_1,
                                TIM_HandleTypeDef* p_htim_phase_2,
                                uint32_t           timer_channel_phase_1A,
                                uint32_t           timer_channel_phase_1B,
                                uint32_t           timer_channel_phase_2A,
                                uint32_t           timer_channel_phase_2B
)
:     MY_PI( 3.14159265358979323846f)
,     HALF_PI(        1.57079632679f)
,     TWO_PI(         6.28318530718f)
,     THREE_PI(       9.42477796077f)
,     THREE_HALVES_PI(4.71238898038f)
,     RAD_PER_SEC_TO_REV_PER_MIN(60.0f/TWO_PI)
,     NUM_POLE_PAIRS(static_cast<float>(pole_pairs))
,     KV_RPM_PER_VOLT(KV*_SQRT2)  // save back emf constant KV = 1/K_bemf
,     PHASE_RESISTANCE(phase_resistance)
,     PHASE_RESISTANCE_INVERSE(1.0f/PHASE_RESISTANCE)
,     PHASE_INDUCTANCE(phase_inductance)
,     PHASE_INDUCTANCE_INVERSE(1.0f/PHASE_INDUCTANCE)

,     PHASE_RESISTANCE_1KHZ(24.29f)
,     PHASE_RESISTANCE_10KHZ(212.00)
,     PHASE_INDUCTANCE_1KHZ(0.0039f)   // 3.9 mH --> 0.0039 H
,     PHASE_INDUCTANCE_10KHZ(0.0032f)

,     MIN_ALIGN_ANGLE_DETECT_MOVEMENT(0.1f)       // Minimum angle to detect movement, adjust as needed
,     MAX_SENSOR_ANGLE(TWO_PI)     // Assuming 360 degree range for the sensor
,     NUM_STEPS(100)
,     ROTATION_ANGLE(MY_PI / 2.0f)  // 90 degrees in radians
,     STEP_SIZE(ROTATION_ANGLE / static_cast<float>(NUM_STEPS) )
,     SQUARE_ROOT_OF_3_INVERSE(1.0f/static_cast<float>(compile_time_sqrt(3.0)))
,     PERM_MAGNET_FLUX_LINKAGE(0.171f)
,     m_sensor(hspi)
,     m_driver(p_htim_phase_1,
               p_htim_phase_2,
               voltage_limit,
               power_supply_voltage,
               timer_channel_phase_1A,
               timer_channel_phase_1B,
               timer_channel_phase_2A,
               timer_channel_phase_2B
               )		     
,     m_U_alpha(0.0f)
,     m_U_beta(0.0f)
,     m_prev_open_loop_timestamp_us(0)
//------------------------------------------
//    from FOCMotor
//------------------------------------------
,   m_target(0.0f)
,   m_target_prev(0.0)
,   m_target_prev_timestamp(0l)
,   m_feed_forward_velocity(0.0f)
,   m_shaft_angle(0.0f)
,   m_shaft_rad_per_sec(0.0f)
,   m_omega_mechanical_rps(0.0f)
,   m_current_sp(0.0f)
,   m_shaft_velocity_target(0.0f)
,   m_shaft_angle_target(0.0f)
,   m_voltage()
,   m_voltage_prev()
,   m_amperage()
,   m_amperage_prev()
,   m_voltage_bemf(0)
,   m_voltage_sensor_align(0.2f * voltage_limit)  //12.0f)   //power_supply_voltage)
,   m_velocity_index_search(DEF_INDEX_SEARCH_TARGET_VELOCITY)

,   m_voltage_limit(voltage_limit)
,   m_current_limit(DEF_CURRENT_LIM)
,   m_velocity_limit(DEF_VEL_LIM)     //   maximum angular velocity to be used for positioning
,   m_enabled(false)
,   m_motor_status(FOC_MOTOR_STATUS::UNINITIALIZED)
,   m_foc_modulation(FOC_MODULATION_TYPE::SINE_PWM)
,   m_modulation_centered(1) 

,   m_torque_control(TORQUE_CONTROL_TYPE::VOLTAGE)  // FOC_CURRENT
,   m_motion_control(MOTION_CONTROL_TYPE::CL_VELOCITY)  //  TORQUE) //OL_VELOCITY  CL_VELOCITY

,   m_PID_amperage_q( DEF_PID_CURR_P,
                      DEF_PID_CURR_I,
                      DEF_PID_CURR_D,
                      DEF_PID_CURR_RAMP, 
                      DEF_POWER_SUPPLY)
                      
,   m_PID_amperage_d( DEF_PID_CURR_P,
                      DEF_PID_CURR_I,
                      DEF_PID_CURR_D,
                      DEF_PID_CURR_RAMP, 
                      DEF_POWER_SUPPLY)
                      
,   m_PID_velocity(   0.005f,   // P 0.05  .2
                      0.01f,  // I  0.1  20
                      0.005f,  // D .02   .001
                      DEF_PID_VEL_RAMP,
                      100) //DEF_PID_VEL_LIMIT)
                      
,   m_PID_angle(      DEF_P_ANGLE_P,
                      0,
                      0,
                      0,
                      DEF_VEL_LIM)
                      
,   m_LPF_current_winding_a(  DEF_CURR_FILTER_Tf)
,   m_LPF_current_winding_b(  DEF_CURR_FILTER_Tf)

,   m_LPF_current_q(  DEF_CURR_FILTER_Tf)
,   m_LPF_current_d(  DEF_CURR_FILTER_Tf)
,   m_LPF_velocity(   0.5f)
,   m_LPF_angle(      0.0002f)  
,   m_LPF_back_emf(   0.05f)

//,   m_motion_downsample(DEF_MOTION_DOWNSMAPLE)
//,   m_motion_cnt(0)
,   m_sensor_offset(0.0f)
,   m_radian_offset_to_electric_zero(NOT_SET)
,   m_sensor_direction(Direction::UNKNOWN)

// TODO: move to another class
,   m_monitor_downsample(DEF_MON_DOWNSMAPLE)
,   m_monitor_start_char('\0')
,   m_monitor_end_char('\0')
,   m_monitor_separator('\t')
,   m_monitor_decimals(4)
,   monitor_variables(_MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE)
//
,   m_p_current_sense(nullptr)
,   m_current_offset_a(100)
,   m_current_offset_b(100)
,   m_hifactor_a(1.0f)
,   m_lofactor_a(1.0f)
,   m_hifactor_b(1.0f)
,   m_lofactor_b(1.0f)
,   m_commanded_speed_radians_per_sec(0.0f)
,   m_commanded_angle_radians(0.0f)
,   m_measured_speed_radians_per_sec(0.0f)
,   m_measured_angle_radians(0.0f)

,   m_target_voltage_q(0.0f)
,   m_accumulated_mechanical_radians(0.0f)
,   m_current_sample()
{
    m_voltage.q = 0.0f;
    m_voltage.d = 0.0f;
    setPhaseVoltage(0.0f, 0.0f, 0.0f);

    return;
}


void StepperMotor::update_samples(uint16_t raw_count)
{

    m_sensor.update_buffers(raw_count, micros());
}



//-----------------------------------------------------------------------------
//                               init
//-----------------------------------------------------------------------------
void StepperMotor::init() 
{    
    m_motor_status       = FOC_MOTOR_STATUS::INITIALIZING;

    m_PID_angle.limit      = m_velocity_limit;
    
   // HAL_Delay(500);
    enable();   // enable motor
   // HAL_Delay(500);

    m_motor_status = FOC_MOTOR_STATUS::UNCALIBRATED;
}

//-----------------------------------------------------------------------------
//                                  disable
//-----------------------------------------------------------------------------
void StepperMotor::disable()
{
  m_driver.disable();
  
  m_enabled = false;
}

//-----------------------------------------------------------------------------
//                                  enable
//-----------------------------------------------------------------------------
void StepperMotor::enable()
{
  m_driver.enable();

  m_enabled = true;
}

//-----------------------------------------------------------------------------
//                                  resistance
//-----------------------------------------------------------------------------
float StepperMotor::resistance(float electical_radians_per_second)
{
//return PHASE_RESISTANCE_10KHZ;

    const float ONE_KHZ_N_RPS(1000  * TWO_PI);
    const float TEN_KHZ_N_RPS(10000 * TWO_PI);


    const float SLOPE = (PHASE_RESISTANCE_10KHZ - PHASE_RESISTANCE_1KHZ)
                      / (TEN_KHZ_N_RPS - ONE_KHZ_N_RPS);
    
    float resistance(0.0f);
    
    if(electical_radians_per_second < ONE_KHZ_N_RPS)
    {
       resistance = PHASE_RESISTANCE_1KHZ;
    }
    else
    if(electical_radians_per_second > TEN_KHZ_N_RPS)
    {
       resistance = PHASE_RESISTANCE_10KHZ;
    }
    else
    {
       resistance = PHASE_RESISTANCE_1KHZ
                  + (SLOPE * electical_radians_per_second); 
    }

    return resistance;
}

//-----------------------------------------------------------------------------
//                                  inductance
//-----------------------------------------------------------------------------
float StepperMotor::inductance(float electical_radians_per_second)
{
  //  return PHASE_INDUCTANCE_10KHZ;

    const float ONE_KHZ_N_RPS(1000  * TWO_PI);
    const float TEN_KHZ_N_RPS(10000 * TWO_PI);

    const float SLOPE = (PHASE_INDUCTANCE_10KHZ - PHASE_INDUCTANCE_1KHZ)
                      / (TEN_KHZ_N_RPS - ONE_KHZ_N_RPS);


    float inductance(0.0f);

    if(electical_radians_per_second <= ONE_KHZ_N_RPS)
    {
        inductance = PHASE_INDUCTANCE_1KHZ;
    }
    else
    if(electical_radians_per_second >= TEN_KHZ_N_RPS)
    {
        inductance = PHASE_INDUCTANCE_10KHZ;
    }
    else
    {
        inductance = PHASE_INDUCTANCE_1KHZ
                   + SLOPE * (electical_radians_per_second - ONE_KHZ_N_RPS); 
    }

    return inductance;
}

//-----------------------------------------------------------------------------
//                                  smooth_voltage_adjustment
//-----------------------------------------------------------------------------
float StepperMotor::smooth_voltage_adjustment( float current_voltage, 
                                                          float desired_voltage, 
                                                          float smoothing_rate, 
                                                          float delta_t)
{
    return current_voltage + (desired_voltage - current_voltage) * smoothing_rate * delta_t;
}

//-----------------------------------------------------------------------------
//                                  smooth_voltage_adjustment
//-----------------------------------------------------------------------------
float StepperMotor::calculate_smoothing_rate(float target_electrical_rps)
{
    const float LOW_SPEED_HZ(3.0f);
    const float LOW_SPEED_THRESHOLD(LOW_SPEED_HZ * TWO_PI);

    // select rates so they sum to 1.0
    const float LOW_SPEED_SMOTHING_RATE(0.1f);           // 0.1-0.5 .3
    const float HIGH_SPEED_SMOOTHING_RATE(0.9f);         // 0.5-0.9 .7

  
    if (target_electrical_rps < LOW_SPEED_THRESHOLD)
    {
        return LOW_SPEED_SMOTHING_RATE;
    }
    else
    {
        return HIGH_SPEED_SMOOTHING_RATE;
    }
}

//-----------------------------------------------------------------------------
//                                initFOC
//
// return true on success
//-----------------------------------------------------------------------------
bool  StepperMotor::initFOC() 
{
    init(); // TODO: should this be 'reset'?

    bool success(true);

  
    m_motor_status = FOC_MOTOR_STATUS::UNCALIBRATED;

    // align motor if necessary
    // alignment necessary for encoders!
    // sensor and motor alignment - can be skipped
    // by setting motor.sensor_direction and motor.zero_electric_angle
 // disabled to debug reset  HAL_Delay(500);

    success &= alignSensor(); // bitwise intentional

// TBV PRP
success = true;

#if 0 
    //m_shaft_angle = m_sensor.read_angle_radians_from_buffer();
    // TBV Is a read really needed here?
    float radians;
    if(!m_sensor.fetch_radians(radians))
    {
        return false;
    }
    m_shaft_angle = radians;
#endif

    if(success)
    {
    	m_motor_status = FOC_MOTOR_STATUS::READY;
    	enable();  // Ensure motor stays enabled after alignment
    }
    else
    {
    	m_motor_status = FOC_MOTOR_STATUS::CALIBRATION_FAILED;
        disable();
    }

    return success;
}

#if 0
//-----------------------------------------------------------------------------
//                        determine_sensor_direction
//-----------------------------------------------------------------------------
bool StepperMotor::determine_sensor_direction()
{
       bool  success(true);

//return true;

       float mechanical_angle(0.0f);
       
       float initial_angle = m_sensor.read_angle_radians();
    
       // Command a 90 degree rotation over 100 steps
       for (uint32_t i = 0; i <= NUM_STEPS; i++ )
       {
           mechanical_angle = initial_angle  +  (i * STEP_SIZE);
         
           setPhaseVoltage(m_voltage_sensor_align, 
                           0.0f,  
                           _normalizeAngle(mechanical_to_electrical_radians(mechanical_angle)));
       
           HAL_Delay(10);
       }

       HAL_Delay(100);
       m_voltage.q = 0.0f;
       m_voltage.d = 0.0f;
       setPhaseVoltage(0.0f, 0.0f,  0.0f);
       
       float final_angle = m_sensor.read_angle_radians();
    
       float delta_angle = final_angle - initial_angle;
    
       // Check for rollover
       if (delta_angle < -MAX_SENSOR_ANGLE + ROTATION_ANGLE) 
       {
           delta_angle += MAX_SENSOR_ANGLE;
       }
    
      
       if (fabs(delta_angle) < MIN_ALIGN_ANGLE_DETECT_MOVEMENT) 
       {
           // Movement too small to determine direction
           success = false;
           m_sensor_direction = Direction::UNKNOWN;
       } 
       
       else if (delta_angle > 0) 
       {
          m_sensor_direction = Direction::CW;
       } 
       else 
       {
         m_sensor_direction = Direction::CCW;
       }

       m_sensor.invert_output(delta_angle < 0.0f);
       
       return success;
}
#endif

//bool StepperMotor::is_clockwise()
//{
//    return Direction::CW == m_sensor_direction;
//}


//-----------------------------------------------------------------------------
//                            alignSensor
//
// Encoder alignment to electrical 0 angle
//
// return true on success
//-----------------------------------------------------------------------------
//  Disabled on 12.2.2025
// bool StepperMotor::alignSensor()
// {
//   const uint32_t TWO_MILLISECONDS(2);

//   if(true) //(Direction::UNKNOWN == m_sensor_direction)
//   {
//     #if 0
// 	// We don't have a zero position sensor
//     // If using a zero index sensor
//     if(needsSearch()) // TODO: need a clearer func name
//     {
//         success = absoluteZeroSearch();
//     }
    
//     // exit if index not found
//     if(!success)
//     {
//         return success;
//     }
//     #endif
    
//     // find natural direction
//     // move one electrical revolution forward
//     // ramp up from 1.5Pi to 3.5Pi
//     float electric_angle;
//     float mid_angle = 0.0f;
    
//     // Initial Rotation (270 degrees CW)
//     for (int i = 0; i <=500; i++ )
//     {
//         electric_angle = THREE_HALVES_PI + TWO_PI * i / 500.0f;
      
//         setPhaseVoltage(m_voltage_sensor_align, 0.0f, electric_angle);
        
//         if(!m_sensor.fetch_radians(mid_angle))
//         {
//             //return false;
//         }

// 	    HAL_Delay(TWO_MILLISECONDS);
//     }

//     //  mid_angle represents the sensor's reading after 
//     //  270 degree clockwise motor rotation.
//     if(!m_sensor.fetch_radians(mid_angle))
//     {
//         //return false;
//     }


    

//     float end_angle = 0.0f;
    
//     // Reverse Rotation (270 degrees CCW)
//     for (int i = 500; i >=0; i-- ) 
//     {
//         electric_angle = THREE_HALVES_PI + TWO_PI * i / 500.0f ;
        
//         setPhaseVoltage(m_voltage_sensor_align, 0.0f, electric_angle);

//         if(!m_sensor.fetch_radians(end_angle))
//         {
//             //return false;
//         }
        
// 	    HAL_Delay(TWO_MILLISECONDS);
//     }
        
//     if(!m_sensor.fetch_radians(end_angle))
//     {
//         //return false;
//     }
    
//     // zero applied voltages
//     setPhaseVoltage(0, 0, 0);                  

//     HAL_Delay(200);

//     // determine the direction the sensor moved
//     if (mid_angle == end_angle)
//     {
//       return 0; // failed calibration
//     }
//     else
//     if (mid_angle < end_angle)
//     {
//         m_sensor_direction = Direction::CCW;
//     }
//     else
//     {
//         m_sensor_direction = Direction::CW;
//     }

//     // Set direction flag
//     m_sensor.invert_output(mid_angle < end_angle);
    
//   }


//   // zero electric angle not known
//   if(NOT_SET == m_radian_offset_to_electric_zero)
//   {
//       // align the electrical phases of the motor and sensor
//       // set angle -90(270 = 3PI/2) degrees
//       //float holding_angle_electrical = _normalizeAngle(
//       //                                      mechanical_to_electrical_radians(
//       //                                                THREE_HALVES_PI));

//       //float holding_angle_electrical = 0.0f;
      
//   //    setPhaseVoltage(m_voltage_sensor_align, 0,  holding_angle_electrical);

//    //   HAL_Delay(200);

      
//       // get the m_amperage zero electric angle
//       m_sensor_offset                  = m_sensor.read_radians_with_direction();
//       m_radian_offset_to_electric_zero = mechanical_to_electrical_radians(m_sensor_offset);
//       //m_radian_offset_to_electric_zero = get_electric_angle_radians_v2(); //get_electric_angle_radians();

//       g_sensor_offset_u16 = m_sensor.get_count();
      
//       HAL_Delay(20);

//       // stop everything
//       setPhaseVoltage(0.0f, 00.0f, 0.0f);

//       HAL_Delay(200);
//   }
  
//   return true; //success;
// }

// refactored on 12.2.2025
// bool StepperMotor::alignSensor()
// {
//     const uint32_t TWO_MILLISECONDS(2U);
//     const int32_t  NUM_STEPS(500);

//     bool success = true;

//     // ------------------------------------------------------------------------
//     // 1) Determine encoder direction relative to applied electrical rotation
//     // ------------------------------------------------------------------------
//     float mid_mechanical_radians = 0.0f;
//     float end_mechanical_radians = 0.0f;

//     // Forward electrical sweep: 1.5π → 1.5π + 2π
//     for (int32_t i = 0; i <= NUM_STEPS; ++i)
//     {
//         const float sweep_fraction =
//             static_cast<float>(i) / static_cast<float>(NUM_STEPS);

//         const float electrical_angle_radians =
//             THREE_HALVES_PI + TWO_PI * sweep_fraction;

//         // NOTE: electrical_angle_radians is already electrical; do NOT call
//         // mechanical_to_electrical_radians() here.
//         setPhaseVoltage(m_voltage_sensor_align,
//                         0.0f,
//                         electrical_angle_radians);

//         (void)m_sensor.fetch_radians(mid_mechanical_radians);

//         HAL_Delay(TWO_MILLISECONDS);
//     }

//     // Reverse electrical sweep: 1.5π + 2π → 1.5π
//     for (int32_t i = NUM_STEPS; i >= 0; --i)
//     {
//         const float sweep_fraction =
//             static_cast<float>(i) / static_cast<float>(NUM_STEPS);

//         const float electrical_angle_radians =
//             THREE_HALVES_PI + TWO_PI * sweep_fraction;

//         setPhaseVoltage(m_voltage_sensor_align,
//                         0.0f,
//                         electrical_angle_radians);

//         (void)m_sensor.fetch_radians(end_mechanical_radians);

//         HAL_Delay(TWO_MILLISECONDS);
//     }

//     if (mid_mechanical_radians < end_mechanical_radians)
//     {
//         // Sensor increases when the field rotates "forward"
//         m_sensor_direction = Direction::CCW;
//         m_sensor.invert_output(true);
//     }
//     else
//     {
//         // Sensor decreases when the field rotates "forward"
//         m_sensor_direction = Direction::CW;
//         m_sensor.invert_output(false);
//     }

//     // ------------------------------------------------------------------------
//     // 2) Determine electrical zero while holding a fixed stator field
//     // ------------------------------------------------------------------------
//     if (NOT_SET == m_radian_offset_to_electric_zero)
//     {
//         // Hold a known electrical angle (0 rad is fine as long as we are consistent)
//         const float holding_electrical_angle_radians = 0.0f;

//         setPhaseVoltage(m_voltage_sensor_align,
//                         0.0f,
//                         holding_electrical_angle_radians);

//         HAL_Delay(200U);

//         // Read the mechanical angle at which the rotor aligns to that field
//         const float mechanical_zero_radians =
//             m_sensor.read_radians_with_direction();

//         // Store raw mechanical angle for debugging/telemetry
//         m_sensor_offset = mechanical_zero_radians;

//         // Convert that mechanical angle to its electrical equivalent;
//         // this is our "electrical zero" offset for later use:
//         //   electrical_now = mech_to_elec(sensor_mech_now) - m_radian_offset_to_electric_zero
//         m_radian_offset_to_electric_zero =
//             mechanical_to_electrical_radians(mechanical_zero_radians);

//         g_sensor_offset_u16 = m_sensor.get_count();

//         HAL_Delay(20U);

//         // Stop driving the motor
//         setPhaseVoltage(0.0f, 0.0f, 0.0f);

//         HAL_Delay(200U);
//     }

//     return success;
// }

// Simplified on 12.2.2025
bool StepperMotor::alignSensor()
{
    const uint32_t SETTLE_MS           = 200U;
    const float    ALIGN_ELECTRICAL_0  = 0.0f;        // electrical angle for "zero" field
    const float    ALIGN_ELECTRICAL_DT = HALF_PI;     // +90 deg electrical step

    float mech_angle_0   = 0.0f;
    float mech_angle_90  = 0.0f;

    // ------------------------------------------------------------------------
    // 1) Snap rotor to a fixed electrical angle and measure mechanical angle
    // ------------------------------------------------------------------------
    setPhaseVoltage(m_voltage_sensor_align,
                    0.0f,
                    ALIGN_ELECTRICAL_0);

    HAL_Delay(SETTLE_MS);

    // Raw mechanical angle at "electrical zero field"
    if (!m_sensor.fetch_radians(mech_angle_0))
    {
        // If sensor read fails, bail out
        setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }

    // ------------------------------------------------------------------------
    // 2) Move stator field +90 electrical degrees and measure again
    // ------------------------------------------------------------------------
    setPhaseVoltage(m_voltage_sensor_align,
                    0.0f,
                    ALIGN_ELECTRICAL_0 + ALIGN_ELECTRICAL_DT);

    HAL_Delay(SETTLE_MS);

    if (!m_sensor.fetch_radians(mech_angle_90))
    {
        setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }

    // ------------------------------------------------------------------------
    // 3) Determine direction from how the mechanical angle changed
    // ------------------------------------------------------------------------
    float mech_delta = normalize_radians(mech_angle_90 - mech_angle_0);

    // If increasing electrical angle leads to increasing mechanical angle,
    // treat that as CCW; otherwise CW. We preserve your original pattern:
    //   condition             -> Direction
    //   (delta > 0)          -> CCW  + invert_output(true)
    //   (delta <= 0)         -> CW   + invert_output(false)
    bool increasing = (mech_delta > 0.0f);

    if (increasing)
    {
        m_sensor_direction = Direction::CCW;
    }
    else
    {
        m_sensor_direction = Direction::CW;
    }

    // Configure the AS5048 output sign according to that direction
    m_sensor.invert_output(increasing);

    // ------------------------------------------------------------------------
    // 4) Re-snap at electrical zero and compute electrical offset
    // ------------------------------------------------------------------------
    setPhaseVoltage(m_voltage_sensor_align,
                    0.0f,
                    ALIGN_ELECTRICAL_0);

    HAL_Delay(SETTLE_MS);

    // Now read mechanical angle with direction already applied
    float mech_zero_with_dir = m_sensor.read_radians_with_direction();

    // Store for diagnostics
    m_sensor_offset = mech_zero_with_dir;

    // Convert that mechanical angle to electrical; this is the offset we
    // subtract from mech_to_elec() everywhere:
    //
    //   electrical_now = mech_to_elec(sensor_mech_now) - m_radian_offset_to_electric_zero
    //
    m_radian_offset_to_electric_zero =
        mechanical_to_electrical_radians(mech_zero_with_dir);

    g_sensor_offset_u16 = m_sensor.get_count();

    HAL_Delay(20U);

    // ------------------------------------------------------------------------
    // 5) Stop driving the motor and initialize open-loop angle accumulator
    // ------------------------------------------------------------------------
    setPhaseVoltage(0.0f, 0.0f, 0.0f);
    
    // CRITICAL: Initialize accumulated angle to match rotor's actual position
    // The rotor is now aligned to electrical angle 0, so set mechanical accumulator
    // to the mechanical angle that corresponds to electrical 0
    m_accumulated_mechanical_radians = mech_zero_with_dir;
    
    HAL_Delay(SETTLE_MS);

    return true;
}






#if 0
//-----------------------------------------------------------------------------
//                          absoluteZeroSearch
//
// Encoder alignment the absolute zero angle - to the index pin
//
// return true if search is complete
//
// Just return since we don't have an index pin
//-----------------------------------------------------------------------------
bool StepperMotor::absoluteZeroSearch()
{
    // Push limits
    float limit_vel    = m_velocity_limit;
    float limit_volt   = m_voltage_limit;
    
    m_velocity_limit   = m_velocity_index_search;
    m_voltage_limit    = m_voltage_sensor_align;
    
    // search the absolute zero with small velocity  
    m_shaft_angle      = 0.0f;
    
    while(needsSearch() && m_shaft_angle < TWO_PI)
    {
      angleOpenloop(THREE_PI);
      
      // update is important for some sensors not to loose count
      // not needed for the search
      //m_sensor.update();
    }
    
    // disable motor
    setPhaseVoltage(0.0f, 0.0f, 0.0f); // TODO: refactor name
    
    // Pop limits
    m_velocity_limit = limit_vel;
    m_voltage_limit  = limit_volt;

    return !needsSearch();
}
#endif

//-----------------------------------------------------------------------------
//                read_angle_radians_from_buffer_with_offset
//-----------------------------------------------------------------------------
float StepperMotor::read_angle_radians_from_buffer_with_offset()
{
  //return m_sensor.read_angle_radians_from_buffer() - m_sensor_offset;
  return m_sensor.read_radians_with_direction() - m_sensor_offset;
}

//-----------------------------------------------------------------------------
//                               loopFOC
//
// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
//-----------------------------------------------------------------------------
float test_shaft_radians = 0.0f;

void StepperMotor::loopFOC(float winding_amperage_a, float winding_amperage_b)
{
    g_loopfoc_counter++;
    
    const float SECONDS_PER_MICROSECOND( 0.000001f); // TODO move to class level
    const float MICROSECONDS_PER_FRAME (static_cast<float>(MICROSECONDS_PER_ITERATION));
    const float DELTA_T_SECONDS(MICROSECONDS_PER_FRAME * SECONDS_PER_MICROSECOND);


    float shaft_radians = 0.0f;
    
#if 0  // Enable sensor read for closed-loop operation
    if(m_sensor.fetch_radians(shaft_radians))
    {
         // divide offset by range of 5 volts, then take half
         //float current_offset_a = m_current_offset_a.update(winding_amperage_a);
         //float current_offset_b = m_current_offset_b.update(winding_amperage_b);
         
         
         float corrected_current_a = 0.5f * winding_amperage_a ; //- current_offset_a;
         float corrected_current_b = 0.5f * winding_amperage_b ; //- current_offset_b;
         
         // Convert offsets to current in amperes ACS712-05B
         //float winding_amps_a = corrected_current_a / 0.185; // Convert corrected voltage to current
         //float winding_amps_b = corrected_current_b / 0.185; // Convert corrected voltage to current
     
         // Filter the current readings
         //m_winding_amperage_a = m_LPF_current_winding_a(winding_amperage_a);
         //m_winding_amperage_b = m_LPF_current_winding_a(winding_amperage_b);
         
         transformCurrents( corrected_current_a,
                            corrected_current_b,
                            shaft_radians,
                            m_amperage.d,
                            m_amperage.q);
         
         test_shaft_radians = shaft_radians;
                            
    }
#else  // SKIP sensor read for open-loop operation - SPI timing causes jitter
    // Still do current sensing and transform (doesn't require working sensor)
    float corrected_current_a = 0.5f * winding_amperage_a;
    float corrected_current_b = 0.5f * winding_amperage_b;
    
    // Use accumulated angle instead of sensor for current transform
    transformCurrents( corrected_current_a,
                       corrected_current_b,
                       0.0f,  // Arbitrary angle - we're not using current feedback anyway
                       m_amperage.d,
                       m_amperage.q);
#endif

    // Continue with motion control even if sensor read fails
    switch (m_motion_control) 
      {
        case MOTION_CONTROL_TYPE::TORQUE:
                         
             break;             
        
        case MOTION_CONTROL_TYPE::CL_ANGLE:
             
             update_position_closed_loop(m_target, DELTA_T_SECONDS);
    
             break;
             
             
        case MOTION_CONTROL_TYPE::CL_VELOCITY:
             // Use m_target as set by update_target_rad_per_sec() or move()
             // No longer hardcoding target speed - respect user's target
             update_speed_closed_loop(m_target,  DELTA_T_SECONDS);
                                  
             break;
             
        case MOTION_CONTROL_TYPE::OL_VELOCITY:

             update_speed_open_loop(m_target,  DELTA_T_SECONDS);

             break;
        
        case MOTION_CONTROL_TYPE::OL_ANGLE:
    
             update_position_open_loop(m_target,  DELTA_T_SECONDS);
             
             break;
    
        default:
             ; // NOP
    }

}

//-----------------------------------------------------------------------------
//                             move
//
// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or voltage loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
// 
// voltage and current elements are in quatrature/Direct coordinate system
//-----------------------------------------------------------------------------
void StepperMotor::update_target_rad_per_sec(float rps)
{
    m_target = rps;
}

//-----------------------------------------------------------------------------
//                                 move
//-----------------------------------------------------------------------------
void StepperMotor::move(float new_target) 
{
    const float SECONDS_PER_MICROSECOND( 0.000001f); // TODO move to class level
    const float MICROSECONDS_PER_FRAME (static_cast<float>(MICROSECONDS_PER_ITERATION));
    const float DELTA_T_SECONDS(MICROSECONDS_PER_FRAME * SECONDS_PER_MICROSECOND);
        
    float delta_target      = new_target - m_target_prev;
    m_target_prev           = new_target;
    

 #if 0   
    m_sensor.update();
        
    m_shaft_angle = get_filtered_shaft_angle();

    m_omega_mechanical_rps = shaft_radians_per_second();

    // calculate back-emf m_voltage if KV_rating available U_bemf = vel*(1/KV)
    float rpm = m_omega_mechanical_rps * RAD_PER_SEC_TO_REV_PER_MIN;
    m_voltage_bemf = rpm/KV_RPM_PER_VOLT;
#endif
    // Ohm's law
    // m_amperage appears to be set but not used
    //    m_amperage.q = (m_voltage.q - m_voltage_bemf) / resistance();

    // choose control loop
    switch (m_motion_control) 
    {
      case MOTION_CONTROL_TYPE::TORQUE:

           // In this context, the new_target has units of volts
           // The quadrature voltage controls the torque

           update_torque_open_loop(new_target, DELTA_T_SECONDS, delta_target);
                       
           break;
           
      
      case MOTION_CONTROL_TYPE::CL_ANGLE:
      {

           m_target = new_target;       
      }
           break;
           
           
      case MOTION_CONTROL_TYPE::CL_VELOCITY:
      {

          // m_target = new_target;    

           m_amperage_prev.d = m_amperage.d; // = 0.0f;
           m_amperage_prev.q = m_amperage.q; // = 0.0f;
           m_voltage_prev.d  = m_voltage.d; //  = 0.0f;
           m_voltage_prev.q  = m_voltage.q; //  = 0.0f;
      }                         
           break;
           
      case MOTION_CONTROL_TYPE::OL_VELOCITY:
      {
           m_target = new_target;  
#if 0
           const float DELTA_T(0.01f);
           float expected_change = m_target * DELTA_T;
           float angle_cmd = m_sensor.get_angle_radians() + expected_change;
           float norm_cmd  = normalize_radians(angle_cmd); // not converting to electrical yet

           float mag_flux_linkage_q            = PHASE_INDUCTANCE * m_amperage.q;
           float back_emf_q_axis               = m_omega_mechanical_rps 
                                               * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

           float voltage_q = m_current_limit * resistance() 
                           + fabs(back_emf_q_axis);
           
           m_voltage.q     = symetric_clamp(voltage_q, m_voltage_limit);
           m_voltage.d     = 0.0f;
           
           setPhaseVoltage(m_voltage.q, m_voltage.d, norm_cmd);
           
           m_amperage.q = (m_voltage.q - fabs(back_emf_q_axis))/resistance();
 #endif          
      }
           break;
           
      case MOTION_CONTROL_TYPE::OL_ANGLE:
      {
           m_target = new_target;  
      }    
           break;

      default:
           ; // NOP
  }
}

//-----------------------------------------------------------------------------
//                        update_torque_open_loop
//-----------------------------------------------------------------------------
void StepperMotor::update_torque_open_loop(
                                             float target_quadrature_voltage, 
                                             float delta_t, 
                                             float delta_target)
{      
      
      // divide by zero guard
      float delta_I_d = (fabs(delta_t) > 0.0f)
                      ? delta_target/delta_t
                      : 0.0f;

      float electrical_rps = mechanical_to_electrical_radians(m_omega_mechanical_rps);
                      
      float voltage_q =  target_quadrature_voltage * resistance(0.0f) 
                      +  inductance(electrical_rps) * delta_I_d
                      +  electrical_rps * m_voltage_bemf;
    
      m_voltage.q = symetric_clamp( voltage_q, 
                                    m_voltage_limit);
     
      // Assume current_d = 0
      // current_q comes from the pid this is the target and affects torque
      //
      //  voltage_d =  phase_resistance * target_Current_d
      //            +  phase_inductance * delta_current_d/delta_t
      //            -  omega * phase_inductance * current_q
      //
      // with current_d = 0, then delta_current_d/delta_t = 0
      // current_q = target
      //
      //  voltage_d =  -  omega * phase_inductance * target  
        
      float voltage_d =  -inductance(electrical_rps)
                      *   mechanical_to_electrical_radians(m_omega_mechanical_rps)
                      *   m_target;                                // Amperes
                
      m_voltage.d = symetric_clamp( voltage_d, 
                                    m_voltage_limit);

     setPhaseVoltage(m_voltage.q, m_voltage.d, get_electric_angle_radians());

}


#if 0 //backup
/-----------------------------------------------------------------------------
//                          update_speed_closed_loop
//
// This implements the torque control loop. As the stepper motors only support 
// torque using voltage mode. 
//
// Read the current motor angle from the sensor, turn it into the electrical 
// angle and transforms the q-axis Uq voltage command motor.voltage_q
//-----------------------------------------------------------------------------
void backup StepperMotor::update_speed_closed_loop(
                                                float target_mechanical_rps, 
                                                float delta_t)
{
    static float rad(0.0f);
    rad += target_mechanical_rps*delta_t;
    
    float electric_radians = normalize_radians(rad);



    float angular_velocity_error(target_mechanical_rps - shaft_radians_per_second());
    
    float current_sp = m_PID_velocity.update(angular_velocity_error); 
    float correction_term(0.0f);

    float electrical_rps           = mechanical_to_electrical_radians(target_mechanical_rps);
    float mag_flux_linkage_q       = PHASE_INDUCTANCE * m_amperage.q;
    float back_emf_q_axis          = electrical_rps 
                                   * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);
  
     m_voltage.q =  current_sp * resistance() + back_emf_q_axis;
     m_voltage.d = -current_sp * inductance() * electrical_rps;
                              
     m_voltage.q = symetric_clamp( m_voltage.q, m_voltage_limit);
     m_voltage.d = symetric_clamp( m_voltage.d, m_voltage_limit);


    setPhaseVoltage(m_voltage.q, m_voltage.d, get_electric_angle_radians());

    
    m_amperage.d = m_amperage_prev.d 
                + ( m_voltage.d -  m_voltage_prev.d ) * delta_t * PHASE_INDUCTANCE_INVERSE;
    
    m_amperage.q = m_amperage_prev.q 
                + ( m_voltage.q -  m_voltage_prev.q ) * delta_t * PHASE_INDUCTANCE_INVERSE;


    float next_position = m_sensor.read_angle_radians();
                        + target_mechanical_rps*delta_t
                        + correction_term;


    setPhaseVoltage(m_voltage.q, m_voltage.d, normalize_radians(next_position));

    m_amperage_prev.d = m_amperage.d;
    m_amperage_prev.q = m_amperage.q;
    m_voltage_prev.d  = m_voltage.d;
    m_voltage_prev.q  = m_voltage.q;

}


#endif


//-----------------------------------------------------------------------------
//                              sample_as5048_25us
//
// Caution: This is invoked from within an interrupt context
//-----------------------------------------------------------------------------
void StepperMotor::sample_as5048_25us()
{
      m_sensor.async_read_angle(); 
}


#if 0
uint32_t StepperMotor::compute_time_difference(uint32_t current_time, uint32_t previous_time) 
{
    const uint32_t MAX_UINT32 (0xFFFFFFFFU);
    
    if (current_time >= previous_time) {
        // No rollover, normal subtraction
        return current_time - previous_time;
    } else {
        // Rollover occurred, adjust the calculation
        return (MAX_UINT32 - previous_time + current_time + 1);
    }
}
#endif


//-----------------------------------------------------------------------------
//                              control_loop_25us
//
// Dependancies:
//
//
//-----------------------------------------------------------------------------
void StepperMotor::control_loop_25us()
{
  
    static ElapsedTime elapsed_time; // microseconds

    float curr_mech_rad = read_angle_radians_from_buffer_with_offset();
    float curr_elec_rad = mechanical_to_electrical_radians(curr_mech_rad);


    // Ensure delta_seconds is consistent and clamp it to avoid extreme values
    // fairly consistent ~50 us or 0.00005 seconds
    //float delta_seconds = std::max(
    //                           0.00001f, 
    //                           std::min(0.001f, elapsed_time.get() / 1000000.0f));

    float delta_mech_rad = m_commanded_speed_radians_per_sec * elapsed_time.get()* 0.000001f;
    float delta_elec_rad = mechanical_to_electrical_radians(delta_mech_rad);

    float cmd_elec_rad = curr_elec_rad + delta_elec_rad;

#if 0  
    //-------------------------------------
    // Smooth the target electrical radians
    //-------------------------------------

    // Smooth the q-axis voltage
    m_voltage.q = smooth_voltage_adjustment(m_voltage.q, m_target_voltage_q, 0.6f, delta_seconds);

    // Apply the phase voltages via PWM
    float smooth_target_electrical_radians = smooth(cmd_elec_rad, curr_elec_rad, 0.1f);
#endif

    g_voltage_q = m_voltage.q;
    g_voltage_d = m_voltage.d;
    g_target_elec_rad = cmd_elec_rad;
    


    setPhaseVoltage(m_voltage.q, m_voltage.d, cmd_elec_rad); 

}



//static unsigned long prev_us = 0;

#if 0 // presplit
void StepperMotor::update_speed_closed_loop(
                                                float target_mechanical_rps,
                                                float delta_seconds)
{
    // Proportional gains (adjust these as needed)
    float Kp_velocity = 0.01f; // Initial gain for velocity correction

    if(FP_ZERO == fpclassify(target_mechanical_rps))
     {
       m_voltage.q  = 0.0f;
       m_voltage.d  = 0.0f;
       setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);

       return;
     }
     

    // const uint32_t MICROSECONDS_PER_ITERATION(25); 


     volatile unsigned long now_us = _micros();
     g_us                          = now_us - prev_us;
     prev_us                       = now_us;
     delta_seconds = MICROSECONDS_PER_ITERATION/1000000.0f;

     // Calculate the target electrical speed and estimate shaft speed
     float target_electrical_rps = mechanical_to_electrical_radians(target_mechanical_rps);
     float shaft_radians_per_sec = m_sensor.get_radians_per_second();
     float velocity_error        = target_mechanical_rps - shaft_radians_per_sec;

     
     float velocity_correction = m_PID_velocity.update(velocity_error); 
     //float velocity_correction   = Kp_velocity * velocity_error;

     float mechanical_rps_cmd   = target_mechanical_rps + velocity_correction;

     // Calculate desired q-axis voltage (without proportional corrections for now)
     float computed_inductance = inductance(target_electrical_rps) ;
     float mag_flux_linkage_q = computed_inductance * m_amperage.q;
     float back_emf_q_axis    = shaft_radians_per_sec * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);
     float desired_voltage_q  = m_current_limit * resistance(target_electrical_rps)
                              + fabs(back_emf_q_axis)
                              + velocity_correction;
     
     
     //static float mechanical_radians(0.0f);
     //mechanical_radians     += mechanical_rps_cmd*delta_seconds;
     
     //float alt_electric_cmd  = mechanical_to_electrical_radians(mechanical_radians);
     //alt_electric_cmd        = _normalizeAngle(alt_electric_cmd);


    //static float electrical_radians = get_electric_angle_radians();
    //electrical_radians += mechanical_to_electrical_radians(mechanical_rps_cmd*delta_seconds);
    //electrical_radians  = _normalizeAngle(electrical_radians);

    float radian_advance_electrical = mechanical_to_electrical_radians(
                                                mechanical_rps_cmd*delta_seconds);

    float target_electrical_radians = _normalizeAngle( 
                                                get_electric_angle_radians()
                                                + radian_advance_electrical
                                                );

    float smooth_target_electrical_radians = 
         smooth( target_electrical_radians, 
                 get_electric_angle_radians(), 
                 0.3f); 

    float target_voltage_q = symetric_clamp( desired_voltage_q, 
                                             m_voltage_limit);
    
    m_voltage.d = 0.0f;
    m_voltage.q = smooth_voltage_adjustment( m_voltage.q, 
                                             target_voltage_q, 
                                             0.5f, 
                                             delta_seconds);   

    setPhaseVoltage(  m_voltage.q, 
                      m_voltage.d, 
                      smooth_target_electrical_radians);

    g_mag_flux_linkage_q = mag_flux_linkage_q;
    g_back_emf_q_axis    = back_emf_q_axis;
    // CRITICAL: g_as5048_angle is now only updated by update_buffers() from SPI data
    // Do not overwrite it here - use the sensor value directly for calculations
    float current_mechanical_angle = normalize_radians((m_sensor.get_mechanical_phase_angle_radians()));
    g_electrical_rad_ref = normalize_radians(mechanical_to_electrical_radians(current_mechanical_angle));
    g_back_emf_q_axis     = back_emf_q_axis;
    g_velocity_correction = velocity_correction;
    g_computed_inductance = computed_inductance;
    g_amperage_q          = m_amperage.q;
    //g_electrical_rad_cmd = electrical_radians;
    g_radian_advance_electrical = radian_advance_electrical;
}
#endif


//-----------------------------------------------------------------------------
//                              calculate_velocity
//-----------------------------------------------------------------------------
float StepperMotor::calculate_velocity(float current_angle, float delta_seconds)
{
    static float previous_angle = 0.0f;

    //float delta_angle = current_angle - previous_angle;
    double delta_angle = static_cast<double>(current_angle) - static_cast<double>(previous_angle);

    if(fabs(delta_angle) > M_PI)
    {
        // Rollover
        if(current_angle < previous_angle)
        {
           delta_angle += 2.0f * M_PI;
        }
        else
        {
            delta_angle -= 2.0f * M_PI;
        }
    }

    //float velocity = delta_angle / delta_seconds;
    double velocity = delta_angle / static_cast<double>(delta_seconds);

    previous_angle = current_angle;
    
    g_delta_angle = delta_angle;
    
    return velocity;
}



//-----------------------------------------------------------------------------
//                              update_speed_closed_loop
//-----------------------------------------------------------------------------
// Disabled on 12.2.2025
// void StepperMotor::update_speed_closed_loop(float target_rad_per_sec, float delta_seconds)
// {
//     static float delta_radians(0.0f);

// #if 0

//     if(  (FP_ZERO == fpclassify(target_rad_per_sec)) 
//          || 
//          (fabs(target_rad_per_sec) < 0.01f)
//       )
//     {
//         m_voltage.q  = 0.0f;
//         m_voltage.d  = 0.0f;
//         setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);

//         m_sensor.fetch_radians(mechanical_radians);

//         return;
//     }
// #endif

// //===============================
// static ElapsedTime elapsed_time; // microseconds

// #if 0
// float curr_mech_rad  = read_angle_radians_from_buffer_with_offset();
// float curr_elec_rad  = mechanical_to_electrical_radians(curr_mech_rad);
// float delta_mech_rad = m_commanded_speed_radians_per_sec * elapsed_time.get()* 0.000001f;
// float delta_elec_rad = mechanical_to_electrical_radians(delta_mech_rad);
// float cmd_elec_rad   = curr_elec_rad + delta_elec_rad;
// #endif
// //===============================


//     // Step 0: compute delta T
//     float delta_t        = static_cast<float>(elapsed_time.get()) * 0.000001f;
   
// #if 1
//     delta_radians        = target_rad_per_sec * delta_t;
//     float cmd_mech_rad   = test_shaft_radians + delta_radians;

//     // Convert to electrical and apply the calibrated offset
//     float cmd_elec_rad   = mechanical_to_electrical_radians(cmd_mech_rad)
//                          - m_radian_offset_to_electric_zero;

//      cmd_elec_rad = normalize_radians(cmd_elec_rad);                    
// #endif


//     // Step 1: Filter the shaft angle
//     //float raw_angle = get_filtered_shaft_angle();

//     // Step 2: Calculate the velocity from the filtered angle
//     //float raw_velocity = calculate_velocity(raw_angle, delta_seconds);  // Derivative of filtered angle

//     // Step 3: Calculate the velocity error and apply the PID controller
//     //float filtered_velocity = m_LPF_velocity(raw_velocity);
//     //float velocity_error = target_rad_per_sec - filtered_velocity;
//     //float velocity_correction = m_PID_velocity.update(velocity_error);
    

    
//    // m_commanded_speed_radians_per_sec = target_rad_per_sec + velocity_correction;

//     // Step 4: Calculate the back EMF
//     float computed_inductance = inductance(mechanical_to_electrical_radians(target_rad_per_sec));
//     //float mag_flux_linkage_q = computed_inductance * m_amperage.q;
//     float back_emf_q_axis = 0.0f; //filtered_velocity * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

//     // Step 5: Filter the back EMF
//     //float filtered_back_emf = m_LPF_back_emf(back_emf_q_axis);

//     // Step 6: Calculate the desired q-axis voltage
//     float desired_voltage_q = m_current_limit * resistance(target_rad_per_sec) 
//                             + fabs(back_emf_q_axis); 
//                            // + velocity_correction;

//     m_target_voltage_q = symetric_clamp(desired_voltage_q, m_voltage_limit);

//     m_voltage.d = 0.0f;
//     m_voltage.q = m_target_voltage_q;


    
//     setPhaseVoltage(m_voltage.q, m_voltage.d, cmd_elec_rad); 

//     g_voltage_q = m_voltage.q;
//     g_voltage_d = m_voltage.d;
//     g_target_elec_rad = _normalizeAngle(cmd_elec_rad);

    

// }


    #if 0
    float target_electrical_rps = mechanical_to_electrical_radians(target_rad_per_sec);

    // Calculate velocity error and apply PID controller
    float sensor_rad_per_sec    = m_sensor.get_radians_per_second();
    float velocity_error        = target_rad_per_sec - sensor_rad_per_sec;
    float velocity_correction   = m_PID_velocity.update(velocity_error);
    
    // Update the commanded mechanical speed based on velocity correction
    m_mechanical_rps_cmd        = target_rad_per_sec + velocity_correction;

    float computed_inductance   = inductance(target_electrical_rps) ;
    float mag_flux_linkage_q    = computed_inductance * m_amperage.q;



    // Calculate back EMF and other motor characteristics
    float back_emf_q_axis       = sensor_rad_per_sec 
                                * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);
    
    float desired_voltage_q     = m_current_limit * resistance(target_electrical_rps) 
                                + fabs(back_emf_q_axis) 
                                + velocity_correction;
    
    m_target_voltage_q          = symetric_clamp( desired_voltage_q, 
                                                  m_voltage_limit);                                         
    #endif


                                                
#if 0
//-----------------------------------------------------------------------------
//                              update_speed_closed_loop
//-----------------------------------------------------------------------------
void StepperMotor::update_speed_closed_loop(
                                                float target_mechanical_rps, 
                                                float delta_seconds)
{
    // Proportional gains (adjust these as needed)
    float Kp_velocity = 0.01f; // Initial gain for velocity correction
    
    if(FP_ZERO == fpclassify(target_mechanical_rps))
     {
       m_voltage.q  = 0.0f;
       m_voltage.d  = 0.0f;
       setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);

       return;
     }

     // Calculate the target electrical speed and estimate shaft speed
     float target_electrical_rps = mechanical_to_electrical_radians(target_mechanical_rps);
     float shaft_radians_per_sec = m_sensor.get_radians_per_second();
     float velocity_error        = target_mechanical_rps - shaft_radians_per_sec;
     float velocity_correction   = Kp_velocity * velocity_error;
     
     float mechanical_rps_cmd   = target_mechanical_rps + velocity_correction;   

     // Calculate desired q-axis voltage (without proportional corrections for now)
     float mag_flux_linkage_q = inductance(target_electrical_rps) * m_amperage.q;
     float back_emf_q_axis    = shaft_radians_per_sec * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);
     float desired_voltage_q  = m_current_limit * resistance(target_electrical_rps) 
                              ; //+ fabs(back_emf_q_axis)
                              ; //+ velocity_correction;

     // Increment mechanical radians based on command
     static float mechanical_radians(0.0f);
     //mechanical_radians   += target_mechanical_rps*delta_seconds;
     mechanical_radians     += mechanical_rps_cmd*delta_seconds;
     float alt_electric_cmd  = mechanical_to_electrical_radians(mechanical_radians);
     alt_electric_cmd        = _normalizeAngle(alt_electric_cmd);


    static float electrical_radians =
    get_electric_angle_radians();
    electrical_radians += mechanical_to_electrical_radians(mechanical_rps_cmd*delta_seconds);
    electrical_radians  = _normalizeAngle(electrical_radians);


    g_electrical_rad_cmd = electrical_radians;
    
    // CRITICAL: g_as5048_angle is now only updated by update_buffers() from SPI data
    // Do not overwrite it here - use the sensor value directly for calculations
    float current_mechanical_angle = normalize_radians((m_sensor.get_mechanical_phase_angle_radians()));
    g_electrical_rad_ref = normalize_radians(mechanical_to_electrical_radians(current_mechanical_angle));



    m_voltage.q     = symetric_clamp(desired_voltage_q, m_voltage_limit);

    // Dynamic smoothing based on speed
    float voltage_smoothing_rate = calculate_smoothing_rate(target_electrical_rps);
    
    // Smoothly adjust the voltage
    m_voltage.q = smooth_voltage_adjustment(m_voltage.q, 
                                            desired_voltage_q, 
                                            voltage_smoothing_rate, 
											delta_seconds);
    
    m_voltage.d     = 0.0f;

    setPhaseVoltage(m_voltage.q, m_voltage.d, electrical_radians);
    
    g_mag_flux_linkage_q = mag_flux_linkage_q;
    g_back_emf_q_axis    = back_emf_q_axis;               
}
#endif

//-----------------------------------------------------------------------------
//                              update_speed_closed_loop
//-----------------------------------------------------------------------------
#if 0 // backup of function before modification
void StepperMotor::update_speed_closed_loop(
                                                float target_mechanical_rps, 
                                                float delta_t)
{
    if(FP_ZERO == fpclassify(target_mechanical_rps))
     {
       m_voltage.q  = 0.0f;
       m_voltage.d  = 0.0f;
       setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);

       return;
     }

     float target_electrical_rps = mechanical_to_electrical_radians(target_mechanical_rps);


     float velocity_error = target_mechanical_rps - m_sensor.get_radians_per_second();



     static float mechanical_radians(0.0f);
     mechanical_radians += target_mechanical_rps*delta_t;
     
     mechanical_radians = normalize_radians(mechanical_radians);
     

    // Flux linkages have units of volt seconds
    // float mag_flux_linkage_d            = PHASE_INDUCTANCE * m_amperage.d;
    float mag_flux_linkage_q               = inductance(target_electrical_rps) * m_amperage.q;

    // float resistive_voltage_drop_d_axis = resistance(target_electrical_rps) * m_amperage.d;
    // float resistive_voltage_drop_q_axis = inductance(target_electrical_rps) * m_amperage.q;
    
    // float inductive_voltage_drop_d_axis = inductance(target_electrical_rps)  * amperage_d_dot;
    // float inductive_voltage_drop_q_axis = inductance(target_electrical_rps)  * amperage_q_dot;    
    
    float back_emf_q_axis  = m_omega_mechanical_rps 
                           * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

    // float back_emf_d_axis  = m_omega_mechanical_rps 
    //                        * (mag_flux_linkage_d + PERM_MAGNET_FLUX_LINKAGE);

    float voltage_q = m_current_limit * resistance(target_electrical_rps) 
                    + fabs(back_emf_q_axis);

    m_voltage.q     = symetric_clamp(voltage_q, m_voltage_limit);
    
    m_voltage.d     = 0.0f;
    setPhaseVoltage(m_voltage.q, m_voltage.d,  mechanical_to_electrical_radians(mechanical_radians));

    // m_amperage.q =  (m_voltage.q - fabs(back_emf_q_axis)) / resistance(target_electrical_rps);                         
    
#if 0
    m_amperage.d = m_amperage_prev.d
                + ( m_voltage.d -  m_voltage_prev.d ) * delta_t / inductance();

    m_amperage.q = m_amperage_prev.q
                + ( m_voltage.q -  m_voltage_prev.q ) * delta_t / inductance();


    m_amperage_prev.d      = m_amperage.d;
    m_amperage_prev.q      = m_amperage.q;

    m_voltage_prev.d       = m_voltage.d;
    m_voltage_prev.q       = m_voltage.q;
#endif

}

#endif











//-----------------------------------------------------------------------------
//                            update_position_closed_loop
//-----------------------------------------------------------------------------
void StepperMotor::update_position_closed_loop(
                                                      float target_mech_angle_radians, 
                                                      float delta_t)
{  
    
    
    //float angular_error = target_mech_angle_radians - m_sensor.get_angle_radians();

    //float v_pid = m_PID_angle.update(angular_error);

    float amperage_d_dot(0.0f);
    //float amperage_q_dot(0.0f);

    if(delta_t > 0.0f)
    {
        amperage_d_dot = (m_amperage.d - m_amperage_prev.d)/delta_t;
       // amperage_q_dot = (m_amperage.q - m_amperage_prev.q)/delta_t;
    }

    // Flux linkages have units of volt seconds
    //float mag_flux_linkage_d            = inductance(0.0f) * m_amperage.d;
    float mag_flux_linkage_q            = inductance(0.0f) * m_amperage.q;


    //float resistive_voltage_drop_d_axis = resistance(0.0f) * m_amperage.d;
    float resistive_voltage_drop_q_axis = resistance(0.0f) * m_amperage.q;


    float inductive_voltage_drop_d_axis = inductance(0.0f) * amperage_d_dot;
    //float inductive_voltage_drop_q_axis = inductance(0.0f) * amperage_q_dot;

    float electrical_rps                = mechanical_to_electrical_radians(m_omega_mechanical_rps);
    //float back_emf_d_axis               = electrical_rps * mag_flux_linkage_d;
    float back_emf_q_axis               = electrical_rps 
                                        * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);
    

#if 0
    // from basic principles
    float voltage_d                     = resistive_voltage_drop_d_axis
                                        + inductive_voltage_drop_d_axis
                                        - back_emf_d_axis;
    

     float voltage_q                    = resistive_voltage_drop_q_axis
                                        + inductive_voltage_drop_q_axis
                                        - back_emf_q_axis
                                        + v_pid;
#else
    // from original
    float voltage_d                     = -electrical_rps*inductive_voltage_drop_d_axis;


    float voltage_q                     = resistive_voltage_drop_q_axis
                                        + back_emf_q_axis;

#endif

     m_voltage.d = symetric_clamp( voltage_d, m_voltage_limit);   
     m_voltage.q = symetric_clamp( voltage_q, m_voltage_limit);

//     m_amperage.d = m_amperage_prev.d 
//                 + ( m_voltage.d -  m_voltage_prev.d ) * delta_t / inductance();
     
//     m_amperage.q = m_amperage_prev.q 
//                 + ( m_voltage.q -  m_voltage_prev.q ) * delta_t / inductance();

     float target_angle_electric =  mechanical_to_electrical_radians(target_mech_angle_radians);
     setPhaseVoltage(m_voltage.q, m_voltage.d, target_angle_electric);     
     
    // update state variables                   
    m_amperage_prev.d = m_amperage.d;
    m_amperage_prev.q = m_amperage.q;
    m_voltage_prev.d  = m_voltage.d;
    m_voltage_prev.q  = m_voltage.q;
}

//-----------------------------------------------------------------------------
//                         update_position_open_loop
//-----------------------------------------------------------------------------
void StepperMotor::update_position_open_loop(
                                                   float target_mechanical_radians, 
                                                   float delta_t)
{    
    const float MAX_RESISTIVE_VOLTAGE_DROP(m_current_limit*resistance(0.0f));

    // quick fix for strange cases (micros overflow + timestamp not defined)
    if(delta_t <= 0 || delta_t > 0.5f)
    {
      delta_t = 1e-3f;
    }

    // calculate the necessary angle to move from m_amperage position towards 
    // m_target angle with maximal velocity (m_velocity_limit)

    // Note: m_velocity_limit may have been modified by absoluteZeroSearch()
  
    // float target_electrical_radians = mechanical_to_electrical_radians(target_mechanical_radians);
   
    float current_angle;
    if(!m_sensor.fetch_radians(current_angle))
    {
        return;
    }
    
    float target_angle_error  = target_mechanical_radians -  current_angle;
    float max_angular_change  = m_velocity_limit * delta_t;

    float shaft_angle_command;

    if(!m_sensor.fetch_radians(shaft_angle_command))
    {
        return;
    }
        
    if( fabs( target_angle_error ) > fabs( max_angular_change ))
    {
      shaft_angle_command += _sign(target_angle_error) * max_angular_change;    
    }
    else
    {
      shaft_angle_command = target_mechanical_radians;
    }

    m_voltage.d = 0; // TODO d-component lag-compensation 

    // Flux linkages have units of volt seconds
    float mag_flux_linkage_q = inductance(0.0f) * m_amperage.q;

    float electrical_rps  = mechanical_to_electrical_radians(m_omega_mechanical_rps);
    float back_emf_q_axis = electrical_rps 
                          * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

    float voltage_q  = MAX_RESISTIVE_VOLTAGE_DROP + fabs(back_emf_q_axis);
    m_voltage.q = symetric_clamp(voltage_q, m_voltage_limit);
 
//    m_amperage.q = (m_voltage.q  - fabs(back_emf_q_axis)) / resistance(0.0f);

    float raw_elec_angle(mechanical_to_electrical_radians(shaft_angle_command));
    

//    m_amperage.d = m_amperage_prev.d 
//                + ( m_voltage.d -  m_voltage_prev.d ) * delta_t / inductance(0.0f);
    
//    m_amperage.q = m_amperage_prev.q 
 //               + ( m_voltage.q -  m_voltage_prev.q ) * delta_t / inductance(0.0f);

    raw_elec_angle -= m_radian_offset_to_electric_zero;        
    setPhaseVoltage(m_voltage.q, m_voltage.d, normalize_radians(raw_elec_angle));

    m_amperage_prev.d = m_amperage.d;
    m_amperage_prev.q = m_amperage.q;
    m_voltage_prev.d  = m_voltage.d;
    m_voltage_prev.q  = m_voltage.q;

    return;
}

//-----------------------------------------------------------------------------
//                                 update_speed_open_loop
//-----------------------------------------------------------------------------
void StepperMotor::update_speed_open_loop(
                                               float target_mechanical_rps, 
                                               float delta_t)
{
     if(FP_ZERO == fpclassify(target_mechanical_rps))
     {
       // DEBUG: Track how often we set angle to 0 due to zero target speed
       // This is one source of the many angle==0 conditions that cause spikes
       extern volatile uint32_t g_angle_zero_skip_counter;
       
       m_voltage.q  = 0.0f;
       m_voltage.d  = 0.0f;
       setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);  // <-- This causes angle==0!

       return;
     }

     float target_electrical_rps = mechanical_to_electrical_radians(target_mechanical_rps);

     static float mechanical_radians(0.0f);
     mechanical_radians += target_mechanical_rps*delta_t;
     
     mechanical_radians= normalize_radians(mechanical_radians);

    // Flux linkages have units of volt seconds
    // float mag_flux_linkage_d            = inductance(target_electrical_rps) * m_amperage.d;
    float mag_flux_linkage_q            = inductance(target_electrical_rps) * m_amperage.q;

    // float resistive_voltage_drop_d_axis = resistance(target_electrical_rps) * m_amperage.d;
    // float resistive_voltage_drop_q_axis = resistance(target_electrical_rps)  * m_amperage.q;
    
    // float inductive_voltage_drop_d_axis = inductance(target_electrical_rps) * amperage_d_dot;
    // float inductive_voltage_drop_q_axis = inductance(target_electrical_rps) * amperage_q_dot;    
    
    float back_emf_q_axis  = m_omega_mechanical_rps 
                           * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

    // float back_emf_d_axis  = m_omega_mechanical_rps 
    //                        * (mag_flux_linkage_d + PERM_MAGNET_FLUX_LINKAGE);

    float voltage_q = m_current_limit * resistance(target_electrical_rps) 
                    + fabs(back_emf_q_axis);

    m_voltage.q     = symetric_clamp(voltage_q, m_voltage_limit);
    
    m_voltage.d     = 0.0f;
    setPhaseVoltage(m_voltage.q, m_voltage.d, mechanical_to_electrical_radians(mechanical_radians));

    return;
}

// refactored on 12/2/2025
void StepperMotor::update_speed_closed_loop(float target_mechanical_rps,
                                            float delta_seconds)
{
    // DEBUG: Track ramped_speed vs epsilon to diagnose why angle becomes 0 so often
    extern volatile float g_debug_ramped_speed;
    extern volatile float g_debug_actual_target_rps;
    extern volatile uint32_t g_debug_epsilon_trigger_count;
    
#if 1  // DEBUG: Smooth ramp for open-loop startup testing only
    // Smooth ramp-up to target speed
    // CRITICAL: Initialize above epsilon threshold (1.0e-5) to avoid repeated angle==0 triggers during startup
    // The ramping increment is ACCEL_RATE * delta_seconds = 1.0 * 0.0001 = 0.0001 rad/s per iteration
    // Starting at 0.0 would cause ramped_speed to hover around epsilon for many iterations,
    // triggering setPhaseVoltage(0,0,0) repeatedly and causing spikes in g_park_sin/cos
    static float ramped_speed = 0.001f;  // Start at 0.001 rad/s (well above epsilon threshold)
#if 0  // Ultra-gentle acceleration for startup synchronization
    const float ACCEL_RATE = 0.25f; // rad/s² - extremely slow to allow rotor to follow
#else  // Faster acceleration (after motor is spinning)
    //const float ACCEL_RATE = 1.0f; // rad/s² - gentle acceleration
    const float ACCEL_RATE = 1.0f; // rad/s² - gentle acceleration
#endif
    
    // Ramp toward target
    if (ramped_speed < target_mechanical_rps)
    {
        ramped_speed += ACCEL_RATE * delta_seconds;
        if (ramped_speed > target_mechanical_rps)
            ramped_speed = target_mechanical_rps;
    }
    else if (ramped_speed > target_mechanical_rps)
    {
        ramped_speed -= ACCEL_RATE * delta_seconds;
        if (ramped_speed < target_mechanical_rps)
            ramped_speed = target_mechanical_rps;
    }
    
    // Use ramped speed instead of direct target
    const float actual_target_rps = ramped_speed;
#else
    // Normal operation - use target directly
    const float actual_target_rps = target_mechanical_rps;
#endif
    
    // Treat very small targets as "stop"
    const float EPSILON_SPEED_RPS(1.0e-4f);
    
    // DEBUG: Always capture these values to see the relationship
    g_debug_ramped_speed = ramped_speed;
    g_debug_actual_target_rps = actual_target_rps;

    if (fabsf(actual_target_rps) < EPSILON_SPEED_RPS)
    {
        // DEBUG: Track how often we set angle to 0 due to very small target speed
        // This is another source of the many angle==0 conditions that cause spikes
        // THE PROBLEM: ramped_speed starts at 0 and increments by ACCEL_RATE*delta_seconds
        // With ACCEL_RATE=1.0 and delta_seconds=0.0001, increment = 0.0001 rad/s
        // This is EXACTLY the epsilon threshold (1.0e-4), so ramped_speed hits this
        // condition repeatedly during startup, causing many angle==0 spikes!
        extern volatile uint32_t g_angle_zero_skip_counter;
        g_debug_epsilon_trigger_count++;
        
        m_voltage.q = 0.0f;
        m_voltage.d = 0.0f;

        // Park the stator field at 0 electrical when stopped
        setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);  // <-- This causes angle==0!
        
        g_target_rad_per_sec = actual_target_rps;

        return;
    }

    // Debug incoming parameters FIRST
    g_ramped_target_rps = ramped_speed; // Show actual ramped speed being used
    g_target_rad_per_sec = target_mechanical_rps; // Show commanded target speed
    
    if (delta_seconds <= 0.0f)
    {
        return;
    }

    // ------------------------------------------------------------------------
    // 1) Use ramped speed for smooth acceleration
    // ------------------------------------------------------------------------
    const float target_mechanical_rad_per_sec = actual_target_rps;
    
    // Update debug variables
    g_loop_counter++;
    g_speed_update_counter++;

    // ------------------------------------------------------------------------
    // 2) Target and measured mechanical speed (radians/sec)
    // ------------------------------------------------------------------------

    const float current_mechanical_radians =
        m_sensor.get_mechanical_phase_angle_radians();

#if 0  // Closed-loop with sensor feedback
    static float previous_mechanical_radians(0.0f);
    static bool  have_previous_sample(false);

    float measured_mechanical_rad_per_sec(0.0f);

    if (have_previous_sample)
    {
        // Shortest signed mechanical delta (in radians)
        const float delta_mechanical_radians =
            normalize_radians(current_mechanical_radians
                              - previous_mechanical_radians);

        measured_mechanical_rad_per_sec =
            delta_mechanical_radians / delta_seconds;
    }
    else
    {
        have_previous_sample = true;
    }

    previous_mechanical_radians = current_mechanical_radians;

    // ------------------------------------------------------------------------
    // 3) Compute feedforward + feedback voltage
    // ------------------------------------------------------------------------
    const float target_electrical_rad_per_sec = 
        mechanical_to_electrical_radians(target_mechanical_rad_per_sec);
    
    // Use lower current for voltage calculation to avoid saturation at startup
    const float STARTUP_CURRENT_LIMIT = 0.5f; // Amps - reduced from 2A
    
    // Base voltage for target speed (feedforward)
    const float mag_flux_linkage_q = inductance(target_electrical_rad_per_sec) * m_amperage.q;
    const float back_emf_q_axis = target_electrical_rad_per_sec 
                                * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);
    
    // m_current_limit
    float base_voltage_q = STARTUP_CURRENT_LIMIT * resistance(target_electrical_rad_per_sec) 
                         + fabs(back_emf_q_axis);

    // Feedback correction (mechanical domain)
    const float Kp_velocity_mech = 0.5f; 
    const float velocity_error_mech_rad_per_sec =
        target_mechanical_rad_per_sec - measured_mechanical_rad_per_sec;
    
    float desired_voltage_q = base_voltage_q + (Kp_velocity_mech * velocity_error_mech_rad_per_sec);

    // Clamp to available DC bus / configured limit
    m_voltage.q = symetric_clamp(desired_voltage_q, m_voltage_limit);
    m_voltage.d = 0.0f; // no field weakening for now
#else  // Pure open-loop (for when sensor is not working)

    const float velocity_error_mech_rad_per_sec = 0.0;
    // ------------------------------------------------------------------------
    // 3) PURE OPEN-LOOP VOLTAGE (sensor broken, returns constant value)
    // ------------------------------------------------------------------------
    
    // Simple voltage calculation for open-loop operation
#if 1  // Higher voltage for stronger torque
    const float OPEN_LOOP_VOLTAGE = 10.0f; // Increased to overcome cogging torque
#else  // Lower voltage for gentler operation
    const float OPEN_LOOP_VOLTAGE = 6.0f; // Reduced voltage
#endif
    
    m_voltage.q = OPEN_LOOP_VOLTAGE;
    m_voltage.d = 0.0f;
#endif

    // ------------------------------------------------------------------------
    // 4) Advance electrical angle based on target velocity (not current position)
    // ------------------------------------------------------------------------
    // Simply advance angle based on commanded speed (open-loop angle generation)
    const float angle_increment = target_mechanical_rad_per_sec * delta_seconds;
    
    // Capture state before accumulation
    g_angle_before_add = m_accumulated_mechanical_radians;
    g_angle_increment = angle_increment;
    
    // Accumulate angle
    m_accumulated_mechanical_radians += angle_increment;
    m_accumulated_mechanical_radians = normalize_radians(m_accumulated_mechanical_radians);
    
    // Capture state after accumulation
    g_angle_after_add = m_accumulated_mechanical_radians;
    
    // Debug outputs
    g_accumulated_mech_rad = m_accumulated_mechanical_radians;
    g_velocity_correction = angle_increment * 1000.0f; // Show increment scaled up
    g_mech_angle_for_foc = m_accumulated_mechanical_radians;
    
    // Convert accumulated angle directly to electrical (no offset subtraction needed)
    float electrical_angle_radians =
        mechanical_to_electrical_radians(m_accumulated_mechanical_radians);

    electrical_angle_radians =
        normalize_radians(electrical_angle_radians);
    
    g_elec_angle_for_foc = electrical_angle_radians;

    // ------------------------------------------------------------------------
    // 5) Apply voltage in dq frame at the correct electrical angle
    // ------------------------------------------------------------------------
    setPhaseVoltage(m_voltage.q,
                    m_voltage.d,
                    electrical_angle_radians);

    // ------------------------------------------------------------------------
    // 6) Debug / telemetry (optional globals)
    // ------------------------------------------------------------------------
    // CRITICAL: g_as5048_angle is now only updated by update_buffers() from SPI data
    // Do not overwrite it here
    g_electrical_rad_ref  = electrical_angle_radians;
    g_velocity_correction = velocity_error_mech_rad_per_sec; // "error" here
    g_amperage_q          = m_amperage.q;
    g_voltage_q           = m_voltage.q;
    g_voltage_d           = m_voltage.d;
}



//-----------------------------------------------------------------------------
//                              clarkeTransform
//-----------------------------------------------------------------------------
void StepperMotor::clarkeTransform( float Ia, 
                            float  Ib, 
                            float& Ialpha, 
                            float& Ibeta) 
{
    Ialpha = Ia;
    Ibeta = (Ia + 2.0f * Ib) * SQUARE_ROOT_OF_3_INVERSE;
}

//-----------------------------------------------------------------------------
//                              clarkeTransform
//
// Function to perform Park Transformation (Rotation)
//-----------------------------------------------------------------------------
void StepperMotor::parkTransform( float  Ialpha, 
                          float  Ibeta, 
                          float  theta, 
                          float& Id, 
                          float& Iq) 
{
    Id =  Ialpha * cosf(theta) + Ibeta * sinf(theta);
    Iq = -Ialpha * sinf(theta) + Ibeta * cosf(theta);
    
    //Id =  Ialpha * arm_cos_f32(theta) + Ibeta * arm_sin_f32(theta);
    //Iq = -Ialpha * arm_sin_f32(theta) + Ibeta * arm_cos_f32(theta);
}

//-----------------------------------------------------------------------------
//                              transformCurrents
//-----------------------------------------------------------------------------
void StepperMotor::transformCurrents( 
                               float  Ia, 
                               float  Ib, 
                               float  encoder_angle_radians, 
                               float& Id, 
                               float& Iq) 
{
    float Ialpha, Ibeta;

    // Perform Clarke Transformation
    clarkeTransform(Ia, Ib, Ialpha, Ibeta);

    // Convert encoder angle to radians
    float theta = encoder_angle_radians; // * MY_PI / 180.0f;

    // Perform Park Transformation
    parkTransform(Ialpha, Ibeta, theta, Id, Iq);
}


//-----------------------------------------------------------------------------
//                        compute_inverse_park_transform
//-----------------------------------------------------------------------------
void StepperMotor::
compute_inverse_park_transform( 
                                float Uq, 
                                float Ud, 
                                float electric_angle)
{
    g_park_transform_counter++;
    
    // Sinusoidal PWM modulation
    // Inverse Park transformation
    float _sa;
    float _ca;
    
    // CRITICAL: The angle passed in should already be in [0, 2π) range
    // Don't re-normalize unless absolutely necessary, as it might introduce errors
    // DEBUG: Capture BOTH the input angle and validated angle to see what's happening
    extern volatile float g_debug_electric_angle_for_sincos;
    extern volatile float g_debug_electric_angle_input;
    g_debug_electric_angle_input = electric_angle;  // Capture input angle
    
    // DEBUG: Capture input angle to verify it matches g_elec_angle_for_foc
    extern volatile float g_debug_electric_angle_input;
    g_debug_electric_angle_input = electric_angle;  // Capture raw input
    
    // CRITICAL: Normalize the angle first to catch cases where angle wraps to 0.0f (e.g., 2π → 0)
    // Then check if normalized angle is 0.0f - if so, skip ALL sin/cos calculations
    // When angle == 0.0f: cos(0) = 1.0 and sin(0) = 0.0, causing jumps from smooth sine wave
    // We must NOT calculate sin(0) or cos(0) to keep signals smooth
    float validated_angle = normalize_radians(electric_angle);
    
    // CRITICAL: Due to floating point precision, fmod(2π, 2π) might not return exactly 0.0f
    // It might return something like -0.0000001f or 0.0000001f, which normalizes to ~0 or ~2π
    // We need to check if the angle is effectively 0 by checking if it's exactly 0.0f OR exactly 2π
    // OR if it's very close to 0 (which would produce sin/cos values very close to sin(0)/cos(0))
    // When angle == 0.0f or angle == 2π: cos(0) = 1.0 and sin(0) = 0.0, causing jumps from smooth sine wave
    // We must NOT calculate sin(0) or cos(0) to keep signals smooth
    // Check for exactly 0.0f, exactly 2π, or if normalized result is exactly 2π (which means it wrapped from 0)
    // NOTE: In normal operation, angle should only be 0 once per revolution - if we see many 0s,
    // there's a bug elsewhere that's incorrectly setting angle to 0.0f
    if(validated_angle == 0.0f || validated_angle == TWO_PI)
    {
        // Angle is exactly 0.0f or 2π (or normalized to exactly 0.0f or 2π) - skip all calculations to avoid sin(0) and cos(0)
        // This keeps g_park_cos, g_park_sin, and m_U_beta at their previous smooth values
        // DEBUG: Track how often this happens - should be ~once per revolution in normal operation
        g_angle_zero_skip_counter++;
        g_debug_angle_when_zero = electric_angle;  // Capture the original input angle
        return;
    }
    
    
    // Angle is non-zero - proceed with sin/cos calculations
    bool angle_is_nonzero = true;  // We already checked above
    
    g_debug_electric_angle_for_sincos = validated_angle;  // Capture angle used for sin/cos
    
    // TEST: Try standard library functions to see if lookup table is the issue
    // CRITICAL: Verify cosf is working correctly - it should return values in [-1, 1] range
    // If validated_angle goes from 0 to 2π, cosf should go from 1 → 0 → -1 → 0 → 1
    // DEBUG: Capture angle IMMEDIATELY before cosf() call to verify relationship
    extern volatile float g_debug_validated_angle_at_cosf;
    g_debug_validated_angle_at_cosf = validated_angle;
    
    // Use sinf() for sine - it works correctly
    // CRITICAL: We only reach here if angle != 0.0f, so we never calculate sin(0) or cos(0)
    _sa = sinf(validated_angle);
    
    // DEBUG: Capture angle before _cos call
    // CRITICAL: Only update debug variables if angle is non-zero to avoid
    // polluting debug data with zero values from disable/init calls
    extern volatile float g_debug_angle_before_cos;
    if(angle_is_nonzero)
    {
        g_debug_angle_before_cos = validated_angle;
    }
    
    // For cosine, let's compute it manually using sin(θ + π/2) to see what's happening
    // We'll trace through the exact same logic as _cos but with debug variables
    float cos_input_angle = validated_angle;
    
    // Normalize if needed (same as _cos does)
    if(cos_input_angle < 0.0f || cos_input_angle >= TWO_PI)
    {
        float normalized = fmodf(cos_input_angle, TWO_PI);
        cos_input_angle = normalized >= 0.0f ? normalized : (normalized + TWO_PI);
    }
    
    // Compute a_sin = angle + π/2 (same as _cos does)
    float a_sin = cos_input_angle + HALF_PI;
    
    // Wrap to [0, 2π) (same as _cos does)
    if(a_sin >= TWO_PI)
    {
        a_sin = a_sin - TWO_PI;
    }
    if(a_sin < 0.0f)
    {
        a_sin = a_sin + TWO_PI;
    }
    
    // Compute cosine using sinf() with the wrapped angle
    float cos_via_sinf = sinf(a_sin);
    
    // Also try _cos for comparison
    _ca = _cos(validated_angle);
    
    // DEBUG: Only update debug variables if angle is non-zero to avoid
    // polluting debug data with zero values from disable/init calls
    extern volatile float g_debug_a_sin_before_wrap;
    extern volatile float g_debug_a_sin_after_wrap;
    extern volatile float g_debug_cos_via_sinf;
    if(angle_is_nonzero)
    {
        g_debug_a_sin_before_wrap = cos_input_angle + HALF_PI;  // Before wrapping
        g_debug_a_sin_after_wrap = a_sin;
        g_debug_cos_via_sinf = cos_via_sinf;
    }
    
    // DEBUG: Capture intermediate values for cosine calculation
    extern volatile float g_debug_cos_angle_before_wrap;
    extern volatile float g_debug_cos_angle_before_wrap_check;
    extern volatile bool g_debug_cos_angle_needs_wrap;
    extern volatile float g_debug_cos_angle_after_wrap;
    extern volatile float g_debug_cos_angle_before_sinf;
    float cos_angle_unwrapped = validated_angle + HALF_PI;
    float cos_angle = cos_angle_unwrapped;
    while(cos_angle >= TWO_PI) cos_angle = cos_angle - TWO_PI;
    while(cos_angle < 0.0f) cos_angle = cos_angle + TWO_PI;
    g_debug_cos_angle_before_wrap = cos_angle_unwrapped;
    g_debug_cos_angle_before_wrap_check = cos_angle_unwrapped;
    g_debug_cos_angle_needs_wrap = (cos_angle_unwrapped >= TWO_PI);
    g_debug_cos_angle_after_wrap = cos_angle;
    g_debug_cos_angle_before_sinf = cos_angle;
    
    // DEBUG: Verify the phase relationship is correct
    extern volatile float g_debug_sinf_result;
    extern volatile float g_debug_cosf_result;
    g_debug_sinf_result = _sa;
    g_debug_cosf_result = _ca;  // This should now show -1 to 1 range and oscillate around 0
    //arm_sin_cos_f32(electric_angle, &_sa, &_ca);
    //sincos(electric_angle, &_sa, &_ca);
    
    // Debug: capture sin/cos values
    // CRITICAL: We only reach here if angle != 0.0f (early return above handles angle == 0.0f)
    // So we can always update since we never calculate sin(0) or cos(0)
    g_park_sin = _sa;
    g_park_cos = _ca;
    
    // Inverse park transform
    // DEBUG: Capture inputs to inverse park transform
    extern volatile float g_debug_Uq_input;
    extern volatile float g_debug_Ud_input;
    extern volatile float g_debug_U_beta_calc_sa;
    extern volatile float g_debug_U_beta_calc_ca;
    extern volatile float g_debug_U_beta_term1;  // _sa * Ud
    extern volatile float g_debug_U_beta_term2;  // _ca * Uq
    if(angle_is_nonzero)
    {
        g_debug_Uq_input = Uq;
        g_debug_Ud_input = Ud;
        g_debug_U_beta_calc_sa = _sa;
        g_debug_U_beta_calc_ca = _ca;
        g_debug_U_beta_term1 = _sa * Ud;
        g_debug_U_beta_term2 = _ca * Uq;
    }
    
    // Calculate m_U_beta step by step for debugging
    // CRITICAL: Capture the exact values AT THE MOMENT OF CALCULATION
    // Only capture when angle is non-zero to avoid polluting with cos(0)=1 values
    extern volatile float g_debug_sa_for_term1;
    extern volatile float g_debug_Ud_for_term1;
    extern volatile float g_debug_ca_for_term2;
    extern volatile float g_debug_Uq_for_term2;
    extern volatile float g_debug_ca_at_calc;
    extern volatile float g_debug_sa_at_calc;
    extern volatile float g_debug_Ud_at_calc;
    extern volatile float g_debug_Uq_at_calc;
    
    // Capture ALL values at the exact same moment, right before calculation
    // CRITICAL: Guard debug variables to prevent zero-sample pollution (cos(0)=1)
    // Only update when angle is non-zero, same as g_park_cos
    if(angle_is_nonzero)
    {
        g_debug_sa_for_term1 = _sa;
        g_debug_Ud_for_term1 = Ud;
        g_debug_ca_for_term2 = _ca;
        g_debug_Uq_for_term2 = Uq;
        g_debug_ca_at_calc = _ca;  // Should match g_debug_ca_for_term2 and g_park_cos
        g_debug_sa_at_calc = _sa;  // Should match g_debug_sa_for_term1 and g_park_sin
        g_debug_Ud_at_calc = Ud;   // Should match g_debug_Ud_for_term1
        g_debug_Uq_at_calc = Uq;   // Should match g_debug_Uq_for_term2
    }
    
    float term1_calc = _sa * Ud;
    float term2_calc = _ca * Uq;
    extern volatile float g_debug_term1_direct;
    extern volatile float g_debug_term2_direct;
    extern volatile float g_debug_term1_verify;
    extern volatile float g_debug_term2_verify;
    extern volatile float g_debug_m_U_beta_calc_direct;
    extern volatile float g_debug_m_U_beta_calc_verify;
    
    // Calculate m_U_beta using the same method as g_debug_m_U_beta_calc_direct
    // This ensures m_U_beta matches the smooth sine wave calculation
    float m_U_beta_calc = term1_calc + term2_calc;
    
    // CRITICAL: Store clean U_beta value for debug plots (only when angle is non-zero)
    // This prevents zero-sample pollution (cos(0)=1) from making the plots look wrong
    extern volatile float g_clean_U_beta_for_plot;
    if(angle_is_nonzero)
    {
        g_clean_U_beta_for_plot = m_U_beta_calc;
    }
    
    // Only update debug variables when angle is non-zero (same guard as above)
    // This prevents zero-sample pollution (cos(0)=1) from making the plots look wrong
    if(angle_is_nonzero)
    {
        g_debug_term1_direct = term1_calc;
        g_debug_term2_direct = term2_calc;
        
        // DEBUG: Verify the calculation step by step
        g_debug_term1_verify = g_debug_sa_for_term1 * g_debug_Ud_for_term1;
        g_debug_term2_verify = g_debug_ca_for_term2 * g_debug_Uq_for_term2;
        
        // Calculate the sum directly for comparison (inside guard for debug)
        float m_U_beta_calc_debug = term1_calc + term2_calc;
        g_debug_m_U_beta_calc_direct = m_U_beta_calc_debug;
        
        // DEBUG: Also calculate using the verified terms
        g_debug_m_U_beta_calc_verify = g_debug_term1_verify + g_debug_term2_verify;
    }
    
    // CRITICAL: We only reach here if angle != 0.0f (early return above handles angle == 0.0f)
    // So we can always update since we never calculate sin(0) or cos(0)
    // Motor controller needs smooth values, not discontinuities
    m_U_alpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
    m_U_beta = term1_calc + term2_calc;  // Identical calculation to g_debug_m_U_beta_calc_direct
    
    // DEBUG: Capture m_U_beta immediately after calculation for comparison
    // Always capture (no guard) to see all values including zeros
    extern volatile float g_debug_m_U_beta_after_calc;
    g_debug_m_U_beta_after_calc = m_U_beta;
    
    // DEBUG: Compare the direct calculation with the member variable
    extern volatile float g_debug_m_U_beta_diff;
    g_debug_m_U_beta_diff = m_U_beta - m_U_beta_calc;
    
    //m_U_alpha = -sinf(electric_angle) * Uq;
    //m_U_beta  =  cosf(electric_angle) * Uq;
    
    return;
}


//-----------------------------------------------------------------------------
//                         setPhaseVoltage
//
// Method using FOC to set Uq and Ud to the motor at the optimal angle
// 
// Function implementing Sine PWM algorithms
// - space vector not implemented yet
//
// Function uses sine approximation
//    regular  sin +  cos ~300us    (no memory usaage)
//    approx  _sin + _cos ~110us    (400Byte ~ 20% of memory)
//
//! @param Uq
//! @param Ud
//! @param angle_el
//-----------------------------------------------------------------------------
void StepperMotor::
setPhaseVoltage(float Uq, float Ud, float electric_angle) 
{
  g_motor_enabled = m_enabled;
  
  // CRITICAL: Always compute the inverse park transform for debugging,
  // even if motor is disabled, so we can see the sin/cos values
  // CRITICAL: electric_angle is already normalized in update_speed_open_loop
  // Pass it through directly without re-normalization to avoid introducing errors
  // The multiple normalization checks were causing the angle to be incorrectly modified
  compute_inverse_park_transform( Uq, Ud, electric_angle);
  
  if(m_enabled)
  { 
      // set the voltages in hardware
      //m_driver.set_pwm_duty_cycle(m_U_alpha, m_U_beta);
      
      m_driver.set_pwm_duty_cycle( m_U_alpha, 
                                   m_U_beta, 
                                   m_hifactor_a, 
                                   m_lofactor_a, 
                                   m_hifactor_b, 
                                   m_lofactor_b);
  }
  
  // Debug: monitor alpha/beta outputs
  // CRITICAL: Always capture m_U_beta right before assignment to g_U_beta
  // This will show if m_U_beta is correct when assigned
  extern volatile float g_debug_m_U_beta_before_assign;
  extern volatile float g_debug_m_U_beta_at_assign;
  g_debug_m_U_beta_before_assign = m_U_beta;  // Always capture for debugging
  
  // Guard debug variables to prevent zero-sample pollution in plots
  // Only update debug variables when angle != 0.0f, same as g_park_cos and g_debug_ca_at_calc
  extern volatile float g_clean_U_beta_for_plot;
  bool angle_is_nonzero_setphase = (electric_angle != 0.0f);
  if(angle_is_nonzero_setphase)
  {
      g_debug_m_U_beta_at_assign = m_U_beta;
  }
  
  // CRITICAL: Only update g_U_beta when angle != 0 to match g_debug_m_U_beta_calc_direct behavior
  // Updating when angle=0 causes discontinuity because m_U_beta = Uq at angle=0, which jumps from smooth sine wave
  // g_debug_m_U_beta_calc_direct only updates when angle != 0, so g_U_beta must do the same
  // CRITICAL: Use same condition as m_U_beta and g_park_cos for consistency
  // Use the same validated_angle check that compute_inverse_park_transform uses
  g_U_alpha = m_U_alpha;
  
  // CRITICAL: Always update g_U_beta from m_U_beta to ensure it reflects the calculated value
  // The conditional update in compute_inverse_park_transform already handles the angle=0 case
  // We need g_U_beta to always show what m_U_beta contains (which is only updated when angle != 0)
  g_U_beta = m_U_beta;
}

//-----------------------------------------------------------------------------
//                              angleOpenloop
//
// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and m_velocity_limit variables
//
// TODO: Remove magic numbers
//-----------------------------------------------------------------------------
float StepperMotor::angleOpenloop(float target_angle)
{
    // get m_amperage timestamp
    unsigned long now_us = micros();
  
    // calculate the sample time from last call
    float timestamp = (now_us - m_prev_open_loop_timestamp_us) * 1e-6f;
    // quick fix for strange cases (micros overflow + timestamp not defined)
    if(timestamp <= 0 || timestamp > 0.5f)
    {
      timestamp = 1e-3f;
    }

    // calculate the necessary angle to move from m_amperage position towards 
    // m_target angle with maximal velocity (m_velocity_limit)

    // Note: m_velocity_limit may have been modified by absoluteZeroSearch()

    
    float shaft_radians;
    if(!m_sensor.fetch_radians(shaft_radians))
    {
        return 0.0f;
    }
  
    float target_angle_error(   target_angle - shaft_radians );
    float delta_angle_measured(     m_velocity_limit * timestamp    );
  
    if( abs( target_angle_error ) > abs( delta_angle_measured ))
    {
      m_shaft_angle += _sign(target_angle_error) * (abs( m_velocity_limit ) * timestamp);
    
      m_omega_mechanical_rps = m_velocity_limit;       
    }
    else
    {
      m_shaft_angle          = target_angle;
      m_omega_mechanical_rps = 0;   
    }

    // If PHASE_RESISTANCE is set use m_voltage_limit, otherwise use m_current_limit
    float Uq = m_voltage_limit;
    if(NOT_SET != resistance(0.0f))
    {
      // in voltage mode
      // the voltage limit is equal to the current limit * the phase resistance
      Uq = m_current_limit*resistance(0.0f) + fabs(m_voltage_bemf);
      Uq = symetric_clamp(Uq, m_voltage_limit);
 
 //     m_amperage.q = (Uq - fabs(m_voltage_bemf)) / resistance(0.0f);
    }
  
    // set the maximal allowed m_voltage (voltage_limit) with the necessary angle
    setPhaseVoltage(Uq,  0, mechanical_to_electrical_radians(m_shaft_angle));
 
    m_prev_open_loop_timestamp_us = now_us;  // save timestamp for next call

    return Uq;
}

//-----------------------------------------------------------------------------
//                         get_filtered_shaft_angle
//-----------------------------------------------------------------------------
float StepperMotor::get_filtered_shaft_angle()
{
    // float radians = m_LPF_angle( m_sensor.get_angle_radians());
    float radians;
    if(m_sensor.fetch_radians(radians))
    {
        if(Direction::CCW == m_sensor_direction)
        {
           radians *= -1.0f;
        }

        // m_sensor_offset is currently 0
        return  radians - m_sensor_offset;
    }
    else
    {
       return 0.0f;
    }
}

//-----------------------------------------------------------------------------
//                              convert_count_to_shaft_angle
//-----------------------------------------------------------------------------
float StepperMotor::convert_count_to_shaft_angle(uint16_t count)
{
    const uint16_t COUNTS_PER_REVOLUTION(0x4000);
    
    float radians = TWO_PI
                  * static_cast<float>(count)
                  / static_cast<float>(COUNTS_PER_REVOLUTION);
    
    //float result = (m_sensor.is_direction_invert())
    //             ? -radians
    //             :  radians;

 //   if(Direction::CCW == m_sensor_direction)
 //   {
 //      radians *= -1.0f;
 //   }

    // m_sensor_offset is currently 0
    return  /*normalize_radians*/(radians - m_sensor_offset);
}

#if 0
//-----------------------------------------------------------------------------
//                         shaft_radians_per_second
//-----------------------------------------------------------------------------
float StepperMotor::shaft_radians_per_second() 
{
  float rad_per_sec = m_LPF_velocity(m_sensor.get_radians_per_second());
  
  return (Direction::CCW ==m_sensor_direction) ? -rad_per_sec : rad_per_sec;
  //return rad_per_sec;
}
#endif

//-----------------------------------------------------------------------------
//                        get_phase_angle_radians (electrical)
//-----------------------------------------------------------------------------
float StepperMotor::get_electric_angle_radians()
{
    //float direction       = static_cast<float>(m_sensor_direction);
    
    //float electic_radians = mechanical_to_electrical_radians(
    //                           m_sensor.get_mechanical_phase_angle_radians()) ;

    float electic_radians = mechanical_to_electrical_radians(
                                m_sensor.get_mechanical_phase_angle_radians()) ;

    // m_radian_offset_to_electric_zero is already in electrical radians
    float raw_angle = electic_radians - m_radian_offset_to_electric_zero;     
    
    return normalize_radians( raw_angle );
}

//-----------------------------------------------------------------------------
//                          rotorFieldAlignment
//-----------------------------------------------------------------------------
void StepperMotor::rotorFieldAlignment(double targetPosition) 
{
    // Initialize motor and position sensor
    float currentPosition = 0.0f; // Current rotor position (radians)
    float voltage         = 0.0f; // Motor voltage (V)
    
    // Specify the alignment speed and acceleration
    float alignmentSpeed        = 2 * MY_PI; // radians per second
 //   float alignmentAcceleration = MY_PI;     // radians per second squared


const int ITERATION_LIMIT(100);
 int iteration_cont(0);
 
    // Loop until the target position is reached
    while (( std::abs(currentPosition - targetPosition) > 0.01) 
            && (iteration_cont++ < ITERATION_LIMIT)
            )
    {
        // Calculate error between current and target position
        float positionError = targetPosition - currentPosition;
        
        // Calculate the required torque (proportional control)
        float torque = positionError * m_current_limit;
        
        // Apply voltage to align the rotor field
        voltage = torque / (alignmentSpeed * NUM_POLE_PAIRS);
        
        // Ensure voltage doesn't exceed the maximum
        voltage = symetric_clamp(voltage, m_voltage_limit);
        
        // Simulate motor response (in a real system, this would control the motor)
        // Update the current position (this is simplified)
        //currentPosition += alignmentSpeed * 0.001; // Assuming a 1 ms time step
       // m_sensor.update();
        currentPosition = m_sensor.read_angle_radians();
        
        // Print current position for demonstration purposes
        //char  buff[128];
        //sprintf(buff, "Current Position: %ld radians\r\n", static_cast<uint32_t>(1000.0f *currentPosition));
       // HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(buff), strlen(buff), HAL_MAX_DELAY);

        HAL_Delay(100);

           
        setPhaseVoltage(voltage, 0.0f, mechanical_to_electrical_radians(currentPosition));

        
    }
    
    // Stop motor (set voltage to zero)
    voltage = 0.0f;
    
    // Print message when alignment is complete
    //char  buff[128];
    //sprintf(buff, "Rotor Field Alignment Complete\r\n");
    //HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(buff), strlen(buff), HAL_MAX_DELAY);
    
}


