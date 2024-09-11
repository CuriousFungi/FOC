#include "StepperMotor.hpp"
//#include "./communication/SimpleFOCDebug.hpp"



#include "../sensors/AS5048A.hpp"
#include "../../Inc/EnumValue.hpp"
#include "../../Inc/clamp.hpp"


#include <math.h>
#include <limits>

volatile float g_shaft_angle = 1.1f;
volatile float g_mechanical_angle = 2.2f;
volatile float g_mag_flux_linkage_q = 3.3f;
volatile float g_back_emf_q_axis = 4.4f;




// Example usage
#if 0
int main() {
    OffsetEstimator estimator(10);  // Moving average window size of 10

    // Simulated current measurements
    float currents[] = {2.6, 2.4, 2.55, 2.45, 2.5, 2.6, 2.4, 2.55, 2.45, 2.5};
    
    for (float current : currents) {
        float offset = estimator.update(current);
        std::cout << "Updated Offset: " << offset << std::endl;
    }

    return 0;
}
#endif


//extern volatile uint16_t* adc_dma_result;
//extern int adc_channel_count;
//extern uint8_t adc_conv_complete_flag;




constexpr double sqrtNewtonRaphson(double x, double curr, double prev) 
{
    return (curr == prev)
           ? curr
           : sqrtNewtonRaphson(x, 0.5 * (curr + x / curr), curr);
}


constexpr double compile_time_sqrt(double x) 
{
    return ((x >= 0) && (x < std::numeric_limits<double>::infinity()))
           ? sqrtNewtonRaphson(x, x, 0)
           : std::numeric_limits<double>::quiet_NaN();
}


//=============================================================================
//                          StepperMotor Class
//=============================================================================
StepperMotor::StepperMotor(SPI_HandleTypeDef* hspi, 
                                GPIO_TypeDef*      p_sensor_chip_select_port,
                                uint16_t           sensor_chip_select_pin,
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
:     PI( 3.14159265358979323846f)
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
,     PHASE_INDUCTANCE_1KHZ(3.9f)
,     PHASE_INDUCTANCE_10KHZ(3.2)

,     MIN_ALIGN_ANGLE_DETECT_MOVEMENT(0.1f)       // Minimum angle to detect movement, adjust as needed
,     MAX_SENSOR_ANGLE(TWO_PI)     // Assuming 360 degree range for the sensor
,     NUM_STEPS(100)
,     ROTATION_ANGLE(PI / 2.0f)  // 90 degrees in radians
,     STEP_SIZE(ROTATION_ANGLE / static_cast<float>(NUM_STEPS) )
,     SQUARE_ROOT_OF_3_INVERSE(1.0f/static_cast<float>(compile_time_sqrt(3.0)))
,     PERM_MAGNET_FLUX_LINKAGE(0.171f)
,     m_sensor(hspi,
               p_sensor_chip_select_port, 
		       sensor_chip_select_pin)
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
,   m_omega_mechanical_rps(0.0f)
,   m_current_sp(0.0f)
,   m_shaft_velocity_target(0.0f)
,   m_shaft_angle_target(0.0f)
,   m_voltage()
,   m_voltage_prev()
,   m_amperage()
,   m_amperage_prev()
,   m_winding_amperage_a(0.0f)
,   m_winding_amperage_b(0.0f)
,   m_voltage_bemf(0)
,   m_voltage_sensor_align(3.0f)   //power_supply_voltage)
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
                      
,   m_PID_velocity(   0.2f,
                      20.0f,
                      0.001f,
                      DEF_PID_VEL_RAMP,
                      DEF_PID_VEL_LIMIT)
                      
,   m_PID_angle(      DEF_P_ANGLE_P,
                      0,
                      0,
                      0,
                      DEF_VEL_LIM)
                      
,   m_LPF_current_winding_a(  DEF_CURR_FILTER_Tf)
,   m_LPF_current_winding_b(  DEF_CURR_FILTER_Tf)

,   m_LPF_current_q(  DEF_CURR_FILTER_Tf)
,   m_LPF_current_d(  DEF_CURR_FILTER_Tf)
,   m_LPF_velocity(   DEF_VEL_FILTER_Tf)
,   m_LPF_angle(      DEF_VEL_FILTER_Tf)  

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


{
    return;
}

//-----------------------------------------------------------------------------
//                               init
//-----------------------------------------------------------------------------
void StepperMotor::init() 
{    
    m_motor_status       = FOC_MOTOR_STATUS::INITIALIZING;

    m_PID_angle.limit      = m_velocity_limit;
    
#if 0 // This seems like a bad idea
    // if using open loop control, set a CW as the default direction if not already set
    if (  (  m_motion_control == MOTION_CONTROL_TYPE::OL_ANGLE
             ||
		     m_motion_control == MOTION_CONTROL_TYPE::OL_VELOCITY
		  )
          &&
		  (m_sensor_direction == Direction::UNKNOWN))
    {
        m_sensor_direction = Direction::CW;
    }
#endif

    HAL_Delay(500);
    enable();   // enable motor
    HAL_Delay(500);

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

float StepperMotor::inductance(float electical_radians_per_second)
{
  //  return PHASE_INDUCTANCE_10KHZ;

    const float ONE_KHZ_N_RPS(1000  * TWO_PI);
    const float TEN_KHZ_N_RPS(10000 * TWO_PI);


    const float SLOPE = (PHASE_INDUCTANCE_10KHZ - PHASE_INDUCTANCE_1KHZ)
                      / (TEN_KHZ_N_RPS - ONE_KHZ_N_RPS);


    float inductance(0.0f);

    if(electical_radians_per_second < ONE_KHZ_N_RPS)
    {
        inductance = PHASE_INDUCTANCE_1KHZ;
    }
    else
    if(electical_radians_per_second > TEN_KHZ_N_RPS)
    {
        inductance = PHASE_INDUCTANCE_10KHZ;
    }
    else
    {
        inductance = PHASE_INDUCTANCE_1KHZ
                   + (SLOPE * electical_radians_per_second); 
    }

    return inductance;
}

// Function to smoothly adjust the voltage
float StepperMotor::smooth_voltage_adjustment( float current_voltage, 
                                                          float desired_voltage, 
                                                          float smoothing_rate, 
                                                          float delta_t)
{
    return current_voltage + (desired_voltage - current_voltage) * smoothing_rate * delta_t;
}



// Function to calculate the smoothing rate
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


    m_sensor.get_diagnostic(); // dummy read
  
    m_motor_status = FOC_MOTOR_STATUS::UNCALIBRATED;

    // align motor if necessary
    // alignment necessary for encoders!
    // sensor and motor alignment - can be skipped
    // by setting motor.sensor_direction and motor.zero_electric_angle
    HAL_Delay(500);

    success &= alignSensor(); // bitwise intentional

    m_sensor.update();
    m_shaft_angle = m_sensor.get_angle_radians();

    if(success)
    {
    	m_motor_status = FOC_MOTOR_STATUS::READY;
    }
    else
    {
    	m_motor_status = FOC_MOTOR_STATUS::CALIBRATION_FAILED;
        disable();
    }

    return success;
}



bool StepperMotor::determine_sensor_direction()
{
       bool  success(true);
       float mechanical_angle(0.0f);
       
       float initial_angle = m_sensor.read_angle_radians();
    
       // Command a 90 degree rotation over 100 steps
       for (uint32_t i = 0; i <= NUM_STEPS; i++ )
       {
           mechanical_angle = initial_angle  +  (i * STEP_SIZE);
         
           setPhaseVoltage(m_voltage_sensor_align, 
                           0.0f,  
                           mechanical_to_electrical_radians(mechanical_angle));
#if 0 // debug
           char char_buffer[50];
           sprintf(char_buffer, "o: %ld  rad: %d\r\n", 
                   i, 
                   static_cast<int32_t>(1000.0f *electric_angle ));
           HAL_UART_Transmit(&huart2, 
                             reinterpret_cast<uint8_t *>(char_buffer), 
                             strlen(char_buffer), 
                             HAL_MAX_DELAY);
#endif
       
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
bool StepperMotor::alignSensor()
{
   return determine_sensor_direction();


  const uint32_t TWO_MILLISECONDS(2);
  bool success(true);

  if(Direction::UNKNOWN == m_sensor_direction)
  {
    if(needsSearch()) // TODO: need a clearer func name
    {
        success = absoluteZeroSearch();
    }
    
    // exit if index not found
    if(!success)
    {
        return success;
    }


    
    
    // find natural direction
    // move one electrical revolution forward
    // ramp up from 1.5Pi to 3.5Pi
    float electric_angle;

    // Initial Rotation (270 degrees CW)
    for (int i = 0; i <=500; i++ )
    {
        electric_angle = THREE_HALVES_PI + TWO_PI * i / 500.0f;
      
        setPhaseVoltage(m_voltage_sensor_align, 0.0f,  electric_angle);

	    HAL_Delay(TWO_MILLISECONDS);
    }

    //  mid_angle represents the sensor's reading after 
    //  270 degree clockwise motor rotation.
    float mid_angle = m_sensor.read_angle_radians();
     
    // Reverse Rotation (270 degrees CCW)
    for (int i = 500; i >=0; i-- ) 
    {
        electric_angle = THREE_HALVES_PI + TWO_PI * i / 500.0f ;
        
        setPhaseVoltage(m_voltage_sensor_align, 0.0f,  electric_angle);
        
	    HAL_Delay(TWO_MILLISECONDS);
    }
        
    float end_angle = m_sensor.read_angle_radians();
    
    // zero applied voltages
    setPhaseVoltage(0, 0, 0);                  

    HAL_Delay(200);

    // determine the direction the sensor moved
    if (mid_angle == end_angle)
    {
      return 0; // failed calibration
    }
    else
    if (mid_angle < end_angle)
    {
        m_sensor_direction = Direction::CCW;
    }
    else
    {
        m_sensor_direction = Direction::CW;
    }

    m_sensor.invert_output(mid_angle < end_angle);

    
    float electrical_radians = 
          mechanical_to_electrical_radians(fabs(mid_angle - end_angle));
    
    if( fabs(electrical_radians - TWO_PI) > 0.5f )
    { 
        // 0.5f is arbitrary number it can be lower or higher!
    }
  }


  // zero electric angle not known
  if(NOT_SET == m_radian_offset_to_electric_zero)
  {
      // align the electrical phases of the motor and sensor
      // set angle -90(270 = 3PI/2) degrees
      setPhaseVoltage(m_voltage_sensor_align, 0,  THREE_HALVES_PI);

      HAL_Delay(700);

      m_sensor.update();
      
      // get the m_amperage zero electric angle
      m_radian_offset_to_electric_zero = get_electric_angle_radians();

      m_sensor_offset = m_radian_offset_to_electric_zero; // TODO: Added but Not sure about this



      char  buff[128];
      sprintf(buff, "Zero Electric Angle: %ld\r\n", static_cast<uint32_t>(1000*m_radian_offset_to_electric_zero));
      HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(buff), strlen(buff), HAL_MAX_DELAY);
      
      HAL_Delay(20);

      // stop everything
      setPhaseVoltage(0.0f, 00.0f, 0.0f);

      HAL_Delay(200);
  }
  
  return success;
}

//-----------------------------------------------------------------------------
//                          absoluteZeroSearch
//
// Encoder alignment the absolute zero angle - to the index
//
// return true if search is complete
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
      m_sensor.update();
    }
    
    // disable motor
    setPhaseVoltage(0.0f, 0.0f, 0.0f); // TODO: refactor name
    
    // Pop limits
    m_velocity_limit = limit_vel;
    m_voltage_limit  = limit_volt;

    return !needsSearch();
}


//-----------------------------------------------------------------------------
//                               loopFOC
//
// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
//-----------------------------------------------------------------------------
void StepperMotor::loopFOC(float winding_amperage_a, float winding_amperage_b)
{
    //const uint32_t TWO_MILLISECOND(2);

    const float SECONDS_PER_MICROSECOND( 0.000001f); // TODO move to class level
    
    unsigned long now_us = _micros();

    // divide offset by range of 5 volts, then take half
    float current_offset_a = m_current_offset_a.update(winding_amperage_a)/10.0f;
    float current_offset_b = m_current_offset_a.update(winding_amperage_b)/10.0f;
#if 0

    if(current_offset_a > 0.0)
    {
        m_hifactor_a = (1.0f-current_offset_a);
        m_lofactor_a = (1.0f+current_offset_a);
    }
    else
    {
        m_hifactor_a = (1.0f+current_offset_a);
        m_lofactor_a = (1.0f-current_offset_a);
    }

    if(current_offset_b > 0.0)
    {
        m_hifactor_b = (1.0f-current_offset_b);
        m_lofactor_b = (1.0f+current_offset_b);
    }
    else
    {
        m_hifactor_b = (1.0f+current_offset_b);
        m_lofactor_b = (1.0f-current_offset_b);
    }
#endif

    float delta_t           = static_cast<float>(now_us - m_target_prev_timestamp)
                            * SECONDS_PER_MICROSECOND;
    
    m_target_prev_timestamp = now_us;
    
    m_sensor.update();

    m_shaft_angle = get_filtered_shaft_angle(); // <-----why here?

    g_shaft_angle = m_shaft_angle;



    // Filter the current readings
    //m_winding_amperage_a = m_LPF_current_winding_a(winding_amperage_a);
    //m_winding_amperage_b = m_LPF_current_winding_a(winding_amperage_b);

    //With ACS712, the input is fairly smooth.
    m_winding_amperage_a = winding_amperage_a;
    m_winding_amperage_b = winding_amperage_b;
    
    transformCurrents( winding_amperage_a,
                       winding_amperage_b,
                       m_shaft_angle,
                       m_amperage.d,
                       m_amperage.q);
    

    switch (m_motion_control) 
      {
        case MOTION_CONTROL_TYPE::TORQUE:
                         
             break;             
        
        case MOTION_CONTROL_TYPE::CL_ANGLE:
             
             update_position_closed_loop(m_target, delta_t);
    
             break;
             
             
        case MOTION_CONTROL_TYPE::CL_VELOCITY:
    
             update_speed_closed_loop(m_target,  delta_t);
                                  
             break;
             
        case MOTION_CONTROL_TYPE::OL_VELOCITY:

             update_speed_open_loop(m_target,  delta_t);

             break;
        
        case MOTION_CONTROL_TYPE::OL_ANGLE:
    
             update_position_open_loop(m_target,  delta_t);
             
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
void StepperMotor::move(float new_target) 
{
    const float MICROSECONDS_PER_SECONDS_PER_MICROSECOND( 0.000001f); // TODO move to class level
    
    unsigned long now_us = _micros();
    
    float delta_t           = static_cast<float>(now_us - m_target_prev_timestamp)
                            * MICROSECONDS_PER_SECONDS_PER_MICROSECOND;
    float delta_target      = new_target - m_target_prev;
    m_target_prev           = new_target;
    m_target_prev_timestamp = now_us;
    

    // downsampling (optional)
   // if(m_motion_cnt++ < m_motion_downsample)
    //{
    //    return;
   // }
    
  //  m_motion_cnt = 0;

    // shaft angle/velocity need the update() to be called first
    // get shaft angle
    // TODO sensor precision: the shaft_angle actually stores the complete position, 
    //                        including full rotations, as a float. For this 
    //                        reason it is NOT precise when the angles become large.
    //                        Additionally, the way LPF works on angle is a precision 
    //                        issue, and the angle-LPF is a problem
    //                        when switching to a 2-component representation.

    // read value even if motor is not in openloop mode
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

           update_torque_open_loop(new_target, delta_t, delta_target);
                       
           break;
           
      
      case MOTION_CONTROL_TYPE::CL_ANGLE:

           m_target = new_target;       

           break;
           
           
      case MOTION_CONTROL_TYPE::CL_VELOCITY:

           m_target = new_target;    

           m_amperage_prev.d = m_amperage.d = 0.0f;
           m_amperage_prev.q = m_amperage.q = 0.0f;
           m_voltage_prev.d  = m_voltage.d  = 0.0f;
           m_voltage_prev.q  = m_voltage.q  = 0.0f;


      
                                
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

           m_target = new_target;  
           
           break;

      default:
           ; // NOP
  }
}

//-----------------------------------------------------------------------------
//                                 
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


    char char_buffer[50];
    sprintf(char_buffer, "rad: %d \r\n", static_cast<int32_t>(1000.0f *next_position ));
    HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(char_buffer), strlen(char_buffer), HAL_MAX_DELAY);




    setPhaseVoltage(m_voltage.q, m_voltage.d, normalize_radians(next_position));

    m_amperage_prev.d = m_amperage.d;
    m_amperage_prev.q = m_amperage.q;
    m_voltage_prev.d  = m_voltage.d;
    m_voltage_prev.q  = m_voltage.q;

}


#endif

//-----------------------------------------------------------------------------
//                          update_speed_closed_loop
//
// This implements the torque control loop. As the stepper motors only support 
// torque using voltage mode. 
//
// Read the current motor angle from the sensor, turn it into the electrical 
// angle and transforms the q-axis Uq voltage command motor.voltage_q
//-----------------------------------------------------------------------------
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

     static float mechanical_radians(0.0f); // TODO: s.b. member
     mechanical_radians += target_mechanical_rps*delta_t;
     
     mechanical_radians = normalize_radians(mechanical_radians);
     
//override
  //  mechanical_radians = g_shaft_angle;


    g_mechanical_angle = mechanical_radians;


    float mag_flux_linkage_q = inductance(target_electrical_rps) * m_amperage.q;  
    float back_emf_q_axis    = m_omega_mechanical_rps 
                             * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

    g_mag_flux_linkage_q = mag_flux_linkage_q;
    g_back_emf_q_axis    = back_emf_q_axis;

    float desired_voltage_q = m_current_limit * resistance(target_electrical_rps) 
                            + fabs(back_emf_q_axis);

    m_voltage.q     = symetric_clamp(desired_voltage_q, m_voltage_limit);

    // Dynamic smoothing based on speed
    float voltage_smoothing_rate = calculate_smoothing_rate(target_electrical_rps);
    
    // Smoothly adjust the voltage
    m_voltage.q = smooth_voltage_adjustment(m_voltage.q, 
                                            desired_voltage_q, 
                                            voltage_smoothing_rate, 
                                            delta_t);
    
    m_voltage.d     = 0.0f;
    setPhaseVoltage(m_voltage.q, 
                    m_voltage.d,  
                    mechanical_to_electrical_radians(mechanical_radians));
}

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
    float angular_error = target_mech_angle_radians - m_sensor.get_angle_radians();

    float v_pid = m_PID_angle.update(angular_error);

    float amperage_d_dot(0.0f);
    float amperage_q_dot(0.0f);

    if(delta_t > 0.0f)
    {
        amperage_d_dot = (m_amperage.d - m_amperage_prev.d)/delta_t;
        amperage_q_dot = (m_amperage.q - m_amperage_prev.q)/delta_t;
    }

    // Flux linkages have units of volt seconds
    float mag_flux_linkage_d            = inductance(0.0f) * m_amperage.d;
    float mag_flux_linkage_q            = inductance(0.0f) * m_amperage.q;


    float resistive_voltage_drop_d_axis = resistance(0.0f) * m_amperage.d;
    float resistive_voltage_drop_q_axis = resistance(0.0f) * m_amperage.q;


    float inductive_voltage_drop_d_axis = inductance(0.0f) * amperage_d_dot;
    float inductive_voltage_drop_q_axis = inductance(0.0f) * amperage_q_dot;

    float electrical_rps                = mechanical_to_electrical_radians(m_omega_mechanical_rps);
    float back_emf_d_axis               = electrical_rps * mag_flux_linkage_d;
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

     float target_angle_electric =  normalize_radians( mechanical_to_electrical_radians(target_mech_angle_radians));
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

    float target_angle_error  = target_mechanical_radians -  m_sensor.get_angle_radians();
    float max_angular_change  = m_velocity_limit * delta_t;

    float shaft_angle_command = m_sensor.get_angle_radians();              // duplicate call 
        
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
       m_voltage.q  = 0.0f;
       m_voltage.d  = 0.0f;
       setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);

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





void StepperMotor::clarkeTransform( float Ia, 
                            float  Ib, 
                            float& Ialpha, 
                            float& Ibeta) 
{
    Ialpha = Ia;
    Ibeta = (Ia + 2.0f * Ib) * SQUARE_ROOT_OF_3_INVERSE;
}

// Function to perform Park Transformation (Rotation)
void StepperMotor::parkTransform( float  Ialpha, 
                          float  Ibeta, 
                          float  theta, 
                          float& Id, 
                          float& Iq) 
{
    Id =  Ialpha * cosf(theta) + Ibeta * sinf(theta);
    Iq = -Ialpha * sinf(theta) + Ibeta * cosf(theta);
}

void StepperMotor::transformCurrents( 
                               float  Ia, 
                               float  Ib, 
                               float  encoderAngle, 
                               float& Id, 
                               float& Iq) 
{
    float Ialpha, Ibeta;

    // Perform Clarke Transformation
    clarkeTransform(Ia, Ib, Ialpha, Ibeta);

    // Convert encoder angle to radians
    float theta = encoderAngle * PI / 180.0f;

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
    // Sinusoidal PWM modulation
    // Inverse Park transformation
    float _sa;
    float _ca;
    _sincos(electric_angle, &_sa, &_ca);
    //sincos(electric_angle, &_sa, &_ca);
    
    // Inverse park transform
    m_U_alpha =  _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
    m_U_beta  =  _sa * Ud + _ca * Uq;  //  cos(angle) * Uq;
    
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
  if(m_enabled)
  {
      compute_inverse_park_transform( Uq, Ud, electric_angle);

      // set the voltages in hardware
      //m_driver.set_pwm_duty_cycle(m_U_alpha, m_U_beta);
      
      m_driver.set_pwm_duty_cycle(m_U_alpha, m_U_beta, m_hifactor_a, m_lofactor_a, m_hifactor_b, m_lofactor_b);

      
  }
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
    unsigned long now_us = _micros();
  
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
  
  
    float target_angle_error(   target_angle - m_shaft_angle );
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
    float radians =    m_sensor.get_angle_radians(); 

    if(Direction::CCW == m_sensor_direction)
    {
       radians *= -1.0f;
    }

    // m_sensor_offset is currently 0
    return  radians - m_sensor_offset;

}

//-----------------------------------------------------------------------------
//                         shaft_radians_per_second
//-----------------------------------------------------------------------------
float StepperMotor::shaft_radians_per_second() 
{
  float rad_per_sec = m_LPF_velocity(m_sensor.get_radians_per_second());
  
  return (Direction::CCW ==m_sensor_direction) ? -rad_per_sec : rad_per_sec;
  //return rad_per_sec;
}

//-----------------------------------------------------------------------------
//                        get_phase_angle_radians (electrical)
//-----------------------------------------------------------------------------
float StepperMotor::get_electric_angle_radians()
{
  float direction       = static_cast<float>(m_sensor_direction);
  
  float electic_radians = mechanical_to_electrical_radians(
                             m_sensor.get_mechanical_phase_angle_radians()) ;
  
  float raw_angle  = direction * electic_radians - m_radian_offset_to_electric_zero;
  //float raw_angle  = electic_radians - m_radian_offset_to_electric_zero;

  return  normalize_radians( raw_angle );
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
    float alignmentSpeed        = 2 * PI; // radians per second
 //   float alignmentAcceleration = PI;     // radians per second squared


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
        char  buff[128];
        sprintf(buff, "Current Position: %ld radians\r\n", static_cast<uint32_t>(1000.0f *currentPosition));
        HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(buff), strlen(buff), HAL_MAX_DELAY);

        HAL_Delay(100);

           
        setPhaseVoltage(voltage, 0.0f, mechanical_to_electrical_radians(currentPosition));

        
    }
    
    // Stop motor (set voltage to zero)
    voltage = 0.0f;
    
    // Print message when alignment is complete
    char  buff[128];
    sprintf(buff, "Rotor Field Alignment Complete\r\n");
    HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(buff), strlen(buff), HAL_MAX_DELAY);


    
}

#if 0
int main() {
    // Specify the target position for alignment (in radians)
    double targetPosition = PI / 4; // Example: Align to 45 degrees
    
    // Call the rotorFieldAlignment function to align the rotor field
    rotorFieldAlignment(targetPosition);
    
    return 0;
}
#endif
//==============================

