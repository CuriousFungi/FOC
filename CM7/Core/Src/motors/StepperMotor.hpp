/**
 *  @file StepperMotor.h
 * 
 */

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <vector>
#include <cstddef>
#include <cstdint>

//#include <deque>
#include <numeric>


#include "limit.hpp"

#include "AS5048A.hpp"

#include "StepperDriver.hpp"
#include "foc_utils.hpp"
#include "time_utils.hpp"
#include "defaults.h"

#include <cstdint>

#include "CurrentSense.hpp"

#include "pid.hpp"
#include "lowpass_filter.hpp"


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_uart.h"

#ifdef __cplusplus
}
#endif


#define MICROSECONDS_PER_ITERATION (100) //(500)


// move to sensor
//#define AS5048_MAX 16384            // 14-bit encoder max value (2^14)
#define PI_SCALING (13176795)       // Approximation of 2*pi * (2^24) to maintain precision
#define US_TO_SEC_SCALING (1000000) // Convert microseconds to seconds for velocity calculation
#define BUFFER_SIZE 10              // Circular buffer size for moving window

#if 0
class OffsetEstimator 
{
public:
    OffsetEstimator(size_t window_size)
        : window_size_(window_size), sum_(0.0f), cnt(0)
    {
    }

    float update(float current_measurement) 
    {
        // Update the window with the new measurement
        if (window_.size() >= window_size_) 
        {
            sum_ -= window_.front();
            window_.pop_front();
        }
        window_.push_back(current_measurement - centerline_);
        sum_ += window_.back();
        cnt++;

        // Calculate the average offset
        return sum_ / static_cast<float>(window_.size());
    }

private:

    const size_t window_size_;
    const float centerline_ = 2.5f;
    float sum_;
    std::deque<float> window_;
    uint32_t cnt;
};
#endif

// TODO: create cirulare queue class
// TODO: refactor to use circular queue
// TODO: move circular queue, offsetestimator, pid classes into seperate .hpp/.cpp files
class OffsetEstimator
{
public:
    OffsetEstimator(size_t window_size)
        : window_size_(window_size),
          window_(window_size, 0.0f),  // Pre-allocate to fixed size with initial values of 0.0
          sum_(0.0f),
          index_(0),
          count_(0),
          centerline_(2.5f)
    {
    }

    float update(float current_measurement)
    {
        // Calculate the measurement adjusted by the centerline
        float adjusted_measurement = current_measurement - centerline_;

        // If the buffer is full, subtract the oldest value from sum_
        if (count_ >= window_size_)
        {
            sum_ -= window_[index_];
        }
        else
        {
            count_++;  // Increase count until buffer is full
        }

        // Insert the new value into the ring buffer and add to sum_
        window_[index_] = adjusted_measurement;
        sum_ += adjusted_measurement;

        // Move to the next index, wrapping if necessary
        index_ = (index_ + 1) % window_size_;

        // Calculate the average offset
        return sum_ / static_cast<float>(count_);
    }

private:
    const size_t window_size_;
    std::vector<float> window_;
    float sum_;
    size_t index_;    // Current insertion index for the ring buffer
    size_t count_;    // Current count of valid entries in buffer (up to window_size_)
    const float centerline_;
};



// monitoring bitmap
#define _MON_TARGET 0b1000000 // monitor target value
#define _MON_VOLT_Q 0b0100000 // monitor voltage q value
#define _MON_VOLT_D 0b0010000 // monitor voltage d value
#define _MON_CURR_Q 0b0001000 // monitor current q value - if measured
#define _MON_CURR_D 0b0000100 // monitor current d value - if measured
#define _MON_VEL    0b0000010 // monitor velocity value
#define _MON_ANGLE  0b0000001 // monitor angle value

/**
 *  Motiron control type
 */
enum class MOTION_CONTROL_TYPE : uint8_t
{
  TORQUE      = 0x00,    
  OL_ANGLE    = 0x01,
  CL_ANGLE    = 0x02,    
  OL_VELOCITY = 0x03,
  CL_VELOCITY = 0x04   
};

/**
 *  Motiron control type
 */
enum class TORQUE_CONTROL_TYPE : uint8_t
{
  VOLTAGE            = 0x00,     //!< Torque control using voltage
  DC_CURRENT         = 0x01,     //!< Torque control using DC current (one current magnitude)
  FOC_CURRENT        = 0x02,     //!< torque control using dq currents
};

/**
 *  FOC modulation type
 */
enum class FOC_MODULATION_TYPE : uint8_t
{
  SINE_PWM           = 0x00,     //!< Sinusoidal PWM modulation
  SPACE_VECTOR_PWM   = 0x01,     //!< Space vector modulation method
  TRAPEZOID_120      = 0x02,
  TRAPEZOID_150      = 0x03,
};



enum class FOC_MOTOR_STATUS : uint8_t
{
  UNINITIALIZED         = 0x00,     //!< Motor is not yet initialized
  INITIALIZING          = 0x01,     //!< Motor intiialization is in progress
  UNCALIBRATED          = 0x02,     //!< Motor is initialized, but not calibrated (open loop possible)
  CALIBRATING           = 0x03,     //!< Motor calibration in progress
  READY                 = 0x04,     //!< Motor is initialized and calibrated (closed loop possible)
  ERROR                 = 0x08,     //!< Motor is in error state (recoverable, e.g. overcurrent protection active)
  CALIBRATION_FAILED    = 0x0E,     //!< Motor calibration failed (possibly recoverable)
  INITIALIZATION_FAILED = 0x0F,     //!< Motor initialization failed (not recoverable)
};



class PIDController2 {
public:
    float Kp, Ki, Kd;  // Proportional, Integral, and Derivative gains
    float previous_error;
    float integral;
    float max_output;
    
    PIDController2(float kp, float ki, float kd, float max_out)
        : Kp(kp), Ki(ki), Kd(kd), previous_error(0), integral(0), max_output(max_out) {}

    // PID calculation function
    float calculate(float setpoint, float measured_value, float delta_time) {
        // Calculate error
        float error = setpoint - measured_value;

        // Proportional term
        float Pout = Kp * error;

        // Integral term
        integral += error * delta_time;
        float Iout = Ki * integral;

        // Derivative term
        float derivative = (error - previous_error) / delta_time;
        float Dout = Kd * derivative;

        // Total output
        float output = Pout + Iout + Dout;

        // Clamp output to the maximum allowed value
        if (output > max_output) {
            output = max_output;
        } else if (output < -max_output) {
            output = -max_output;
        }

        // Store error for the next derivative calculation
        previous_error = error;

        return output;
    }
};



//=============================================================================
//                          StepperMotor Class
//=============================================================================
class StepperMotor
{
  public:

	//-------------------------------------------------------------------------
	//                                CTor
	//
	//   @param hspi                             SPI_HandleTypeDef*
	//   @param p_sensor_chip_select_port        GPIO_TypeDef*
	//   @param sensor_chip_select_pin
    //   @param pp  pole pair number
    //   @param R  motor phase resistance      - [Ohm]
    //   @param KV  motor KV rating (1/K_bemf) - [rpm/V]
    //   @param L  motor phase inductance      - [H]
	//   @param voltage_limit                  - [volts]
    //   @param p_htim,
    //   @param timer_channel_a,
    //   @param timer_channel_b
	//-------------------------------------------------------------------------
    explicit
    StepperMotor( SPI_HandleTypeDef* hspi,
                  int                pole_pairs,
                  float              phase_resistance,
                  float              KV,
                  float              phase_inductance,
                  float              voltage_limit,
                  float              power_supply_voltage,
                  TIM_HandleTypeDef* p_htim_1,
                  TIM_HandleTypeDef* p_htim_2,
                  uint32_t           timer_channel_phase_1A,
                  uint32_t           timer_channel_phase_1B,
                  uint32_t           timer_channel_phase_2A,
                  uint32_t           timer_channel_phase_2B);

	//-------------------------------------------------------------------------
    //                                  init
	//-------------------------------------------------------------------------
  	void init();

	//-------------------------------------------------------------------------
    //                                  disable
	//-------------------------------------------------------------------------
  	void disable();


	//-------------------------------------------------------------------------
    //                                  enable
	//-------------------------------------------------------------------------
    void enable();

	//-------------------------------------------------------------------------
    //                                  initFOC
    //
    // Initialize FOC parameters and align zero position of sensor and motor.
    // Skip alignment procedure if zero_electric_offset parameter is set.
    //
    // returns true on success
	//-------------------------------------------------------------------------
    bool initFOC();

	//-------------------------------------------------------------------------
    //                                  loopFOC
    //
    // FOC task loop which determines motor angle and commands PWM voltages.
    // The fasster this can be run the better.
	//-------------------------------------------------------------------------
    void loopFOC(float winding_amperage_a, float winding_amperage_b);

	//-------------------------------------------------------------------------
    //                               move
    //
    //  @param target  Either voltage, angle or velocity based on the
    //                 motor.controller. If it is not set the motor will use the
    //                 target set in its variable motor.target
    //
    //  @note Doesn't need to be run upon each loop execution - depends of the use case
	//-------------------------------------------------------------------------
    void move(float target = NOT_SET);             // was override;



    uint32_t compute_time_difference(uint32_t current_time, uint32_t previous_time);
    uint16_t get_as5048a_value(){return m_sensor.get_raw_count();}
    //void reinit_dma_for_spi(){m_sensor.reinit_dma_for_spi();}
    void update_buffers(uint16_t new_angle, uint32_t new_timestamp){m_sensor.update_buffers(new_angle,new_timestamp);}
    void start_spi_conversion(){m_sensor.start_spi_conversion();}
    void spi_reset_in_progress(){m_sensor.spi_reset_in_progress();}

    void update_target_rad_per_sec(float rps);
    void control_loop_25us();
    void sample_as5048_25us();
    float convert_count_to_shaft_angle(uint16_t count);
    float calculate_delta_angle(uint16_t last_angle, uint16_t current_angle);
   // void conversion_complete(){m_sensor.conversion_complete();}
   // void process_encoder_data(){m_sensor.process_encoder_data();}
   // void timestamp_angle_reading(){m_sensor.timestamp();}
    bool async_read_complete(){return m_sensor.async_read_complete();}

    float read_angle_radians_from_buffer_with_offset();
    void update_samples(uint16_t raw_count);
    bool is_sample_valid(uint16_t raw_value){return m_sensor.is_sample_valid(raw_value);}

    bool error_detected()
    {
       return m_sensor.error_detected();
    }

    uint16_t get_errors()
    {
       return m_sensor.get_errors();
    }

    void     clear_error()
    {
        return  m_sensor.clear_error();
    }

    float mechanical_to_electrical_radians(float mechanical_radians)
    {
        return mechanical_radians * NUM_POLE_PAIRS;
    } 
    
    void update_torque_open_loop(float target_quadrature_voltage, float delta_t, float delta_target);


    void update_speed_open_loop(float target_mechanical_rps, float delta_t);
    void update_speed_closed_loop(float target_mechanical_rps,           float delta_t);

    void update_position_open_loop(float target_mech_angle_radians, float delta_t);
    void update_position_closed_loop(float target_mech_angle_radians, float delta_t);


    //-------------------------------------------------------------------------
    //                          setPhaseVoltage
    //
    //! @param Uq Current voltage in q axis to set to the motor
    //! @param Ud Current voltage in d axis to set to the motor
    //! @param angle_el m_amperage electrical angle of the motor
    //
    //! @note Uses FOC to set Uq at optimal angle.
    //
    // This is the heart of the FOC algorithm
    //-------------------------------------------------------------------------
    void setPhaseVoltage(float Uq, float Ud, float angle_el);


    void rotorFieldAlignment(double targetPosition);

    bool is_clockwise();

    float calculate_velocity(float current_angle, float delta_seconds);
    
    struct Sample get_current_sample(){return m_current_sample;}
    void set_async_read_complete(){m_sensor.set_async_read_complete();}
  private:

    void compute_inverse_park_transform( 
                                    float Uq, 
                                    float Ud, 
                                    float electric_angle);


    //float shaft_radians_per_second();         // from FOCMotor::
    float get_electric_angle_radians();       // from FOCMotor::


  
    float get_filtered_shaft_angle();
  
    //-------------------------------------------------------------------------
    //                          alignSensor
    //
    // Sensor alignment to electrical 0 angle of the motor
    // returns true on success
    //-------------------------------------------------------------------------
    bool alignSensor();

    //bool determine_sensor_direction();             // replacement for alignSensor

    //-------------------------------------------------------------------------
    //                           absoluteZeroSearch
    //
    // Serach for the absolute 0 of sensor angle.
    // return true if found
    //-------------------------------------------------------------------------
    //bool absoluteZeroSearch();
        
    // Open loop motion control    
    /**
     * Function (iterative) generating open loop movement for target velocity
     * it uses voltage_limit variable
     * 
     * @param target_velocity - rad/s
     */
   // float velocityOpenloop(float target_velocity);
    /**
     * Function (iterative) generating open loop movement towards the target angle
     * it uses voltage_limit and velocity_limit variables
     * 
     * @param target_angle - rad
     */
    float angleOpenloop(float target_angle);

    
    // private function used to determine if encoder has index


    // getter for index pin
    // return -1 if no index
    int needsSearch()
    {
      return false;
    }


    //-------------------------------------------------------------------------
    //                           normalize_radians
    //
    // Return angle in range [0, TWO_PI)
    //-------------------------------------------------------------------------
    float normalize_radians(float radians)
    {
        float a = fmod(radians, TWO_PI);
        
        return a < 0 ? a + TWO_PI : a;
    }



    void clarkeTransform( float Ia, 
                                float  Ib, 
                                float& Ialpha, 
                                float& Ibeta); 
    
    void parkTransform(         float  Ialpha, 
                                float  Ibeta, 
                                float  theta, 
                                float& Id, 
                                float& Iq);
    
    void transformCurrents( float Ia, 
                                   float  Ib, 
                                   float  encoderAngle, 
                                   float& Id, 
                                   float& Iq);

    float resistance(float electical_radians_per_second);
    float inductance(float electical_radians_per_second);
    
    float smooth_voltage_adjustment( float current_voltage, 
                                                float desired_voltage, 
                                                float smoothing_rate, 
                                                float delta_t);
    
    float calculate_smoothing_rate(float target_electrical_rps);

    const float         MY_PI; 
    const float         HALF_PI;
    const float         TWO_PI;
    const float         THREE_PI;
    const float         THREE_HALVES_PI;
    const float         RAD_PER_SEC_TO_REV_PER_MIN;
    
    const float         NUM_POLE_PAIRS;              // altough an integer, its always used as a float
    
    const float         KV_RPM_PER_VOLT;
    const float         PHASE_RESISTANCE; 
    const float         PHASE_RESISTANCE_INVERSE;
    const float         PHASE_INDUCTANCE;
    const float         PHASE_INDUCTANCE_INVERSE;


    const float         PHASE_RESISTANCE_1KHZ;
    const float         PHASE_RESISTANCE_10KHZ;
    const float         PHASE_INDUCTANCE_1KHZ;
    const float         PHASE_INDUCTANCE_10KHZ;


    // for determine_sensor_direction
    const float         MIN_ALIGN_ANGLE_DETECT_MOVEMENT;
    const float         MAX_SENSOR_ANGLE;
    const uint32_t      NUM_STEPS;
    const float         ROTATION_ANGLE;
    const float         STEP_SIZE;
    const float         SQUARE_ROOT_OF_3_INVERSE;
    const float         PERM_MAGNET_FLUX_LINKAGE;

    AS5048A             m_sensor;
    StepperDriver       m_driver; 

    // Phase voltages for inverse Park and Clarke transform
    float	            m_U_alpha;
	float               m_U_beta;

    unsigned long                m_prev_open_loop_timestamp_us;

    // state variables
    float               m_target;                  //!< current target value - depends of the controller
    float               m_target_prev;
    unsigned long       m_target_prev_timestamp;
    float               m_feed_forward_velocity;   // TODO: this isn't really used
    float               m_shaft_angle;             //!< current motor angle
    float               m_shaft_rad_per_sec;
    float               m_omega_mechanical_rps;          //!< current motor velocity 

    float               m_current_sp;              //!< target current ( q current )
    float               m_shaft_velocity_target;       //!< current target velocity
    float               m_shaft_angle_target;          //!< current target angle
    DQVoltage_s         m_voltage;                 //!< current d and q voltage set to the motor
    DQVoltage_s         m_voltage_prev;
    DQCurrent_s         m_amperage;                 //!< m_amperage d and q current measured
    DQCurrent_s         m_amperage_prev; 



    // m_voltage_bemf should always be positive
    float               m_voltage_bemf;            //!< estimated backemf voltage (if provided KV constant)
    
    // motor configuration parameters
    float               m_voltage_sensor_align;    //!< sensor and motor align voltage parameter
    float               m_velocity_index_search;   //!< target velocity for index search
      
    
    // limiting variables
    float               m_voltage_limit;           //!< Voltage limiting variable - global limit
    float               m_current_limit;           //!< Current limiting variable - global limit
    float               m_velocity_limit;          //!< Velocity limiting variable - global limit
    
    // motor status vairables
    bool                m_enabled;                 //!< enabled or disabled motor flag
    FOC_MOTOR_STATUS    m_motor_status;            //!< motor status
      
    // pwm modulation related variables
    FOC_MODULATION_TYPE m_foc_modulation;          //!<  parameter determining modulation algorithm
    int8_t              m_modulation_centered;     //!< flag (1) centered modulation around driver limit /2  or  (0) pulled to 0
     
    
    // configuration structures
    TORQUE_CONTROL_TYPE m_torque_control;          //!< parameter determining the torque control type
    MOTION_CONTROL_TYPE m_motion_control;          //!< parameter determining the control loop to be used
   
    // controllers and low pass filters
    
    PIDController       m_PID_amperage_q;           //!< parameter determining the q current PID config
    PIDController       m_PID_amperage_d;           //!< parameter determining the d current PID config

    PIDController       m_PID_velocity;            //!< parameter determining the velocity PID configuration
    PIDController       m_PID_angle;               //!< parameter determining the position PID configuration

    LowPassFilter       m_LPF_current_winding_a;
    LowPassFilter       m_LPF_current_winding_b;


    LowPassFilter       m_LPF_current_q;           //!<  parameter determining the current Low pass filter configuration
    LowPassFilter       m_LPF_current_d;           //!<  parameter determining the current Low pass filter configuration

    LowPassFilter       m_LPF_velocity;            //!<  parameter determining the velocity Low pass filter configuration
    LowPassFilter       m_LPF_angle;               //!<  parameter determining the angle low pass filter configuration
    LowPassFilter       m_LPF_back_emf;

   // unsigned int        m_motion_downsample;       //!< parameter defining the ratio of downsampling for move commad
   // unsigned int        m_motion_cnt;              //!< counting variable for downsampling for move commad
    
    // sensor related variabels
    float               m_sensor_offset;           //!< user defined sensor zero offset
    float               m_radian_offset_to_electric_zero;     //!< absolute zero electric angle - if available
    Direction           m_sensor_direction;        //!< default is CW. if sensor_direction == Direction::CCW then direction will be flipped compared to CW. Set to UNKNOWN to set by calibration
    
    
    
     // Utility function intended to be used with serial plotter to monitor motor variables
     // significantly slowing the execution down!!!!
    
    // TODO: move to another class start
    unsigned int        m_monitor_downsample;      //!< show monitor outputs each monitor_downsample calls
    char                m_monitor_start_char;      //!< monitor starting character
    char                m_monitor_end_char;        //!< monitor outputs ending character
    char                m_monitor_separator;       //!< monitor outputs separation character
    unsigned int        m_monitor_decimals;        //!< monitor outputs decimal places
      
    // initial monitoring will display target, voltage, velocity and angle
    uint8_t             monitor_variables;         //!< Bit array holding the map of variables the user wants to monitor
    // TODO: move to another class end
    
    CurrentSense*       m_p_current_sense; 
      
    // monitor counting variable
    unsigned int        m_monitor_cnt;             //!< counting variable

    OffsetEstimator     m_current_offset_a;
    OffsetEstimator     m_current_offset_b;
    float               m_hifactor_a;
    float               m_lofactor_a;
    float               m_hifactor_b;
    float               m_lofactor_b;

    float               m_commanded_speed_radians_per_sec;
    float               m_commanded_angle_radians;
    float               m_measured_speed_radians_per_sec;
    float               m_measured_angle_radians;

    
    float               m_target_voltage_q;
    float               m_accumulated_mechanical_radians; // For open-loop angle generation


   
    struct Sample      m_current_sample;
};


#endif // inclusion guard
