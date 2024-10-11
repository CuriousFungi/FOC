
#include "AS5048A.hpp"
#include "../../Inc/EnumValue.hpp"
#include "../common/time_utils.hpp"

#include <math.h>


#include <cmath>  // For std::sin and M_PI


//extern "C" {
//    #include "arm_math.h"
//} // Include CMSIS-DSP


volatile uint16_t g_as5048_u16_angle(0);
volatile float g_as5048_velocity(0.0f);

extern volatile float g_experimental_velocity;


#include <algorithm>

//TODO: look at https://github.com/sosandroid/AMS_AS5048B/blob/master/ams_as5048b.cpp

#if 1 // DEBUG
extern UART_HandleTypeDef huart2;
#include <cstdio>
#include <cstring>
#endif

extern DAC_HandleTypeDef hdac1;

//-----------------------------------------------------------------------------
//                          CTor
//-----------------------------------------------------------------------------
AS5048A::AS5048A(    SPI_HandleTypeDef* hspi, 
                    GPIO_TypeDef*      p_chip_select_port,
                    uint16_t           chip_select_pin)
:   TWO_PI(6.28318530718f)
,   BIT_RESOLUTION(14)
,   READ_WRITE_BIT(1<<BIT_RESOLUTION)
,   PARITY_BIT(READ_WRITE_BIT << 1)
,   SPI_READ_ANGLE_CMD (   value_of(AS5048A_REGISTERS::ANGLE_14_BITS) 
                         | READ_WRITE_BIT
                         | PARITY_BIT
                       )
,   COUNTS_PER_REVOLUTION((1 << BIT_RESOLUTION))
,   COUNTS_PER_HALF_REVOLUTION(COUNTS_PER_REVOLUTION >> 1)
,   AS5048_MAX(0x4000)
,   m_hspi(hspi)
,   m_p_chip_select_port(p_chip_select_port)
,   m_chip_select_pin(chip_select_pin)
,   m_clock_speed(1000000)
,   m_position_count(0)
,   m_error_detected(false)
,   m_prev_timestamp_microseconds(0)
,   m_prev_angle_radians(0.0f)
,   m_full_rotations(0)
,   m_prev_full_rotations(0)
// from sensor
,   m_min_elapsed_time(0.000100f)
,   m_velocity(0.0f)
,   m_prev_angle_timestamp_us(0)
,   m_prev_radians_per_sec(0.0f)
,   m_prev_velocity_timestamp_us(0)

,   m_prev_microseconds(0)
,   m_invert_output(false)
,   m_async_read_complete(true)
,   m_register_value(0)
,   m_current_index(0)
{
    init_SPI_buffers();
    HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_SET);

    return;
}

//-----------------------------------------------------------------------------
//                          update
//-----------------------------------------------------------------------------
void AS5048A::update()
{
    const float MICROSECONDS_PER_SECOND = 1000000.0f;
    const float MAX_RADIANS_CHANGE = TWO_PI * 0.2f; // Example threshold to detect large spikes
    const float ALPHA = 0.1f;  // Smoothing factor for radians per second

    float curr_radians = read_angle_radians();
    uint32_t curr_microseconds = _micros();

    // Handle rollover of the microsecond counter
    uint32_t delta_microseconds;
    if (curr_microseconds < m_prev_microseconds) {
        delta_microseconds = (UINT32_MAX - m_prev_microseconds) + curr_microseconds + 1;
    } else {
        delta_microseconds = curr_microseconds - m_prev_microseconds;
    }

    float delta_radians = curr_radians - m_prev_angle_radians;

    // Handle overflow/underflow if the angle crosses the wraparound point
    if (fabs(delta_radians) > (0.8f * TWO_PI)) {
        if (delta_radians > 0.0f) {
            m_full_rotations -= 1;
            delta_radians += TWO_PI;
        } else {
            m_full_rotations += 1;
            delta_radians -= TWO_PI;
        }
    }

    // Combine full rotations into delta radians
    int32_t delta_rotations = m_full_rotations - m_prev_full_rotations;
    float delta_rotation_radians = TWO_PI * static_cast<float>(delta_rotations);
    float delta_radians_combined = delta_rotation_radians + delta_radians;

    // Outlier rejection: Ignore unrealistic large spikes in delta radians
    if (fabs(delta_radians_combined) > MAX_RADIANS_CHANGE) {
        delta_radians_combined = 0.0f; // Ignore this update if it's an outlier
    }

    // Calculate radians per second with smoothing
    float delta_microseconds_f = static_cast<float>(delta_microseconds);
    float instantaneous_radians_per_second = (MICROSECONDS_PER_SECOND * delta_radians_combined) / delta_microseconds_f;

    // Smooth the radians per second to reduce spikes
    float radians_per_second = ALPHA * instantaneous_radians_per_second + (1.0f - ALPHA) * m_prev_radians_per_sec;

    // Update state variables
    m_prev_full_rotations = m_full_rotations;
    m_prev_angle_radians = curr_radians;
    m_prev_radians_per_sec = radians_per_second;
    m_prev_microseconds = curr_microseconds;
}

#if 0
void AS5048A::update()
{
    const float MICROSECONDS_PER_SECOND = 1000000.0f;

    float curr_radians = read_angle_radians();
    uint32_t curr_microseconds = _micros();

    // Handle rollover of the microsecond counter
    uint32_t delta_microseconds;
    if (curr_microseconds < m_prev_microseconds) {
        delta_microseconds = (UINT32_MAX - m_prev_microseconds) + curr_microseconds + 1;
    } else {
        delta_microseconds = curr_microseconds - m_prev_microseconds;
    }

    float delta_radians = curr_radians - m_prev_angle_radians;

    // Handle overflow/underflow if the angle crosses the wraparound point
    if (fabs(delta_radians) > (0.8f * TWO_PI)) {
        if (delta_radians > 0.0f) {
            m_full_rotations -= 1;
            delta_radians += TWO_PI;
        } else {
            m_full_rotations += 1;
            delta_radians -= TWO_PI;
        }
    }

    int32_t delta_rotations = m_full_rotations - m_prev_full_rotations;
    float delta_rotation_radians = TWO_PI * static_cast<float>(delta_rotations);
    float delta_radians_combined = delta_rotation_radians + delta_radians;

    // Calculate radians per second
    float delta_microseconds_f = static_cast<float>(delta_microseconds);
    float radians_per_second = (MICROSECONDS_PER_SECOND * delta_radians_combined) / delta_microseconds_f;

    // Optional: Smoothing to reduce noise spikes
    float alpha = 0.1f; // Smoothing factor
    radians_per_second = alpha * radians_per_second + (1.0f - alpha) * m_prev_radians_per_sec;

    // Update state variables
    m_prev_full_rotations = m_full_rotations;
    m_prev_angle_radians = curr_radians;
    m_prev_radians_per_sec = radians_per_second;
    m_prev_microseconds = curr_microseconds;
}
#endif

#if 0
void AS5048A::update()
{

// m_invert_output

    const float MICROSECONDS_PER_SECOND(1000000.0f);
    
    float    curr_radians      = read_angle_radians();
    uint32_t curr_microseconds = _micros();   //DWT->CYCCNT;

    // TODO: handle rollover if curr_microseconds < m_prev_microseconds 

    float    delta_radians(curr_radians - m_prev_angle_radians);

    // if overflow happened track it as full rotation
    if(fabs(delta_radians) > (0.8f * TWO_PI) )
    {
       if(delta_radians > 0.0f )
       {
          m_full_rotations -= 1;
          delta_radians    += TWO_PI;
       }
       else
       {
           m_full_rotations += 1;
           delta_radians    -= TWO_PI;

       }
    }

    int32_t  delta_rotations(m_full_rotations - m_prev_full_rotations);
    float    delta_rotations_f(static_cast<float>(delta_rotations));
    float    delta_rotation_radians(TWO_PI * delta_rotations_f);

   // float    delta_radians_combined = delta_rotation_radians + delta_radians;

    uint32_t delta_microseconds(curr_microseconds - m_prev_microseconds);
    float    delta_microseconds_f(static_cast<float>(delta_microseconds));

    float    radians_per_second = MICROSECONDS_PER_SECOND 
                               // * delta_radians_combined / delta_microseconds_f;
                                * delta_radians / delta_microseconds_f;
    // update state variables
    m_prev_full_rotations  = m_full_rotations;
    m_prev_angle_radians   = curr_radians;
    m_prev_radians_per_sec = radians_per_second;
    m_prev_microseconds    = curr_microseconds;
}
#endif

float AS5048A::calculate_delta_angle(uint16_t last_angle, uint16_t current_angle)
{
	const uint16_t AS5048_MAX(0x4000);
    int32_t delta_angle = current_angle - last_angle;

    if (delta_angle > AS5048_MAX / 2) {
        delta_angle -= AS5048_MAX;  // Handle rollover
    }
    else if (delta_angle < -AS5048_MAX / 2) {
        delta_angle += AS5048_MAX;
    }

    // Convert to radians
    return (static_cast<float>(delta_angle) * 2.0f * M_PI) / static_cast<float>(AS5048_MAX);
}



//-----------------------------------------------------------------------------
//                          get_radians_per_second
//-----------------------------------------------------------------------------
float AS5048A::get_radians_per_second() 
{
    return m_prev_radians_per_sec;
}
void AS5048A::set_prev_radians_per_sec(float val)
{
    m_prev_radians_per_sec = val;
}

//-----------------------------------------------------------------------------
//                          invert_output
//-----------------------------------------------------------------------------
void  AS5048A::invert_output(bool invert)
{
   m_invert_output = invert;
}


//-----------------------------------------------------------------------------
//                          read_angle_radians
//-----------------------------------------------------------------------------

static uint16_t previous_count     = 0;  // Previous encoder reading

uint16_t readWithDeadband(uint16_t count)
{
    
    const uint16_t  DEADBAND_THRESHOLD = 5;  // Noise threshold for stationary encoder
    
    if (abs(static_cast<int>(count) - static_cast<int>(previous_count)) > DEADBAND_THRESHOLD) {
        previous_count = count;  // Update only if change exceeds the threshold
    }
    return previous_count;
}

uint16_t convertToSineDAC(uint16_t count) 
{
    
    const uint16_t  DAC_MAX_VALUE      = 0xFFF;    // Maximum value for 12-bit DAC (4095)
    const uint16_t  MAX_POSITION       = 0x3FFF;    // Maximum value for 14-bit encoder (16383)
    const uint16_t  DAC_MIDPOINT       = DAC_MAX_VALUE / 2;  // Center value for DAC (2047)


    // Apply deadband filtering to the encoder value
    uint16_t filtered_count = readWithDeadband(count);

    // Normalize the encoder value to an angle in radians [0, 2*PI)
    float angle = static_cast<float>(filtered_count) / MAX_POSITION * 2.0f * M_PI;

    // Calculate the sine of the angle, scaled to the DAC range
    float sine_value = std::sin(angle);  // Output range: [-1, 1]
    //float sine_value = arm_sin_f32(angle);  // Output range: [-1, 1]

    // Ensure angle is within expected range
     if (angle < 0.0f)
     {
        angle += 2.0f * M_PI;
     }
     else if (angle >= 2.0f * M_PI) 
     {
        angle -= 2.0f * M_PI;
     }

    // Scale sine value to the DAC range centered around DAC_MIDPOINT
    uint16_t dac_value = static_cast<uint16_t>(DAC_MIDPOINT + sine_value * (DAC_MAX_VALUE / 2.0f));

    // Clamp dac_value to avoid going out of range
    dac_value = std::min(dac_value, DAC_MAX_VALUE);    

    

    // Scale sine value to the DAC range centered around DAC_MIDPOINT
    //uint16_t dac_value = static_cast<uint16_t>(DAC_MIDPOINT + sine_value * (DAC_MAX_VALUE / 2.0f));
    
    // Ensure dac_value is never below zero
    //dac_value = std::max(dac_value, static_cast<uint16_t>(0));
    
    // Optional: Clip to DAC range
    //dac_value = std::min(dac_value, DAC_MAX_VALUE);

    return dac_value;
}

#if 0
const int FILTER_SIZE = 8;  // Adjust filter size based on noise level
static uint16_t filter_buffer[FILTER_SIZE] = {0};
static int filter_index = 0;

uint16_t smoothDACOutput(uint16_t new_value) {
    filter_buffer[filter_index] = new_value;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    uint32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += filter_buffer[i];
    }
    return static_cast<uint16_t>(sum / FILTER_SIZE);
}
#endif

const float SMOOTHING_FACTOR = 0.2f;  // Adjust between 0 (no smoothing) and 1 (max smoothing)
static float previous_dac_value = 0;  // Persistent DAC value

uint16_t smoothDACOutput(uint16_t new_value) 
{
    //previous_dac_value = SMOOTHING_FACTOR * static_cast<float>(new_value) + (1.0f - SMOOTHING_FACTOR) * previous_dac_value;
    //return static_cast<uint16_t>(previous_dac_value);
    return new_value;
}


float AS5048A::read_angle_radians_v2()
{
    float radians = TWO_PI
                  * static_cast<float>(m_register_value)
                  / static_cast<float>(COUNTS_PER_REVOLUTION);
    
    float result = (m_invert_output) 
                 ? -radians 
                 :  radians;
}


float AS5048A::read_angle_radians()
{
    uint16_t raw_count = get_raw_count();

    float radians = TWO_PI
                  * static_cast<float>(raw_count)
                  / static_cast<float>(COUNTS_PER_REVOLUTION);
    
    float result = (m_invert_output) 
                 ? -radians 
                 :  radians;

//---------------------
#if 0
const uint16_t  DAC_MAX_VALUE      = 0xFFF;    // Maximum value for 12-bit DAC (4095)
const uint16_t  MAX_POSITION       = 0x3FFF;    // Maximum value for 14-bit encoder (16383)
const uint16_t  DAC_MIDPOINT       = DAC_MAX_VALUE / 2;  // Center value for DAC (2047)


// Apply deadband filtering to the encoder value
uint16_t filtered_count = readWithDeadband(count);

// Normalize the encoder value to an angle in radians [0, 2*PI)
float angle = static_cast<float>(filtered_count) / MAX_POSITION * 2.0f * M_PI;

// Calculate the sine of the angle, scaled to the DAC range
float sine_value = std::sin(angle);  // Output range: [-1, 1]

// Ensure angle is within expected range
 if (angle < 0.0f)
 {
    angle += 2.0f * M_PI;
 }
 else if (angle >= 2.0f * M_PI) 
 {
    angle -= 2.0f * M_PI;
 }

#endif

//------------------------
//   DAC output debug 

    // Scale sine value to the DAC range centered around DAC_MIDPOINT
   // uint16_t dac_value = static_cast<uint16_t>(DAC_MIDPOINT + sine_value * (DAC_MAX_VALUE / 2.0f));

    // Clamp dac_value to avoid going out of range
   // dac_value = std::min(dac_value, DAC_MAX_VALUE);    

    uint16_t new_dac_value = convertToSineDAC(raw_count);
    new_dac_value = smoothDACOutput(new_dac_value);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, new_dac_value);


//---------------------------




    
    //char buff[50];
    //sprintf(buff, "raw count A: 0x%lX  res:%ld\r\n", raw_count, static_cast<uint32_t>(1000.0f *result));
    //HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(buff), strlen(buff), HAL_MAX_DELAY);

    return result;
}

//-----------------------------------------------------------------------------
//                          get_angle_radians
//-----------------------------------------------------------------------------
float AS5048A::get_angle_radians()
{  
  return  m_prev_angle_radians;
}

//-----------------------------------------------------------------------------
//                       convert_count_to_degrees
//-----------------------------------------------------------------------------
float AS5048A::convert_count_to_degrees(uint16_t count)
{
    
  float  f_count(static_cast<float>(count));
    
  return f_count * 360.0f / static_cast<float>(COUNTS_PER_REVOLUTION);
}




void AS5048A::conversion_complete()
{    
    m_register_value &= ~0xC000;  // Strip parity and error bits
         
    update_buffers(m_register_value, _micros());
}
//-----------------------------------------------------------------------------
//                          get_raw_count
//-----------------------------------------------------------------------------
//uint32_t AS5048A::get_raw_count()
//{
//   uint16_t count = read_register(value_of(AS5048A_REGISTERS::ANGLE_14_BITS));

 //  return static_cast<uint32_t>(count);
//}


//-----------------------------------------------------------------------------
//                          get_state
//-----------------------------------------------------------------------------
uint16_t AS5048A::get_state()
{
    uint16_t state = read_register(value_of(AS5048A_REGISTERS::DIAG_AND_AGC));
    
	return static_cast<uint32_t>(state);
}

//-----------------------------------------------------------------------------
//                          error_detected
//-----------------------------------------------------------------------------
bool AS5048A::error_detected()
{
	return m_error_detected;
}

//-----------------------------------------------------------------------------
//                          clear_error
//-----------------------------------------------------------------------------
void AS5048A::clear_error()
{
	read_register(value_of(AS5048A_REGISTERS::CLEAR_ERROR_FLAG));
}

//-----------------------------------------------------------------------------
//                          get_gain
//-----------------------------------------------------------------------------
uint8_t AS5048A::get_gain()
{
	uint16_t data = get_state();
    
	return static_cast<uint8_t>(data) & 0xFF;
}

//-----------------------------------------------------------------------------
//                          get_diagnostic
//-----------------------------------------------------------------------------
uint8_t AS5048A::get_diagnostic()
{
	uint16_t data = get_state() && 0xFF00;
    
	return static_cast<uint8_t>(data >> 8);
}

//-----------------------------------------------------------------------------
//                          get_errors
//-----------------------------------------------------------------------------
uint16_t AS5048A::get_errors()
{
	return read_register(value_of(AS5048A_REGISTERS::CLEAR_ERROR_FLAG));
}

//-----------------------------------------------------------------------------
//                          set_zero_position
//-----------------------------------------------------------------------------
void AS5048A::set_zero_position_count(uint16_t position_count)
{
	m_position_count = position_count % COUNTS_PER_REVOLUTION;
}

//-----------------------------------------------------------------------------
//                          get_zero_position
//-----------------------------------------------------------------------------
uint16_t AS5048A::get_zero_position_count()
{
	return m_position_count;
}

//-----------------------------------------------------------------------------
//                          normalize_angle_degrees
//-----------------------------------------------------------------------------
float AS5048A::normalize_angle_degrees(float angle_degrees)
{
// m_invert_output

// http://stackoverflow.com/a/11498248/3167294
	#ifdef ANGLE_MODE_1
		angle_degrees += 180;
	#endif

	angle_degrees = fmod(angle_degrees, 360);

	if (angle_degrees < 0)
	{
		angle_degrees += 360;
	}

	#ifdef ANGLE_MODE_1
		angle_degrees -= 180;
	#endif

	return angle_degrees;
}


//-----------------------------------------------------------------------------
//                               delay_microseconds
//-----------------------------------------------------------------------------
void AS5048A::delay_microseconds(volatile uint32_t microseconds)
 {
     uint32_t clk_cycle_start = DWT->CYCCNT;
     uint32_t clk_cycle_end   = clk_cycle_start + microseconds * (HAL_RCC_GetHCLKFreq() / 1000000);
     while (DWT->CYCCNT < clk_cycle_end);

     return;
 }

 //-----------------------------------------------------------------------------
 //                               read_register
 //-----------------------------------------------------------------------------
uint16_t AS5048A::read_register(uint16_t reg_address)
{
// disable
return 0xBEEF;


	uint16_t command = 0x4000;    // PAR = 0 R/W=R
	command = command | reg_address;

	//Add a parity bit on the the MSB
	command |= static_cast<uint16_t>(spiCalcEvenParity(command) << 0xF);

    const uint32_t TIMEOUT(1000); // rather than HAL_MAX_DELAY


    HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_RESET);

    uint16_t register_value;
    HAL_SPI_TransmitReceive( m_hspi,
                        reinterpret_cast<uint8_t*>(&command),
						reinterpret_cast<uint8_t*>(&register_value),
                        1,
  					  TIMEOUT);
    while (HAL_SPI_GetState(m_hspi) != HAL_SPI_STATE_READY) {}


    HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_SET);

    m_error_detected = (register_value & 0x4000);

    //uint16_t angle_cnt  = ((register_value & 0xFF) << 8)
    //                    | ((register_value >> 8) & 0xFF);  // Swap MSB and LSB

    // Strip parity and error bits
    return register_value & ~0xC000;
}

extern "C"
{
    void SPI_TxRx_completion_callback(SPI_HandleTypeDef *hspi);
}

// Function to initiate the SPI read
void AS5048A::read_register_async(uint16_t reg_address)
{
    uint16_t command = 0x4000;    // PAR = 0 R/W=R
    command = command | reg_address;

    // Add a parity bit on the MSB
    command |= static_cast<uint16_t>(spiCalcEvenParity(command) << 15);

#if 0
    HAL_SPI_TransmitReceive_IT(m_hspi, 
                               reinterpret_cast<uint8_t*>(&command),
                               reinterpret_cast<uint8_t*>(&m_register_value), 
                               sizeof(m_register_value));   
#else
    if (HAL_SPI_GetState(m_hspi) == HAL_SPI_STATE_READY)
    {
        HAL_SPI_TransmitReceive_IT(m_hspi, 
                                   reinterpret_cast<uint8_t*>(&command),
                                   reinterpret_cast<uint8_t*>(&m_register_value), 
                                   1);   
    }
    else
    {
        // SPI is busy
    }
#endif
                               
}

void AS5048A::async_read_angle()
{ 
    // reset guard conditions
    m_async_read_complete = false; 


    uint16_t READ_ANGLE_COMMAND(0xFFFF);
    volatile uint16_t temp = READ_ANGLE_COMMAND;
    
    if (HAL_SPI_GetState(m_hspi) == HAL_SPI_STATE_READY)
    {
    	 __HAL_SPI_ENABLE_IT(m_hspi, (SPI_IT_RXNE | SPI_IT_ERR));
         
        volatile HAL_SPI_StateTypeDef state = m_hspi->State;
        //SPI_TxRx_completion_callback(m_hspi);
        HAL_SPI_TransmitReceive_IT(m_hspi, 
                                   reinterpret_cast<uint8_t*>(&READ_ANGLE_COMMAND),
                                   reinterpret_cast<uint8_t*>(&m_register_value), 
                                   1);   
        state = m_hspi->State;


        if (__HAL_SPI_GET_FLAG(m_hspi, SPI_FLAG_OVR))
        {
            printf("SPI Overrun Error Detected\n");
            __HAL_SPI_CLEAR_OVRFLAG(m_hspi);
        }
        if (__HAL_SPI_GET_FLAG(m_hspi, SPI_FLAG_MODF))
        {
            printf("SPI Mode Fault Error Detected\n");
            __HAL_SPI_CLEAR_MODFFLAG(m_hspi);
        }



        
        temp++;
    }
    else
    {
        // SPI is busy
    	temp++;
    }
}

void AS5048A::init_SPI_buffers(void)
{
    // Initialize buffers to zero
    memset(m_timestamp_buffer, 0, sizeof(m_timestamp_buffer));
    memset(m_angle_buffer, 0, sizeof(m_angle_buffer));
    
    // Reset index and flags
    //m_current_index = 0;            // handled by Ctor
    //m_async_read_complete = false;   // handled by Ctor
}

void AS5048A::update_buffers(uint16_t new_angle, uint32_t new_timestamp)
{
    // Update the current position in the circular buffer
    m_angle_buffer[m_current_index] = new_angle;
    m_timestamp_buffer[m_current_index] = new_timestamp;

    // Move the index forward (circularly)
    m_current_index = (m_current_index + 1) % SPI_BUFFER_SIZE;

    // Mark that data is ready for processing
    m_async_read_complete = true;


    
    g_as5048_u16_angle = new_angle;
}

float AS5048A::read_angle_radians_from_buffer()
{
    HAL_NVIC_DisableIRQ(SPI2_IRQn); 


    // Check if there is valid data in the buffer
    if (m_current_index < 1)
    {
        HAL_NVIC_EnableIRQ(SPI2_IRQn);
        
        // Return 0 if the buffer has not been populated yet
        return 0.0f;
    }

    // The most recent angle value is at the last index in the circular buffer
    uint16_t latest_angle_u16 = m_angle_buffer[(m_current_index - 1 + SPI_BUFFER_SIZE) % SPI_BUFFER_SIZE];

    HAL_NVIC_EnableIRQ(SPI2_IRQn);


    // Convert the 14-bit angle data to radians
    // AS5048A has a 14-bit resolution (0 to 16383 -> 0 to 2π radians)
    float angle_radians = (static_cast<float>(latest_angle_u16) * TWO_PI) / AS5048_MAX;

    return angle_radians;
}

void AS5048A::process_encoder_data()
{
    static uint32_t prev_timestamp = 0;
    static uint16_t prev_angle     = 0;

    // Retrieve current timestamp and angle
    uint32_t curr_timestamp = m_timestamp_buffer[m_current_index];
    uint16_t curr_angle     = m_angle_buffer[m_current_index];

    // Compute time difference
    uint32_t delta_time_us = curr_timestamp - prev_timestamp;
    float    delta_time_s  = static_cast<float>(delta_time_us) * 0.000001f;

    // Compute angle difference (handle wrapping)
    float delta_angle = calculate_delta_angle(prev_angle, curr_angle);

    float velocity = delta_angle / delta_time_s;

    prev_angle     = curr_angle;
    prev_timestamp = curr_timestamp;
}

#if 1
float AS5048A::calculate_velocity_from_buffer(void)
{
    int32_t angle_diff_total = 0;  // Accumulate total angular difference
    uint32_t time_total_us = 0;    // Accumulate total time difference (in microseconds)

    // Ensure at least 2 samples are available
    if (m_current_index < 2) return 0.0f;

    // Calculate velocity based on the entire buffer
    for (int i = 1; i < SPI_BUFFER_SIZE; i++)
    {
        // Previous and current index (handle wrap-around)
        int current_idx = (m_current_index + i) % SPI_BUFFER_SIZE;
        int previous_idx = (current_idx == 0) ? SPI_BUFFER_SIZE - 1 : current_idx - 1;

        // Calculate angle difference (handle rollover)
        int32_t angle_diff = m_angle_buffer[current_idx] - m_angle_buffer[previous_idx];
        if (angle_diff > AS5048_MAX / 2)
        {
            angle_diff -= AS5048_MAX;  // Handle wrap-around
        }
        else if (angle_diff < -AS5048_MAX / 2)
        {
            angle_diff += AS5048_MAX;
        }

        uint32_t time_diff = m_timestamp_buffer[current_idx] 
                           - m_timestamp_buffer[previous_idx];
               
        if (time_diff > 0)
        {
           angle_diff_total += angle_diff;
           time_total_us    += time_diff;
        }   
    }

    // Ensure we have non-zero time difference to avoid division by zero
    //if (time_total_us == 0) return 0.0f;

    // Convert the total angle difference to radians
    //float angle_diff_radians = (static_cast<float>(angle_diff_total) * 2.0f * M_PI) / AS5048_MAX;
    float angle_diff_radians = (float)(angle_diff_total) * 2.0f * M_PI / AS5048_MAX;

    // Convert the time from microseconds to seconds
    float time_total_seconds = (float)(time_total_us) * 1e-6f;

    // Calculate velocity (radians per second)
    return angle_diff_radians / time_total_seconds;
}
#endif

#if 0
float AS5048A::calculate_velocity_from_buffer(void)
{
    int32_t angle_diff_total = 0;  // Accumulate total angular difference
    uint32_t time_total_us = 0;    // Accumulate total time difference (in microseconds)

    // Ensure at least 2 samples are available
    if (m_current_index < 2) return 0.0f;

    for (int i = 1; i < SPI_BUFFER_SIZE; i++) {
        int current_idx = (m_current_index + i) % SPI_BUFFER_SIZE;
        int previous_idx = (current_idx == 0) ? SPI_BUFFER_SIZE - 1 : current_idx - 1;

        // Calculate angle difference (handle rollover)
        int32_t angle_diff = m_angle_buffer[current_idx] - m_angle_buffer[previous_idx];

        // Handle wrapping cases
        if (angle_diff > AS5048_MAX / 2) {
            angle_diff -= AS5048_MAX;
        } else if (angle_diff < -AS5048_MAX / 2) {
            angle_diff += AS5048_MAX;
        }

        // Accumulate angle difference and time difference
        angle_diff_total += angle_diff;

        uint32_t time_diff = m_timestamp_buffer[current_idx] - m_timestamp_buffer[previous_idx];
        
        if (time_diff <= 0) {
            // Debugging: Invalid timestamp difference
            return 0.0f;
        }

        time_total_us += time_diff;
    }

    // Ensure we have non-zero time difference to avoid division by zero
    if (time_total_us == 0) return 0.0f;

    // Convert angle difference to radians
    float angle_diff_radians = static_cast<float>(angle_diff_total) * 2.0f * M_PI / AS5048_MAX;

    // Convert time difference to seconds
    float time_total_seconds = static_cast<float>(time_total_us) * 1e-6f;

    // Calculate velocity (radians per second)
    float rad_per_sec = angle_diff_radians / time_total_seconds;

    // Optional: Debugging printouts
    // printf("Angle Diff: %f, Time Diff: %f, Velocity: %f\n", angle_diff_radians, time_total_seconds, rad_per_sec);

    return rad_per_sec;
}
#endif





bool AS5048A::request_raw_count()
{
    bool success(false);

    
    if(m_async_read_complete)
    {
	   m_async_read_complete = false;  // Reset the flag

       // Initiate the SPI read
       read_register_async(value_of(AS5048A_REGISTERS::ANGLE_14_BITS) );
       success = true;
    }

    return success;
}

uint16_t AS5048A::get_current_raw_count()
{
   return m_register_value;
}

uint16_t AS5048A::blocking_get_raw_count()
{
    #if 0
    while(!request_raw_count());
    while(!async_read_complete());
    return get_current_raw_count();
    #else
    return get_raw_count();
    #endif
}

uint16_t AS5048A::get_raw_count()
{
    uint16_t result = 0;

    //if(m_async_read_complete)
    {
//	   m_async_read_complete = false;  // Reset the flag

       // Initiate the SPI read
 //      read_register_async(value_of(AS5048A_REGISTERS::ANGLE_14_BITS) );  // Assuming AS5048A_ANGLE_REG is the register address

       // Wait for the SPI read to complete (polling or can be modified for task-based waiting)
 //      while (!m_async_read_complete)
 //      {
        // You could add a timeout here to prevent infinite waiting
//       }
       result = m_register_value;
    }

    // Return the received value
    return result;
}







//-----------------------------------------------------------------------------
//                               write_register
//
// TODO: Ugly alert
//-----------------------------------------------------------------------------
uint16_t AS5048A::write_register(uint16_t registerAddress, uint16_t data)
{
    //disable
return 0xDEAD;

    
	uint8_t dat[2];

	uint16_t command = 0b0000000000000000; // PAR=0 R/W=W
	command |= registerAddress;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	dat[1] = command & 0xFF;
	dat[0] = ( command >> 8 ) & 0xFF;

	//Start the write command with the target address
	HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(m_hspi, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(m_hspi) != HAL_SPI_STATE_READY) {}
	HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_SET);

	uint16_t dataToSend = 0b0000000000000000;
	dataToSend |= data;

	//Craft another packet including the data and parity
	dataToSend |= ((uint16_t)spiCalcEvenParity(dataToSend)<<15);
	dat[1] = command & 0xFF;
	dat[0] = ( command >> 8 ) & 0xFF;

	//Now send the data packet
	HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(m_hspi, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(m_hspi) != HAL_SPI_STATE_READY) {}
	HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_SET);

	//Send a NOP to get the new data in the register
	dat[1] = 0x00;
	dat[0] = 0x00;
	HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(m_hspi, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(m_hspi) != HAL_SPI_STATE_READY) {}
	HAL_SPI_Receive(m_hspi, (uint8_t *)&dat, 2, 0xFFFF);
	while (HAL_SPI_GetState(m_hspi) != HAL_SPI_STATE_READY) {}
	HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_SET);

	//Return the data, stripping the parity and error bits
	return (( ( dat[1] & 0xFF ) << 8 ) | ( dat[0] & 0xFF )) & ~0xC000;
}


//-----------------------------------------------------------------------------
//                               spiCalcEvenParity
//-----------------------------------------------------------------------------
uint8_t AS5048A::spiCalcEvenParity(uint16_t value)
{
	uint8_t cnt(0);

	for (uint8_t i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
    
	return cnt & 0x1;
}

#if 0

uint8_t AS5048A::spiCalcEvenParity(uint16_t value)
{
    uint8_t cnt = 0;
    while (value)
    {
        cnt ^= value & 1;  // XOR the least significant bit
        value >>= 1;       // Shift right by 1 bit
    }
    return cnt & 1;  // Return the last XOR result (0 or 1)
}

#endif


//-----------------------------------------------------------------------------
//                      get_counts_advanced_past_position
//-----------------------------------------------------------------------------
int16_t AS5048A::get_counts_advanced_past_position()
{
	uint16_t count(get_raw_count());

    //char buff[50];
    //sprintf(buff, "raw count B: 0x%X\r\n", count);
    //HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(buff), strlen(buff), HAL_MAX_DELAY);

	int16_t rotation = static_cast<int16_t>(count)
			         - static_cast<int16_t>(m_position_count);

	if (rotation > COUNTS_PER_HALF_REVOLUTION)
	{
		rotation = -((COUNTS_PER_REVOLUTION) - rotation);
	}

	return rotation;
}

//-----------------------------------------------------------------------------
//                      getMechanicalAngle
//-----------------------------------------------------------------------------
float AS5048A::get_mechanical_phase_angle_radians() 
{
    return m_prev_angle_radians;
}

//-----------------------------------------------------------------------------
//                      get_accumulated_radians
//-----------------------------------------------------------------------------
float AS5048A::get_accumulated_radians()
{
    return m_prev_angle_radians +
          (TWO_PI * static_cast<float>(m_full_rotations));
}


