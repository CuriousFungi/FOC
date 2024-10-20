
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
extern SPI_HandleTypeDef hspi2;

#include <string.h>

// define the static members
uint32_t AS5048A::spi_timestamp_buffer[SPI_BUFFER_SIZE]   __attribute__ ((section(".spi_buffers_4"))) ;
uint16_t AS5048A::spi_angle_buffer[SPI_BUFFER_SIZE]       __attribute__ ((section(".spi_buffers_2"))) ;
uint16_t AS5048A::spi_as5048_register_value  __attribute__ ((section(".spi_buffers_4"))) =0;
uint16_t AS5048A::spi_index_curr                      __attribute__ ((section(".spi_buffers_2"))) =0;       
uint16_t AS5048A::spi_index_prev                      __attribute__ ((section(".spi_buffers_2"))) =0;       

bool     AS5048A::spi_async_read_complete                 __attribute__ ((section(".spi_buffers_4"))) ;
bool     AS5048A::clear_error_in_progress;


//=============================================================================
//                         local utils
//=============================================================================

const float SMOOTHING_FACTOR = 0.2f;  // Adjust between 0 (no smoothing) and 1 (max smoothing)
//static float previous_dac_value = 0;  // Persistent DAC value

uint16_t smoothDACOutput(uint16_t new_value) 
{
    //previous_dac_value = SMOOTHING_FACTOR * static_cast<float>(new_value) + (1.0f - SMOOTHING_FACTOR) * previous_dac_value;
    //return static_cast<uint16_t>(previous_dac_value);
    return new_value;
}


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

//=============================================================================
//                           
//=============================================================================
#if 0
extern "C"
{
    void SPI_TxRx_completion_callback(SPI_HandleTypeDef *hspi);
}
#endif

//=============================================================================
//                           AS5048A members
//=============================================================================

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
//,   m_clock_speed(1000000)
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
//,   spi_async_read_complete(true)
//,   spi_as5048_register_value(0)
//,   spi_current_index(0)
{
    init_SPI_buffers();
    HAL_GPIO_WritePin(m_p_chip_select_port, m_chip_select_pin, GPIO_PIN_SET);

    AS5048A::clear_error_in_progress = false;

    AS5048A::spi_index_curr= 0;
    AS5048A::spi_index_prev = 0;


    // only for debug purpose
    memset(spi_timestamp_buffer,0xFFFFFFFF, AS5048A::SPI_BUFFER_SIZE*sizeof(uint32_t));

    return;
}


extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;


void AS5048A::reinit_dma_for_spi() 
{
    spi_async_read_complete = true;
#if 0
    // Disable DMA to reset it
    __HAL_DMA_DISABLE(&hdma_spi2_rx);
    __HAL_DMA_DISABLE(&hdma_spi2_tx);


    while (((DMA_Stream_TypeDef *)(hdma_spi2_rx.Instance))->CR & DMA_SxCR_EN)
    {
        // Wait for DMA to fully disable
    }
    while (((DMA_Stream_TypeDef *)(hdma_spi2_tx.Instance))->CR & DMA_SxCR_EN) 
    {
        // Wait for TX DMA to fully disable
    }


    // Clear relevant flags
    __HAL_DMA_CLEAR_FLAG(&hdma_spi2_rx, DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TEIF0_4);
    __HAL_DMA_CLEAR_FLAG(&hdma_spi2_tx, DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TEIF0_4);

    // Set new parameters if needed, e.g., buffer address, length
    if (HAL_DMA_Start(&hdma_spi2_rx, (uint32_t)&.Instance->RXDR, (uint32_t)&spi_as5048_register_value, 1) != HAL_OK)
    {
        // Handle DMA start failure (e.g., set error flag, retry, or log the error)
    }

    uint16_t READ_ANGLE_COMMAND(0xFFFF);


    // Start TX DMA
    if (HAL_DMA_Start(&hdma_spi2_tx, (uint32_t)&READ_ANGLE_COMMAND, (uint32_t)&hspi2.Instance->TXDR, 1) != HAL_OK) 
    {
         // Handle TX DMA start failure
    }


    // Enable DMA again
    __HAL_DMA_ENABLE(&hdma_spi2_rx);
    __HAL_DMA_ENABLE(&hdma_spi2_tx);
    #else
    // Check if there's an error in the SPI peripheral
    if (HAL_SPI_GetError(&hspi2) != HAL_SPI_ERROR_NONE) {
        // Deinitialize and reinitialize the SPI to recover
        HAL_SPI_DeInit(&hspi2);
        HAL_SPI_Init(&hspi2);
    }
    uint16_t READ_ANGLE_COMMAND(0xFFFF);
    // Attempt to reinitiate DMA transfer if needed
    if (HAL_SPI_TransmitReceive_DMA(&hspi2,
           (uint8_t*)&READ_ANGLE_COMMAND, 
           (uint8_t*)&spi_as5048_register_value, 
           1) != HAL_OK) 
    {
        // Handle DMA reinit failure
        //printf("Failed to restart SPI DMA transfer\n");
    }
    #endif
}



#if 0
//-----------------------------------------------------------------------------
//                                update
//
//                            Pre DMA approach
//-----------------------------------------------------------------------------
void AS5048A::update()
{
    const float MICROSECONDS_PER_SECOND = 1000000.0f;
    const float MAX_RADIANS_CHANGE = TWO_PI * 0.2f; // Example threshold to detect large spikes
    const float ALPHA = 0.1f;  // Smoothing factor for radians per second

    float curr_radians = read_angle_radians();
    uint32_t curr_microseconds = micros();

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
#endif

#if 0
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
#endif

#if 0
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
#endif

//-----------------------------------------------------------------------------
//                          invert_output
//-----------------------------------------------------------------------------
void  AS5048A::invert_output(bool invert)
{
   m_invert_output = invert;
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


    //------------------------
    //   DAC output debug 

    // Scale sine value to the DAC range centered around DAC_MIDPOINT
    // uint16_t dac_value = static_cast<uint16_t>(DAC_MIDPOINT + sine_value * (DAC_MAX_VALUE / 2.0f));

    // Clamp dac_value to avoid going out of range
    // dac_value = std::min(dac_value, DAC_MAX_VALUE);    

    uint16_t new_dac_value = convertToSineDAC(raw_count);
    new_dac_value = smoothDACOutput(new_dac_value);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, new_dac_value);
    
    return result;
}

//-----------------------------------------------------------------------------
//                          get_angle_radians
//-----------------------------------------------------------------------------
float AS5048A::get_angle_radians()
{  
  return read_angle_radians_from_buffer();
}

#if 0
//-----------------------------------------------------------------------------
//                       convert_count_to_degrees
//-----------------------------------------------------------------------------
float AS5048A::convert_count_to_degrees(uint16_t count)
{
    
  float  f_count(static_cast<float>(count));
    
  return f_count * 360.0f / static_cast<float>(COUNTS_PER_REVOLUTION);
}
#endif



//-----------------------------------------------------------------------------
//                       conversion_complete
//-----------------------------------------------------------------------------
void AS5048A::conversion_complete()
{    
    update_buffers(spi_as5048_register_value & ~0xC000, micros());
}

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

#if 0

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
#endif

#if 0
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
#endif

#if 0
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
#endif

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


#if 0
//-----------------------------------------------------------------------------
//                               read_register_async
//
//                       HAL_SPI_TransmitReceive_IT version
//-----------------------------------------------------------------------------
void AS5048A::read_register_async(uint16_t reg_address)
{
    uint16_t command = 0x4000;    // PAR = 0 R/W=R
    command = command | reg_address;

    // Add a parity bit on the MSB
    command |= static_cast<uint16_t>(spiCalcEvenParity(command) << 15);

    if (HAL_SPI_GetState(m_hspi) == HAL_SPI_STATE_READY)
    {
        HAL_SPI_TransmitReceive_IT(m_hspi, 
                                   reinterpret_cast<uint8_t*>(&command),
                                   reinterpret_cast<uint8_t*>(&spi_as5048_register_value), 
                                   1);   
    }
    else
    {
        // SPI is busy
    }                       
}
#endif

//-----------------------------------------------------------------------------
//                              async_read_angle
//
//                      HAL_SPI_TransmitReceive_DMA version
//-----------------------------------------------------------------------------
void AS5048A::async_read_angle()
{ 
    if(!spi_async_read_complete) return;

    // reset guard conditions
    spi_async_read_complete = false; 

    uint16_t READ_ANGLE_COMMAND(0xFFFF);
    
    if (HAL_SPI_GetState(m_hspi) == HAL_SPI_STATE_READY)
    {
    	 __HAL_SPI_ENABLE_IT(m_hspi, (SPI_IT_RXNE | SPI_IT_ERR));
         
        if(HAL_OK !=  HAL_SPI_TransmitReceive_DMA(m_hspi, 
                                   reinterpret_cast<uint8_t*>(&READ_ANGLE_COMMAND),
                                   reinterpret_cast<uint8_t*>(&spi_as5048_register_value), 
                                   1))
        {
            spi_async_read_complete = true;
            return;
        }

#if 0
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
#endif
    }
    else
    {
        // SPI is busy
    }
}

//-----------------------------------------------------------------------------
//                          init_SPI_buffers
//-----------------------------------------------------------------------------
void AS5048A::init_SPI_buffers(void)
{
    AS5048A::spi_as5048_register_value = 0;
    // Initialize buffers to zero
   memset(spi_timestamp_buffer, 0, SPI_BUFFER_SIZE * sizeof(spi_timestamp_buffer[0]));
   //memset(spi_angle_buffer,     0, SPI_BUFFER_SIZE * sizeof(spi_angle_buffer[0]));
}


//-----------------------------------------------------------------------------
//                         update_buffers
//-----------------------------------------------------------------------------
void AS5048A::update_buffers(uint16_t new_angle, uint32_t new_timestamp)
{    
    __disable_irq();

    // Update the current position in the circular buffer
    spi_angle_buffer[    spi_index_curr] = new_angle;
    spi_timestamp_buffer[spi_index_curr] = new_timestamp;

    // Move the index forward (circularly)
    spi_index_prev = spi_index_curr;
    spi_index_curr = (spi_index_curr + 1) % SPI_BUFFER_SIZE;

    // Mark that data is ready for processing
    spi_async_read_complete = true;
    
    g_as5048_u16_angle = new_angle; 

    __enable_irq();
}


//-----------------------------------------------------------------------------
//                       read_angle_radians_from_buffer
//-----------------------------------------------------------------------------
float AS5048A::read_angle_radians_from_buffer()
{
    HAL_NVIC_DisableIRQ(SPI2_IRQn); 
    uint16_t latest_angle_u16 = spi_angle_buffer[spi_index_prev];
    HAL_NVIC_EnableIRQ(SPI2_IRQn);


    // Convert the 14-bit angle data to radians
    // AS5048A has a 14-bit resolution (0 to 16383 -> 0 to 2π radians)
    float angle_radians = (static_cast<float>(latest_angle_u16) * TWO_PI) / AS5048_MAX;

    return angle_radians;
}
//-----------------------------------------------------------------------------
//                       read_angle_radians_from_buffer
//-----------------------------------------------------------------------------
float AS5048A::read_radians_with_direction()
{
    float angle_radians = read_angle_radians_from_buffer();
    return (m_invert_output) ? -angle_radians : angle_radians;
}

#if 0
//-----------------------------------------------------------------------------
//                         process_encoder_data
//-----------------------------------------------------------------------------
void AS5048A::process_encoder_data()
{
    //static uint32_t prev_timestamp = 0;
    //static uint16_t prev_angle     = 0;

    // Retrieve current timestamp and angle
    //uint32_t curr_timestamp = spi_timestamp_buffer[spi_current_index];
    //uint16_t curr_angle     = spi_angle_buffer[    spi_current_index];

    // Compute time difference, handling rollover
  //  uint32_t delta_time_us = calculate_time_difference(curr_timestamp, prev_timestamp);
    //float delta_time_s = static_cast<float>(delta_time_us) * 0.000001f;

    // Compute angle difference (handle wrapping)
   // float delta_angle = calculate_delta_angle(prev_angle, curr_angle);

    //float velocity = delta_angle / delta_time_s;

    //prev_angle     = curr_angle;
    //prev_timestamp = curr_timestamp;
}
#endif

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
uint32_t AS5048A::
calculate_time_difference( uint32_t current_timestamp, 
                                      uint32_t last_timestamp)
{
    // Check for rollover and handle it correctly
    if (current_timestamp >= last_timestamp)
    {
        return current_timestamp - last_timestamp;
    }
    else
    {
        // Handle the rollover case for a 32-bit timer
        return (0xFFFFFFFF - last_timestamp) + current_timestamp + 1;
    }
}


#if 1
//-----------------------------------------------------------------------------
//                        calculate_velocity_from_buffer
//-----------------------------------------------------------------------------
void AS5048A::calculate_velocity_from_buffer(struct Sample &current_sample)
{
    int32_t  angle_diff_total = 0;    // Accumulate total angular difference
    uint32_t time_total_us    = 0;    // Accumulate total time difference (in microseconds)

    // Ensure at least 2 samples are available
    //if (spi_index_curr < 2) return;

    // Calculate velocity based on the entire buffer
    for (uint32_t i = 1; i < SPI_BUFFER_SIZE; i++)
    {
        // Previous and current index (handle wrap-around)
        int curr_idx = (spi_index_curr + i) % SPI_BUFFER_SIZE;
        int prev_idx = (curr_idx == 0) ? SPI_BUFFER_SIZE - 1 : curr_idx - 1;

        // Calculate angle difference (handle rollover)
        int32_t angle_diff = spi_angle_buffer[curr_idx] 
                           - spi_angle_buffer[prev_idx];
        
        if (angle_diff > AS5048_MAX / 2)
        {
            angle_diff -= AS5048_MAX;  // Handle wrap-around
        }
        else if (angle_diff < -AS5048_MAX / 2)
        {
            angle_diff += AS5048_MAX;
        }

        uint32_t time_diff = calculate_time_difference(
                              spi_timestamp_buffer[curr_idx], 
                              spi_timestamp_buffer[prev_idx]);                  
               
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

    current_sample.count            = spi_angle_buffer[SPI_BUFFER_SIZE - 1]; // latest sample for now
    current_sample.radians          = read_angle_radians_from_buffer();
    current_sample.radians_per_second = angle_diff_radians / time_total_seconds;


    return;
}
#endif






#if 0
bool AS5048A::request_raw_count()
{
    bool success(false);

    
    if(spi_async_read_complete)
    {
	   spi_async_read_complete = false;  // Reset the flag

       // Initiate the SPI read
       read_register_async(value_of(AS5048A_REGISTERS::ANGLE_14_BITS) );
       success = true;
    }

    return success;
}


uint16_t AS5048A::get_current_raw_count()
{
   return spi_as5048_register_value;
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
#endif

//-----------------------------------------------------------------------------
//                            get_raw_count
//-----------------------------------------------------------------------------
uint16_t AS5048A::get_raw_count()
{
    return spi_as5048_register_value;
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

    #if 1
    while (value)
    {
        cnt ^= value & 1;  // XOR the least significant bit
        value >>= 1;       // Shift right by 1 bit
    }
    return cnt & 1;  // Return the last XOR result (0 or 1)

    #else
	for (uint8_t i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}

    #endif
	return cnt & 0x1;
}

#if 0
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
#endif

//-----------------------------------------------------------------------------
//                      getMechanicalAngle
//-----------------------------------------------------------------------------
float AS5048A::get_mechanical_phase_angle_radians() 
{
    //return m_prev_angle_radians;
    return read_angle_radians_from_buffer();
}

#if 0
//-----------------------------------------------------------------------------
//                      get_accumulated_radians
//-----------------------------------------------------------------------------
float AS5048A::get_accumulated_radians()
{
    return read_angle_radians_from_buffer() +
          (TWO_PI * static_cast<float>(m_full_rotations));
}
#endif

