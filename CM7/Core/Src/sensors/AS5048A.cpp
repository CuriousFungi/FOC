
#include "AS5048A.hpp"
#include "../../Inc/EnumValue.hpp"
#include "../common/time_utils.hpp"
#include <math.h>
#include <cmath>  // For std::sin and M_PI
#include <algorithm>

//TODO: look at https://github.com/sosandroid/AMS_AS5048B/blob/master/ams_as5048b.cpp

#if 1 // DEBUG
extern UART_HandleTypeDef huart2;
#include <cstdio>
#include <cstring>
#endif

extern DAC_HandleTypeDef hdac1;

float g_angle_radians;


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
,   COUNTS_PER_REVOLUTION((1 << BIT_RESOLUTION)-1)
,   COUNTS_PER_HALF_REVOLUTION(COUNTS_PER_REVOLUTION >> 1)
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
{
    return;
}

//-----------------------------------------------------------------------------
//                          update
//-----------------------------------------------------------------------------
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

//-----------------------------------------------------------------------------
//                          get_radians_per_second
//-----------------------------------------------------------------------------
float AS5048A::get_radians_per_second() 
{
    return m_prev_radians_per_sec;
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

    // Ensure angle is within expected range
     if (angle < 0.0f)
     {
        angle += 2.0f * M_PI;
     }
     else if (angle >= 2.0f * M_PI) 
     {
        angle -= 2.0f * M_PI;
     }

     g_angle_radians = angle;

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





float AS5048A::read_angle_radians()
{

    uint32_t raw_count = get_raw_count();

    float count(static_cast<float>
               ( (m_invert_output) ? -raw_count : raw_count));
    
    
    float result =   TWO_PI * count /  static_cast<float>(COUNTS_PER_REVOLUTION);

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

g_angle_radians = angle;
#endif

//------------------------
//   DAC output debug 

    // Scale sine value to the DAC range centered around DAC_MIDPOINT
   // uint16_t dac_value = static_cast<uint16_t>(DAC_MIDPOINT + sine_value * (DAC_MAX_VALUE / 2.0f));

    // Clamp dac_value to avoid going out of range
   // dac_value = std::min(dac_value, DAC_MAX_VALUE);    

    uint16_t new_dac_value = convertToSineDAC(count);
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

//-----------------------------------------------------------------------------
//                          get_raw_count
//-----------------------------------------------------------------------------
uint32_t AS5048A::get_raw_count()
{
   uint16_t count = read_register(value_of(AS5048A_REGISTERS::ANGLE_14_BITS));

   return static_cast<uint32_t>(count);
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

//-----------------------------------------------------------------------------
//                               write_register
//
// TODO: Ugly alert
//-----------------------------------------------------------------------------
uint16_t AS5048A::write_register(uint16_t registerAddress, uint16_t data)
{
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


