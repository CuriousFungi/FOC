#ifndef AS5048A_H
#define AS5048A_H


#include <cstdint>

// 500 uS/25uS
#define SPI_BUFFER_SIZE 20


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"


#ifdef __cplusplus
}
#endif

enum class Direction : int8_t
{
    CW      =  1,  // clockwise
    CCW     = -1,  // counter clockwise
    UNKNOWN =  0   // not yet known or invalid state
};


/**
 *  Pullup configuration structure
 */
enum class Pullup : uint8_t
{
    USE_INTERN = 0x00, //!< Use internal pullups
    USE_EXTERN = 0x01  //!< Use external pullups
};



//=============================================================================
//                               AS5048A Class
//=============================================================================
class AS5048A
{
 public:

    //-------------------------------------------------------------------------
    //                              CTOR
    //-------------------------------------------------------------------------
    AS5048A(
                               SPI_HandleTypeDef* hspi, 
                               GPIO_TypeDef*      p_chip_select_port,
                               uint16_t           chip_select_pin);


    //-------------------------------------------------------------------------
    //                              update
    //-------------------------------------------------------------------------
    void update();

    //-------------------------------------------------------------------------
    //                              get_radians_per_second
    //-------------------------------------------------------------------------
    float get_radians_per_second(); 

    //-------------------------------------------------------------------------
    //                        getSensorAngle_Radians
    //-------------------------------------------------------------------------
    float get_angle_radians();

    //-------------------------------------------------------------------------
    //                        error_detected
    //-------------------------------------------------------------------------
    bool error_detected();

    uint16_t get_errors();
    void     clear_error();
    uint8_t  get_diagnostic();
    
    float    get_mechanical_phase_angle_radians();
    float    get_accumulated_radians();

    // TODO temporarily public
    void     delay_microseconds(volatile uint32_t microseconds);

    float read_angle_radians();
    float read_angle_radians_v2();
    
    void  invert_output(bool invert);
  bool     invert_output(){return m_invert_output;} // Temporary bridge

  uint16_t get_raw_count();
  void set_prev_radians_per_sec(float val);
  void conversion_complete();
  void read_register_async(uint16_t reg_address);
  float calculate_velocity_from_buffer(void);
  void update_buffers(uint16_t new_angle, uint32_t new_timestamp);
  void init_SPI_buffers(void);
  void process_encoder_data();
  float calculate_delta_angle(uint16_t last_angle, uint16_t current_angle);
  void timestamp(){m_timestamp_buffer[m_current_index] = DWT->CYCCNT / 84; } // _micros();
  bool async_read_complete(){return m_async_read_complete;}
  bool request_raw_count();
  uint16_t get_current_raw_count();
  uint16_t blocking_get_raw_count();
  float read_angle_radians_from_buffer();

  void async_read_angle();

  

  //  float read_angle_radians();

    enum class AS5048A_REGISTERS : uint16_t
    {
        NOP                         = 0x0000, // Read Only
        CLEAR_ERROR_FLAG            = 0x0001, // Read Only  
        PROGRAMMING_CONTROL         = 0x0003, // R/W
        OTP_REG_ZERO_POSN_HI_8_BITS = 0x0016, // R/W/Prog
        OTP_REG_ZERO_POSN_LO_6_BITS = 0x0017, // R/W/Prog
        DIAG_AND_AGC                = 0x3FFD, // Read Only
        MAGNITUDE_14_BITS           = 0x3FFE, // Read Only
        ANGLE_14_BITS               = 0x3FFF  
    };
        
    uint8_t  spiCalcEvenParity(uint16_t value);
        
    private:

    uint16_t read_register(uint16_t reg_address);

    int16_t  get_counts_advanced_past_position();

    uint16_t write_register(uint16_t registerAddress, uint16_t data);


    uint16_t get_state();
    uint8_t  get_gain();

    void     set_zero_position_count(uint16_t position_count);
    uint16_t get_zero_position_count();

    float    normalize_angle_degrees(float angle_degrees);
    
    float    convert_count_to_degrees(uint16_t count);

    const float        TWO_PI;
    const uint16_t     BIT_RESOLUTION;
    const uint16_t     READ_WRITE_BIT;
    const uint16_t     PARITY_BIT;
    const uint16_t     SPI_READ_ANGLE_CMD;
    const uint16_t     COUNTS_PER_REVOLUTION;
    const uint16_t     COUNTS_PER_HALF_REVOLUTION;
    const uint16_t     AS5048_MAX;
    
    SPI_HandleTypeDef* m_hspi;                // SPI handle
    GPIO_TypeDef*      m_p_chip_select_port;
    uint16_t           m_chip_select_pin;     //!< SPI chip select pin
    uint32_t           m_clock_speed;
    
    uint16_t           m_position_count;
    bool               m_error_detected;
    uint32_t           m_prev_timestamp_microseconds;
    float              m_prev_angle_radians;  // result of last call to getSensorAngle_Radians(), used for full rotations and velocity
    int32_t            m_full_rotations;
    int32_t            m_prev_full_rotations;

    // from sensor

    float              m_min_elapsed_time;
    
    float              m_velocity;
    
    long               m_prev_angle_timestamp_us;    // timestamp of last call to get_accumulated_radians, used for velocity
    float              m_prev_radians_per_sec;       // angle at last call to get_radians_per_second, used for velocity
    long               m_prev_velocity_timestamp_us; // last velocity calculation timestamp

    uint32_t           m_prev_microseconds;
    bool               m_invert_output;
    bool               m_async_read_complete;
    uint16_t           m_register_value;
    unsigned long      m_timestamp_buffer[SPI_BUFFER_SIZE];
    uint16_t           m_angle_buffer[SPI_BUFFER_SIZE];

    uint8_t            m_current_index;  // Index to track the circular buffer

    
};


#endif // Inclusion guard
