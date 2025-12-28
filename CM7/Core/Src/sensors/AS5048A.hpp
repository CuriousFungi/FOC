#ifndef AS5048A_H
#define AS5048A_H


#include <cstdint>
#include "KalmanFilter.hpp"


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"

#define NUM_RX_READINGS 2 //16
#define NUM_TX_READINGS 2 //16

  //  typedef volatile uint16_t DMAAlignedType __attribute__((aligned(4), section(".dma_rx_buffer"), used));

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

struct Sample
{
   Sample()
   : count(0)
   , radians(0.0f)
   , radians_per_second(0.0f)
   {
      return;
   }
   uint16_t count;
   float    radians;
   float    radians_per_second;
};


//=============================================================================
//                               AS5048A Class
//=============================================================================
class AS5048A
{
 public:

    
    // declare the static members
    static constexpr size_t SPI_BUFFER_SIZE = 20;  // 500 uS/25uS
    static uint32_t spi_timestamp_buffer[SPI_BUFFER_SIZE];
    static uint16_t spi_angle_buffer[SPI_BUFFER_SIZE]; 
    
    static uint16_t spi_index_curr ;     
    static uint16_t spi_index_prev ;      

    
    static  bool   clear_error_in_progress;
    //-------------------------------------------------------------------------
    //                              CTOR
    //-------------------------------------------------------------------------
    AS5048A(SPI_HandleTypeDef* hspi);


    //-------------------------------------------------------------------------
    //                              update
    //-------------------------------------------------------------------------
    //void update(); // Pre DMA approach

    //-------------------------------------------------------------------------
    //                              get_radians_per_second
    //-------------------------------------------------------------------------
    //float get_radians_per_second(); 


    //-------------------------------------------------------------------------
    //                        error_detected
    //-------------------------------------------------------------------------
    bool error_detected();

    void check_health();
    
    //void reinit_dma_for_spi(); 
    void   start_spi_conversion();

    uint16_t get_errors();
    void     clear_error();
    uint8_t  get_diagnostic();
    
    float    get_mechanical_phase_angle_radians();

    float    read_angle_radians();
    bool     is_sample_valid(uint16_t value){return (0 == (value & 0x4000));}
    
    void     invert_output(bool invert);
    bool     is_direction_invert(){return m_invert_output;} // Temporary bridge

    uint16_t get_raw_count();
    void     calculate_velocity_from_buffer(struct Sample &current_sample);
    void     update_buffers(uint16_t new_angle, uint32_t new_timestamp);
    void     init_SPI_buffers(void);
    bool     async_read_complete(){return m_spi_async_read_complete;}
    void     set_async_read_complete(){m_spi_async_read_complete = true;}
    void     spi_reset_in_progress(){m_spi_reset_in_progress = true;}
   // float    read_angle_radians_from_buffer();
    float    read_radians_with_direction();
    void     async_read_angle();
    uint16_t get_count(){return spi_angle_buffer[spi_index_prev];}
    
    bool     fetch_radians(float &result);

    uint32_t calculate_time_difference(uint32_t current_timestamp, uint32_t last_timestamp);

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
    uint16_t write_register(uint16_t registerAddress, uint16_t data);

    uint16_t get_state();
    uint8_t  get_gain();

    //void     set_zero_position_count(uint16_t position_count);
    //uint16_t get_zero_position_count();
    //float    normalize_angle_degrees(float angle_degrees);
    //float    convert_count_to_degrees(uint16_t count);

    const float        TWO_PI;
    const uint16_t     BIT_RESOLUTION;
    const uint16_t     READ_WRITE_BIT;
    const uint16_t     PARITY_BIT;
    const uint16_t     SPI_READ_ANGLE_CMD;
    const uint16_t     COUNTS_PER_REVOLUTION;
    const uint16_t     COUNTS_PER_HALF_REVOLUTION;
    const uint16_t     AS5048_MAX;
    
    SPI_HandleTypeDef* m_hspi;                // SPI handle
    
    uint16_t           m_position_count;
    bool               m_error_detected;
    uint32_t           m_prev_timestamp_microseconds;
    float              m_prev_angle_radians;  // result of last call to getSensorAngle_Radians(), used for full rotations and velocity
    int32_t            m_full_rotations;
    int32_t            m_prev_full_rotations;

    float              m_min_elapsed_time;
    
    float              m_velocity;
    
    long               m_prev_angle_timestamp_us;    // timestamp of last call to get_accumulated_radians, used for velocity
    float              m_prev_radians_per_sec;       // angle at last call to get_radians_per_second, used for velocity
    long               m_prev_velocity_timestamp_us; // last velocity calculation timestamp

    uint32_t           m_prev_microseconds;
    
    // Kalman filter for optimal velocity estimation
    KalmanFilter2D     m_kalman_filter;
    bool               m_invert_output;

   //uint8_t            spi_current_index;  // Index to track the circular buffer
    volatile bool       m_spi_async_read_complete;  // PRP try to disable the cache on this
   bool                 m_spi_reset_in_progress;

   public:

    // 'used' prevents removal by optimizer
    static volatile uint16_t  __attribute__(( aligned(32), section(".dma_tx_buffer2"), used)) m_spi_as5048_tx_buff[NUM_TX_READINGS] ;
    static volatile uint16_t  __attribute__(( aligned(32), section(".dma_rx_buffer2"), used)) m_spi_as5048_rx_buff[NUM_RX_READINGS] ;


};


#endif // Inclusion guard
