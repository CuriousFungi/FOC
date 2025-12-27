/*******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************/
#include "main.h"
#include "string.h"

#include <string>
//#include <vector>

#include "string.h"
#include "stdio.h"

#include "StepperMotor.hpp"
#include "time_utils.hpp"
#include "ElapsedTime.hpp"

#include "as5048a.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal_dac.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_adc.h"
#include "stm32h7xx_hal_adc_ex.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_pcd.h"
//#include "stm32h7xx_hal_eth.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_hsem.h"
#include "stm32h7xx_hal_flash_ex.h"

//extern uint32_t Get_Stack_Usage(void);

#ifdef __cplusplus
}
#endif

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif


extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern DAC_HandleTypeDef hdac1;

extern SPI_HandleTypeDef hspi4;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart3;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

volatile uint16_t adc_dma_result[ADC_BUFFER_SIZE];

// This variable calculate the array length.
// In our case, array size in 2
int adc_channel_count = sizeof(adc_dma_result)/sizeof(adc_dma_result[0]);

// This flag will help to detect
// the DMA conversion completed or not
volatile uint8_t adc_conv_complete_flag = 0;
volatile float winding_amperage_a(0.0f);
volatile float winding_amperage_b(0.0f);

volatile float g_adc_to_voltage_a_0_5(0.0f);
volatile float g_centered_voltage_a_absp925(0.0f);

volatile float g_adc_to_voltage_b_0_5 (0.0f);
volatile float g_centered_voltage_b_absp925(0.0f);



// Declare the function to get the buffer address from C++
extern uint32_t* get_timestamp_buffer_address(void);

static StepperMotor stepper = StepperMotor(
                                     &hspi4,        //  sensor spi
                                     50,             //  number of pole pairs (was 100, corrected to 50)
                                     212.0f,  //1.45f,         //  phase resistance
                                     1.0f,          // TODO: determine real  _KV,
                                     3.2f,    //4.0f,          // mH inductance
                                     20.0f,         // voltage limit
                                     20.0f,         // power supply voltage limit L298N
                                     &htim1,
                                     &htim8,
                                     TIM_CHANNEL_1,  // tmr 1
                                     TIM_CHANNEL_2,  // tmr 1
                                     TIM_CHANNEL_1,  // tmr 8
                                     TIM_CHANNEL_2); // tmr 8
void enableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT unit
  DWT->CYCCNT       = 0; // Reset cycle counter
  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk; // Enable cycle counter
}

bool is_foc_initialized = false;

struct winding_currents
{
  float winding_amperage_a;
  float winding_amperage_b;
};

winding_currents udpate_amperage(void)
{
    const float MAX_16BIT_ADC_COUNT(static_cast<float>(0xFFFF));
    const float V_REF(5.0f);
    //const float ZERO_CURRENT_VOLTAGE (2.5f);
    const float ZERO_CURRENT_VOLTAGE_A = 2.56f;   // Adjusted zero-current voltage for sensor A
    const float ZERO_CURRENT_VOLTAGE_B = 2.59f;   // Adjusted zero-current voltage for sensor B
    const float ACS712_05B_MILLIVOLTS_PER_AMP(0.185f);

    // 5A * 0.185V/A-->0.925V
    const float tweek(0.925f+.42f);

    // Converts ADC value to voltage (0V to 5V)
    float adc_to_voltage_a = (static_cast<float>(adc_dma_result[0]) / MAX_16BIT_ADC_COUNT) * V_REF;
    float adc_to_voltage_b = (static_cast<float>(adc_dma_result[1]) / MAX_16BIT_ADC_COUNT) * V_REF;

    // Center the voltage around 0A
    float centered_voltage_a = adc_to_voltage_a - ZERO_CURRENT_VOLTAGE_A - tweek;
    float centered_voltage_b = adc_to_voltage_b - ZERO_CURRENT_VOLTAGE_B - tweek;

    // Convert voltage to current in Amperes
    float winding_amps_a = centered_voltage_a / ACS712_05B_MILLIVOLTS_PER_AMP;
    float winding_amps_b = centered_voltage_b / ACS712_05B_MILLIVOLTS_PER_AMP;

    // Use clamping if needed to prevent unrealistic values
    winding_amps_a = fmaxf(fminf(winding_amps_a, 5.0f), -5.0f);
    winding_amps_b = fmaxf(fminf(winding_amps_b, 5.0f), -5.0f);


    g_adc_to_voltage_a_0_5       = adc_to_voltage_a;
    g_centered_voltage_a_absp925 =centered_voltage_a;

    g_adc_to_voltage_b_0_5        = adc_to_voltage_b;
    g_centered_voltage_b_absp925  = centered_voltage_b;

    return {
             .winding_amperage_a = winding_amps_a,
             .winding_amperage_b = winding_amps_b
           };

}
volatile float g_cmd_rps(0.0f);
static float rps = 0.01f;
void update_ramp(void)
{
     if (rps < 30.0f) // 50
     {
        rps += 0.1f;    //0.0002f;
        g_cmd_rps = rps;
        stepper.update_target_rad_per_sec(rps);
     }
}


extern "C"
void wrapper_control_loop_25us(void)
{
    if(is_foc_initialized)
    {
        stepper.control_loop_25us();
    }
}

#if 0
extern "C"
void wrapper_reinit_dma_for_spi(void)
{
    stepper.reinit_dma_for_spi();
}
#endif

extern "C"
int is_foc_initialization_complete()
{
   return is_foc_initialized ? 1 : 0;
}


//-----------------------------------------------------------------------------
//                          wrapper_sample_as5048_25us
//
// Caution: This is invoked from within an interrupt context
//-----------------------------------------------------------------------------
extern "C"
void wrapper_sample_as5048_25us(void)
{
    //if(is_foc_initialized)
    {
       stepper.sample_as5048_25us();
    }

}

extern "C"
void foc_iteration(void)
{
    if(is_foc_initialized)
    {
        winding_currents result = udpate_amperage();
        stepper.loopFOC(result.winding_amperage_a, result.winding_amperage_b);
        update_ramp();
    }
    else
    {
        rps = 0.0f;
    }
}

#if 0
void StepperMotor::process_encoder_data()
{
    static uint32_t last_timestamp = 0;

    if (read_ready)  // Ensure SPI read is complete
    {
        uint32_t current_timestamp = _micros();
        uint32_t delta_time_us = current_timestamp - last_timestamp;

        if (delta_time_us > MINIMUM_TIME_INTERVAL)  // Ensure enough time has passed
        {
            // Get the most recent angle
            uint16_t current_angle = angle_buffer[current_index];

            // Calculate velocity using delta angle and delta time
            float delta_angle = calculate_delta_angle(last_angle, current_angle);
            float delta_time_s = static_cast<float>(delta_time_us) * 0.000001f;

            // Compute velocity
            float velocity = delta_angle / delta_time_s;

            // Apply optional filtering
            filtered_velocity = m_LPF_velocity(velocity);

            // Update PID control or other feedback mechanism
            float velocity_error = target_rad_per_sec - filtered_velocity;
            float velocity_correction = m_PID_velocity.update(velocity_error);

            // Update control outputs or system state
            set_motor_speed(target_rad_per_sec + velocity_correction);

            // Store the last angle and timestamp for the next loop
            last_angle = current_angle;
            last_timestamp = current_timestamp;
        }

        read_ready = false;  // Reset read flag
    }
}
#endif

extern "C"
void start_spi_conversion()
{
	stepper.start_spi_conversion();
}

extern uint8_t *aRxBuffer;

extern "C"
void complete_spi_conversion()
{
    stepper.set_async_read_complete();

#if 1
    const uint16_t ERROR_BIT(0x4000);
    // Ensure memory ordering with a Data Memory Barrier
    __DMB();


    // Ensure data synchronization with a Data Synchronization Barrier
    __DSB();
    SCB_InvalidateDCache_by_Addr ((uint32_t *)&AS5048A::m_spi_as5048_rx_buff, 2);

    volatile uint16_t received_value  = AS5048A::m_spi_as5048_rx_buff[0];


    // CRITICAL: Always update buffers to ensure g_as5048_angle and g_as5048_velocity are updated
    // Even if error bit is set, we still want to see the angle data
    stepper.update_buffers(received_value & ~0xC000, micros());

    if (ERROR_BIT == (ERROR_BIT & received_value))
    {
        // Trouble in paradise - but we still updated the buffers above
        __HAL_SPI_DISABLE(&hspi4);

        stepper.spi_reset_in_progress();
        //stepper.set_async_read_complete();
    }

    // Re-enable timer interrupt
    // TBV remove
    //__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
#endif
}

extern "C"
int async_read_complete()
{
  return stepper.async_read_complete() ? 1 : 0;
}


extern "C"
void set_async_read_complete()
{
   stepper.set_async_read_complete();
}

extern "C"
void cpp_main(void)
{
  enableCycleCounter();

  // Initialize the DMA conversion
 //volatile HAL_StatusTypeDef status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_result , adc_channel_count);

  is_foc_initialized = stepper.initFOC();

  uint32_t last_foc_time = micros();

  while (1)
  {
      uint32_t current_time = micros();
      uint32_t elapsed = current_time - last_foc_time;

      if(elapsed >= MICROSECONDS_PER_ITERATION)
      {
          foc_iteration();
          last_foc_time = current_time;
      }
  }
}
