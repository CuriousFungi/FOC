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
#include <vector>

#include "string.h"
#include "stdio.h"

#include "./motors/StepperMotor.hpp"

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

#ifdef __cplusplus
}
#endif

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif


extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern DAC_HandleTypeDef hdac1;

//extern ETH_HandleTypeDef heth;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart3;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;


#define NUM_ADC_CHANNELS 2
#define BUFFER_SIZE  32
volatile uint16_t adc_dma_result[BUFFER_SIZE];

// This variable calculate the array length.
// In our case, array size in 2
int adc_channel_count = sizeof(adc_dma_result)/sizeof(adc_dma_result[0]);

// This flag will help to detect
// the DMA conversion completed or not
volatile uint8_t adc_conv_complete_flag = 0;

// when DMA conversion is completed, HAL_ADC_ConvCpltCallback function
// will interrupt the processor. You can find this function in
// Drivers>STM32F4xx_HAL_Drivers>stm32f4xx_hal_adc.c file as __weak attribute
#if 0
extern "C"
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // I set adc_conv_complete_flag variable to 1 when,
    // HAL_ADC_ConvCpltCallback function is call.
    if (hadc->Instance == ADC1)
    {
      adc_conv_complete_flag = 1;
        //HAL_ADC_Stop_DMA(&hadc1);
    }
}
#endif


/* USER CODE BEGIN PV */


volatile float winding_amperage_a(0.0f);
volatile float winding_amperage_b(0.0f);

volatile float g_adc_to_voltage_a_0_5(0.0f);
volatile float g_centered_voltage_a_absp925(0.0f);

volatile float g_adc_to_voltage_b_0_5 (0.0f);
volatile float g_centered_voltage_b_absp925(0.0f);
volatile unsigned long g_us(0);


StepperMotor stepper = StepperMotor(
                                     &hspi2,        //  sensor spi
                                     SPI2_CS_GPIO_Port,         //  p_sensor_chip_select_port,
                                     SPI2_CS_Pin,   //  sensor_chip_select_pin,
                                     100,            //  number of pole pairs
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
    const float ZERO_CURRENT_VOLTAGE (2.5f);
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
static float rps = 0.0f;
void update_ramp(void)
{
     if (rps < 50.0f)
     {
        rps += 0.0002f;
        g_cmd_rps = rps;
        stepper.update_target_rad_per_sec(rps);
     }
}

extern "C"
void timestamp_angle_reading(void)
{
    stepper.timestamp_angle_reading();
}

extern "C"
void wrapper_control_loop_25us(void)
{
    
    if(is_foc_initialized)
    {
        stepper.control_loop_25us();
    }
}

extern "C"
int is_foc_initialization_complete()
{
   return is_foc_initialized ? 1 : 0;
}


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
    RefreshWatchdog();
    
    if(!is_foc_initialized)
    {
        rps = 0.0f;
    }
    else
    {  
        winding_currents result = udpate_amperage();
        stepper.loopFOC(result.winding_amperage_a, result.winding_amperage_b);
        update_ramp();

#if 0

        // Inside your main loop or another appropriate place
        if (__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_TEIF0_4))
        {
            // printf("DMA Transfer Error Detected.\n");
            __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TEIF0_4);
        }

        // Check for ADC overrun
        if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR))
        {
            __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
            // Log or handle ADC overrun
        }
        //--------------------------
        // Check DMA status
        volatile uint32_t dma_stream_flags = DMA1->LISR;  // Check interrupt status register for stream 0
        //printf("DMA LISR Flags: 0x%08X\n", dma_stream_flags);
        
        // Check if ADC is running
        if (HAL_IS_BIT_SET(hadc1.Instance->CR, ADC_CR_ADSTART))
        {
            //printf("ADC conversion is active.\n");
            volatile int dummy1 = 0;
        }
        else
        {
            //printf("ADC conversion is not active. Restart ADC-DMA.\n");
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_result, NUM_ADC_CHANNELS);
        }
        //---------------------------
        // Check the DMA stream control register (CR)
        volatile uint32_t dma_stream_cr = DMA1_Stream0->CR; // Check the configuration of the DMA stream
        //printf("DMA Stream Control Register (CR): 0x%08X\n", dma_stream_cr);

        // Check for DMA stream enable flag
        if (dma_stream_cr & DMA_SxCR_EN)
        {
            volatile int dummy2 = 0;
            //printf("DMA Stream is enabled.\n");
        }
        else
        {
            //printf("DMA Stream is not enabled. Verify initialization and configuration.\n");
        }

        // Check for errors in DMA low interrupt status register (LISR)
        volatile uint32_t dma_lisr = DMA1->LISR;
        //printf("DMA LISR (Low Interrupt Status Register): 0x%08X\n", dma_lisr);
        volatile int dummy3 = dma_lisr;

        // Re-enable DMA stream if not enabled
        if (!(DMA1_Stream0->CR & DMA_SxCR_EN))
        {
            DMA1_Stream0->CR |= DMA_SxCR_EN; // Enable the stream
            //printf("DMA Stream manually enabled.\n");
        }

        volatile uint32_t fifo_status = DMA1_Stream0->FCR;
        //printf("DMA FIFO Status: 0x%08X\n", fifo_status);
        volatile int dummyA = fifo_status;

        uint32_t nvic_iser = NVIC->ISER[0];  // Check NVIC set-enable register for correct IRQ
        //printf("NVIC ISER: 0x%08X\n", nvic_iser);
        volatile int dummyB = nvic_iser;
        //-------------------------    
  #endif
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
void complete_spi_conversion()
{
       	stepper.conversion_complete();
}

extern "C"
int async_read_complete()
{
  return stepper.async_read_complete() ? 1 : 0;
}


extern "C"
void cpp_main(void)
{
  enableCycleCounter();

  // Initialize the DMA conversion
 volatile HAL_StatusTypeDef status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_result , adc_channel_count);

//-------------------

  volatile uint32_t dma_cr = DMA1_Stream0->CR;
  if (!(dma_cr & DMA_SxCR_EN))
  {
      //printf("DMA Stream is not enabled when expected. Re-enabling.\n");
      DMA1_Stream0->CR |= DMA_SxCR_EN; // Manually enable if not set
  }



//--------------


  uint32_t count(0);
  char char_buffer[50];
  std::vector<float> speeds;
  for (float i = 0.0f; i <= 8.0f; i += 0.1f)
  {
     speeds.push_back(i);
  }
  int num_speeds = speeds.size();
  int speed_index = 0;
  bool success = stepper.initFOC();
  
  is_foc_initialized = success;


  unsigned long prev_us = _micros();
 // const uint32_t MICROSECONDS_PER_ITERATION(25);
  const uint32_t MICROSECONDS_PER_SECOND(1000000);
  const uint32_t ITERATIONS_PER_SECOND(MICROSECONDS_PER_SECOND/MICROSECONDS_PER_ITERATION);
  
  // https://github.com/RobTillaart/ACS712
  const float V_REF(5.0f);
  //const float AMPS_PER_VOLT(2.0f); // per IBT2
  const float AMPS_PER_VOLT(5.41f); // per ACS712
  const float FULL_SCALE_ADC(V_REF * AMPS_PER_VOLT);

  //const float LSB_VALUE(FULL_SCALE_ADC / static_cast<float>(0xFFF));

  const float LSB_VALUE(
                       ( 1000.0f * ((1000.0f*V_REF) / static_cast<float>(0xFFFF)))
                      / (185.0f));
  

  while (1)
  {
      static unsigned long prev_time = 0;
      unsigned long curr_time    = _micros();
      unsigned long elapsed_time = curr_time - prev_time;
      if(elapsed_time >= 500UL)  // MICROSECONDS_PER_ITERATION)
      {    
          foc_iteration();
          prev_time = curr_time;
          g_us = elapsed_time;

          if (stepper.async_read_complete())
          {
             stepper.process_encoder_data();
          }
      }
  }
}

