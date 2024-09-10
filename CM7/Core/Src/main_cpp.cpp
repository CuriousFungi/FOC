/* USER CODE BEGIN Header */
/**
  ******************************************************************************
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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

#if defined ( __GNUC__ ) /* GNU Compiler */

//extern ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
//extern ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

//extern ETH_TxPacketConfig TxConfig;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern DAC_HandleTypeDef hdac1;

//extern ETH_HandleTypeDef heth;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart3;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;


// TODO: restructure ADC interface
#define NUM_ADC_CHANNELS 2
volatile uint16_t adc_dma_result[NUM_ADC_CHANNELS];
// This variable calculate the array length.
// In our case, array size in 2
int adc_channel_count = sizeof(adc_dma_result)/sizeof(adc_dma_result[0]);
// This flag will help to detect
// the DMA conversion completed or not
volatile uint8_t adc_conv_complete_flag = 0;

// when DMA conversion is completed, HAL_ADC_ConvCpltCallback function
// will interrupt the processor. You can find this function in
// Drivers>STM32F4xx_HAL_Drivers>stm32f4xx_hal_adc.c file as __weak attribute
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	// I set adc_conv_complete_flag variable to 1 when,
	// HAL_ADC_ConvCpltCallback function is call.
	adc_conv_complete_flag = 1;
    HAL_ADC_Stop_DMA(&hadc1);
}


/* USER CODE BEGIN PV */

StepperMotor stepper = StepperMotor(
                                     &hspi2,        //  sensor spi
                                     SPI2_CS_GPIO_Port,         //  p_sensor_chip_select_port,
                                     SPI2_CS_Pin,   //  sensor_chip_select_pin,
                                     50,            //  number of pole pairs
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_ADC1_Init(void);
// static void MX_DAC1_Init(void);
// static void MX_ETH_Init(void);
// static void MX_SPI1_Init(void);
// static void MX_TIM1_Init(void);
// static void MX_TIM8_Init(void);
// static void MX_USART3_UART_Init(void);
// static void MX_USB_OTG_FS_PCD_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

#if 0
// TODO: restructure ADC interface
#define NUM_ADC_CHANNELS 4
volatile uint16_t adc_dma_result[NUM_ADC_CHANNELS];
// This variable calculate the array length.
// In our case, array size in 2
int adc_channel_count = sizeof(adc_dma_result)/sizeof(adc_dma_result[0]);
// This flag will help to detect
// the DMA conversion completed or not
volatile uint8_t adc_conv_complete_flag = 0;
// when DMA conversion is completed, HAL_ADC_ConvCpltCallback function
// will interrupt the processor. You can find this function in
// Drivers>STM32F4xx_HAL_Drivers>stm32f4xx_hal_adc.c file as __weak attribute
extern "C"
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	// I set adc_conv_complete_flag variable to 1 when,
	// HAL_ADC_ConvCpltCallback function is call.
	adc_conv_complete_flag = 1;
  HAL_ADC_Stop_DMA(&hadc1);
}
#endif

extern "C"
void cpp_main(void)
{



	// Initialize the DMA conversion
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_result , adc_channel_count);







 // int32_t timeout;

  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  // timeout = 0xFFFF;
  // while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  // if ( timeout < 0 )
  // {
  // Error_Handler();
  // }

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();

  /* USER CODE BEGIN Init */

  enableCycleCounter();
  /* USER CODE END Init */

  /* Configure the system clock */
 // SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
//__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
//HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
//HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
// timeout = 0xFFFF;
// while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
// if ( timeout < 0 )
// {
// Error_Handler();
// }
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  // MX_GPIO_Init();
  // MX_ADC1_Init();
  // MX_DAC1_Init();
  // MX_ETH_Init();
  // MX_SPI1_Init();
  // MX_TIM1_Init();
  // MX_TIM8_Init();
  // MX_USART3_UART_Init();
  // MX_USB_OTG_FS_PCD_Init();

#if 0
  __HAL_RCC_TIM8_CLK_ENABLE();
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_OC_Start(&htim8,  TIM_CHANNEL_6);
  __HAL_RCC_TIM1_CLK_ENABLE();
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
#endif
  
  // Initialize the DMA conversion
   HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_result , adc_channel_count);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //int as_count(0);
  uint32_t count(0);
  char char_buffer[50];
  std::vector<float> speeds;
  for (float i = 0.0f; i <= 60.0f; i += 1.0f)
  {
         speeds.push_back(i);
  }
  int num_speeds = speeds.size();
  int speed_index = 0;
  bool success = stepper.initFOC();
  if(success)
  {
   stepper.move(2.0f); // for closedloop velocity
  }
 // float target_angle_radians(0.0f);
  unsigned long prev_us = _micros();
  const uint32_t MICROSECONDS_PER_ITERATION(20);
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



const float ADC_OFFSET_VOLTAGE(0.0f);  //FULL_SCALE_ADC/2.0f);
  while (1)
  {
      unsigned long now_us = _micros();
  static bool init_failure_announced(false);
  if((now_us - prev_us) > MICROSECONDS_PER_ITERATION)
  {
      float winding_amperage_a(0.0f);
      float winding_amperage_b(0.0f);
      //static float prev_winding_amperage_a(0.0f);
      //static float prev_winding_amperage_b(0.0f);
      
      if(success)
      {
    	  if(adc_conv_complete_flag == 1)
    	  {
              winding_amperage_a = (LSB_VALUE * static_cast<float>(adc_dma_result[0]));
              winding_amperage_b = (LSB_VALUE * static_cast<float>(adc_dma_result[1]));
              
              adc_conv_complete_flag = 0;
              HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(adc_dma_result), NUM_ADC_CHANNELS);
    	  }

    	  uint32_t start = DWT->CYCCNT;  // Start timing

          stepper.loopFOC(winding_amperage_a, winding_amperage_b);

          uint32_t end = DWT->CYCCNT;    // End timing
          uint32_t cycles = end - start; // Calculate elapsed cycles

            // Optionally, you could log or store the 'cycles' variable for later analysis
            printf("Cycles: %lu\n", cycles);

      }
      else
      {
         if(!init_failure_announced)
         {
          sprintf(char_buffer, "\t\tstepper.initFOC(stepper.initFOC failed\r\n");
          HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(char_buffer), strlen(char_buffer), HAL_MAX_DELAY);
          init_failure_announced = true;
         }
      }
#if 1 // used for OL speed control tests
      if(++count > ITERATIONS_PER_SECOND/4)  // 1 sec when if((now_us - prev_us) > 1000) used
      {
          count = 0;
          if(speed_index < num_speeds)
          {
                char  dash[] = "-";
                HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(dash), strlen(dash), HAL_MAX_DELAY);
              stepper.move(speeds[speed_index++]);
          }
      }
#endif
#if 0
	  if(++count > ITERATIONS_PER_SECOND)  // 1 sec
	  {
	      count = 0;
          as_count++;
          target_angle_radians += (clockwise) ? 0.174533f: 0.174533f; // 10 degrees
          clockwise = !clockwise;
          if(success)
          {
          }
	      float radians = stepper.get_angle_radians();     
	      if(false) //(stepper.error_detected())
	      {
	    	  uint16_t error = stepper.get_errors();
	          sprintf(char_buffer, "\t\tAS5048 Error: %d\r\n", (int)error);
	          HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(char_buffer), strlen(char_buffer), HAL_MAX_DELAY);
			  stepper.clear_error();
	      }
	      else
	      {
	          float degrees = radians * 360.0f / 6.28318530718f;
	          sprintf(char_buffer, "%d\t\tdeg: %d\r\n", as_count, static_cast<int>(degrees*1000.0f));
	          HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(char_buffer), strlen(char_buffer), HAL_MAX_DELAY);
	      }
	  }
 #endif
      prev_us = now_us;
  }
  }
  /* USER CODE END 3 */
}

