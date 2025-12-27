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
#include "interrupt_priorities.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdbool.h>

//#define USE_HAL_ADC_REGISTER_CALLBACKS (1)

#include "stm32h7xx.h"

#include "stm32h7xx_hal.h" // Ensure correct HAL is included for your MCU
#include "stm32h7xx_hal_adc.h"



// Define ITM port 0 register address for printf redirection
#define ITM_STIMULUS_PORT0    (*((volatile unsigned int*)0xE0000000)) 
#define ITM_TRACE_EN          (*((volatile unsigned int*)0xE0000E00))

enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};

#define BUFFERSIZE 1
/* Buffer used for reception */
#define BUFFER_ALIGNED_SIZE (((BUFFERSIZE+31)/32)*32)
ALIGN_32BYTES(uint8_t aRxBuffer[BUFFER_ALIGNED_SIZE]);

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;

//#define USE_HAL_SPI_REGISTER_CALLBACKS = 1U;



extern volatile uint16_t* adc_dma_result;

volatile uint8_t tx_complete_flag = 0;
volatile uint8_t rx_complete_flag = 0;



//=========================


//#include "stm32h755xx.h"
#define TIM1_UP_TIM16_IRQn       ((IRQn_Type)25)  // Example IRQ number, actual value may vary

#define CORE_CLOCK_HZ 400000000U  // 400 MHz for STM32H755


uint16_t as5048_read_buffer;  // 16-bit buffer for storing the angle data
float filtered_velocity = 0.0f;  // Store filtered velocity value

extern void DWT_Init(void);
extern void start_spi_conversion();

HAL_StatusTypeDef Start_ADC_DMA(void);


#ifdef __cplusplus
extern "C" {
#endif

void cpp_main(void);
void foc_iteration(void);      
void wrapper_control_loop_25us(void);
void wrapper_sample_as5048_25us(void);
//void wrapper_reinit_dma_for_spi(void);


//void RefreshWatchdog(void);
void complete_spi_conversion(void);


#ifdef __cplusplus
}
#endif
       void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
       void MX_TIM1_Init(void);
       void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
//static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
//static void MX_WWDG1_Init(void);
static void MX_SPI4_Init(void);
       void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);


#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif


uint16_t dac_buffer[100] = {0};


ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1;

//SPI_HandleTypeDef hspi2;
//DMA_HandleTypeDef hdma_spi2_rx;
//DMA_HandleTypeDef hdma_spi2_tx;

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef  hpcd_USB_OTG_FS;


#if 0
WWDG_HandleTypeDef hwwdg1;


#undef ENABLE_WATCHDOG
//#define ENABLE_WATCHDOG
#ifdef ENABLE_WATCHDOG

 //-----------------------------------------------------------------------------
 //                          MX_WWDG1_Init
 //-----------------------------------------------------------------------------
static void MX_WWDG1_Init(void)
{

  /* USER CODE BEGIN WWDG1_Init 0 */

  /* USER CODE END WWDG1_Init 0 */

  /* USER CODE BEGIN WWDG1_Init 1 */

  /* USER CODE END WWDG1_Init 1 */
  hwwdg1.Instance = WWDG1;
  hwwdg1.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg1.Init.Window = 0x10;
  hwwdg1.Init.Counter = 0x7F;
  hwwdg1.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG1_Init 2 */

  /* USER CODE END WWDG1_Init 2 */

}

//-----------------------------------------------------------------------------
//                          RefreshWatchdog
//-----------------------------------------------------------------------------
void RefreshWatchdog(void)
/**
  * Enable DMA controller clock
  */
{
    HAL_WWDG_Refresh(&hwwdg1);
}
#else

//-----------------------------------------------------------------------------
//                          DisableWatchdogs
//-----------------------------------------------------------------------------
void DisableWatchdogs(void) 
{
   // Unlock access to IWDG_PR and IWDG_RLR registers
   IWDG1->KR = 0x00005555;
   IWDG2->KR = 0x00005555;
   
   // DMA controller clock enable
    __HAL_RCC_WWDG_CLK_DISABLE();

    
  // Set IWDG reload register to minimum to effectively disable it
  IWDG1->RLR = 0x0000;

  // Set prescaler to the highest value to reduce timeout frequency
  //IWDG1->PR = IWDG_PRESCALER_256;

#if 0
   IWDG1->KR = 0x0000;  // Prevents the IWDG from starting

    if ((WWDG1->CR & WWDG_CR_WDGA) != 0) {
        __HAL_RCC_WWDG_CLK_DISABLE();  // Disable clock to WWDG as an extreme measure
    }
#endif
}

//-----------------------------------------------------------------------------
//                          MX_WWDG1_Init
//-----------------------------------------------------------------------------
static void MX_WWDG1_Init(void)
{
    DisableWatchdogs();
}

//-----------------------------------------------------------------------------
//                          RefreshWatchdog
//-----------------------------------------------------------------------------
void RefreshWatchdog(void)
{
    return;
}
#endif


void WWDG_RST_IRQHandler(void)
{
    RefreshWatchdog();

    __BKPT();
}
#endif

//-----------------------------------------------------------------------------
//                              MPU_Config
//-----------------------------------------------------------------------------
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;
    const uint32_t RAM_D1_BASE = 0x24000000U;
    const uint32_t RAM_D2_BASE = 0x30000000U;
    //const uint32_t RAM_D3_BASE = 0x38000000U;
    const uint32_t OFFSET_256K = 0x00040000U;
    //const uint32_t OFFSET_128K = 0x00020000U;

    /* Disable the MPU */
    HAL_MPU_Disable();

    // Region 0
    // Configure Flash memory region (read-only, cacheable)
    MPU_InitStruct.Number = MPU_REGION_NUMBER0; // Start from lowest
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x08000000; // Start of Flash memory
    MPU_InitStruct.Size = MPU_REGION_SIZE_1MB; // Flash memory size
    MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO_URO; // Read-only
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE; // Enable caching
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);


    // Region 1  D1 lower 256
    // Configure remaining RAM_D1 region (read/write, cacheable)
    MPU_InitStruct.Number = MPU_REGION_NUMBER1; 
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = RAM_D1_BASE;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;  //256KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE; // Cacheable for general use
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE; // Private access
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

#if 0
    // Region 2  D1 Middle 128
    // Configure remaining RAM_D1 region (read/write, cacheable)
    MPU_InitStruct.Number = MPU_REGION_NUMBER2; 
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = RAM_D1_BASE + OFFSET_256K;
    MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE; // Cacheable for general use
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE; // Private access
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    // Region 3  D1 upper 128
    // Configure RAM_D2 for DMA operations
    MPU_InitStruct.Number = MPU_REGION_NUMBER3; // Sequential number
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = RAM_D1_BASE + OFFSET_256K + OFFSET_128K;
    MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE; // Strongly ordered for DMA
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE; // DMA-compatible
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    //MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
#endif
    // Region 2  DTCMRAM
    // Configure DTCMRAM region (read/write, non-cacheable) 
    MPU_InitStruct.Number = MPU_REGION_NUMBER2;
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x20000000; // Start of DTCMRAM
    MPU_InitStruct.Size = MPU_REGION_SIZE_128KB; // Size of DTCMRAM
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE; // Bufferable
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE; // Non-cacheable
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE; // DMA-compatible
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);


    // Region 3  D2 lower 256K
    // Configure remaining RAM_D1 region (read/write, cacheable)
    MPU_InitStruct.Number = MPU_REGION_NUMBER3; 
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = RAM_D2_BASE;
    MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    // Region 4  D2 upper 32K for DMA operations
    // Configure remaining RAM_D2 region
    MPU_InitStruct.Number = MPU_REGION_NUMBER4; 
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = RAM_D2_BASE + OFFSET_256K;
    MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE; // Cacheable for general use
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE; // Private access
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;    
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    // Region 5 RAM_D3
    // Configure RAM_D3 region (read/write, cacheable)
    MPU_InitStruct.Number = MPU_REGION_NUMBER5;
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x38000000; // Start of RAM_D3
    MPU_InitStruct.Size = MPU_REGION_SIZE_64KB; // Size of RAM_D3
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE; // Cacheable
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE; // Private access
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;    
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    // Region 6  ITCMRAM 
    // Configure ITCMRAM region (read/write, non-cacheable)
    MPU_InitStruct.Number = MPU_REGION_NUMBER6; // Next sequential number
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x00000000; // Start of ITCMRAM
    MPU_InitStruct.Size = MPU_REGION_SIZE_64KB; // Size of ITCMRAM
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE; // Non-cacheable
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE; // Private access
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;    
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Enable the MPU with default settings */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT); //MPU_PRIVILEGED_DEFAULT); MPU_HFNMI_PRIVDEF
}

//-----------------------------------------------------------------------------
//                              MPU_Config
//-----------------------------------------------------------------------------
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void Fill_DAC_Buffer(uint16_t * pBuff, uint32_t num_items)
{
	for(int i = 0; i < num_items; i++)
		pBuff[i] = 0;
}

//-----------------------------------------------------------------------------
//                       HAL_TIM_PWM_PulseFinishedCallback
//
// If the duty cycle is 0, HAL_TIM_PWM_PulseFinishedCallback might not be 
// invoked.Hybrid approach is to use both
//-----------------------------------------------------------------------------
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1 ) //|| htim->Instance == TIM8)
    {
        wrapper_sample_as5048_25us();
    }
}

//-----------------------------------------------------------------------------
//                    HAL_TIM_PeriodElapsedCallback
//-----------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    
    if(htim->Instance == TIM1)
    {
       // wrapper_sample_as5048_25us();
    }
    else if (htim->Instance == TIM8) 
    {
       //wrapper_sample_as5048_25us();
       wrapper_control_loop_25us();
    }
}

//-----------------------------------------------------------------------------
//                       HAL_SPI_TxRxCpltCallback
//
// SPI_DMATransmitReceiveCplt--> HAL_SPI_TxRxCpltCallback
//-----------------------------------------------------------------------------

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi4)
    {
    	complete_spi_conversion();
        hspi4.State = HAL_SPI_STATE_READY;
    }
}

//-----------------------------------------------------------------------------
//                        HAL_SPI_TxCpltCallback
//-----------------------------------------------------------------------------
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI4) 
    {
        // Clear the HAL busy state for TX
        hspi->State = HAL_SPI_STATE_READY;

        // Clear specific SPI flags if needed
        __HAL_SPI_CLEAR_OVRFLAG(hspi);  // Clear overrun
        __HAL_SPI_CLEAR_FREFLAG(hspi);  // Clear framing errors if any
        // Custom logic, e.g., setting a flag to indicate transmission complete
        tx_complete_flag = 1;
    }
}

//-----------------------------------------------------------------------------
//                     HAL_SPI_RxCpltCallback
//-----------------------------------------------------------------------------
extern void set_async_read_complete();
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI4) 
    { 
        // Clear the HAL busy state for RX
        hspi->State = HAL_SPI_STATE_READY;

        // Clear specific SPI flags if needed
        __HAL_SPI_CLEAR_OVRFLAG(hspi);  // Clear overrun
        __HAL_SPI_CLEAR_FREFLAG(hspi);  // Clear framing errors if any
        // Custom logic, e.g., setting a flag to indicate reception complete
        rx_complete_flag = 1;
        set_async_read_complete();
    }
}

//-----------------------------------------------------------------------------
//                          HAL_SPI_ErrorCallback
//-----------------------------------------------------------------------------
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) 
{
    if (hspi == &hspi4)
    {
        //static uint32_t last_overrun_data   = 0;
        //static uint32_t last_overrun_status = 0;

        uint32_t sr_status       = hspi->Instance->SR;        
        //uint32_t dma_lisr_status = DMA1->LISR;

           // Check for EOT (Bit 15)
           if (sr_status & SPI_SR_EOT) {
               // Clear EOT flag
               __HAL_SPI_CLEAR_EOTFLAG(hspi);
           }

           // Check for DXP (Bit 13)
           if (sr_status & SPI_SR_DXP) {
               // Data exchange was in progress
           }

           // Check for RXP (Bit 1)
           if (sr_status & SPI_SR_RXP) {
               // Data is available in the RX buffer
           }

           // Check for TXP (Bit 0)
           if (sr_status & SPI_SR_TXP) {
               // TX buffer is empty
           }
#if 0
           // Check for DMA errors for SPI2 RX (DMA1 Stream 1)
             // 1. Transfer Error
             if (__HAL_DMA_GET_FLAG(hspi->hdmarx, DMA_FLAG_TEIF1_5)) {
                 // Handle transfer error
                 __HAL_DMA_CLEAR_FLAG(hspi->hdmarx, DMA_FLAG_TEIF1_5);  // Clear the flag
             }

             // 2. Direct Mode Error
             if (__HAL_DMA_GET_FLAG(hspi->hdmarx, DMA_FLAG_DMEIF1_5)) {
                 // Handle direct mode error
                 __HAL_DMA_CLEAR_FLAG(hspi->hdmarx, DMA_FLAG_DMEIF1_5);  // Clear the flag
             }

             // 3. FIFO Error
             if (__HAL_DMA_GET_FLAG(hspi->hdmarx, DMA_FLAG_FEIF1_5)) {
                 // Handle FIFO error
                 __HAL_DMA_CLEAR_FLAG(hspi->hdmarx, DMA_FLAG_FEIF1_5);  // Clear the flag
             }

             // Check for DMA errors for SPI2 TX (DMA1 Stream 2)
             // 1. Transfer Error
             if (__HAL_DMA_GET_FLAG(hspi->hdmatx, DMA_FLAG_TEIF2_6)) {
                 // Handle transfer error
                 __HAL_DMA_CLEAR_FLAG(hspi->hdmatx, DMA_FLAG_TEIF2_6);  // Clear the flag
             }

             // 2. Direct Mode Error
             if (__HAL_DMA_GET_FLAG(hspi->hdmatx, DMA_FLAG_DMEIF2_6)) {
                 // Handle direct mode error
                 __HAL_DMA_CLEAR_FLAG(hspi->hdmatx, DMA_FLAG_DMEIF2_6);  // Clear the flag
             }

             // 3. FIFO Error
             if (__HAL_DMA_GET_FLAG(hspi->hdmatx, DMA_FLAG_FEIF2_6)) {
                 // Handle FIFO error
                 __HAL_DMA_CLEAR_FLAG(hspi->hdmatx, DMA_FLAG_FEIF2_6);  // Clear the flag
             }

#endif

        // Stop ongoing DMA transfers
        HAL_DMA_Abort(hspi->hdmarx);
        HAL_DMA_Abort(hspi->hdmatx);

        // Reset the SPI peripheral
        __HAL_SPI_DISABLE(hspi);
        __HAL_SPI_ENABLE(hspi);


        // Reinitialize SPI DMA transfer for AS5048A read
   //     start_spi_conversion();

        // Set error and communication flags
        //spi_error_detected      = true;


        // Optional: Restart timer triggering the AS5048A read
       // __HAL_TIM_SET_COUNTER(&htim1, 0);
    }
}

//-----------------------------------------------------------------------------
//                        HAL_DMA_ErrorCallback
//-----------------------------------------------------------------------------
void HAL_DMA_ErrorCallback(DMA_HandleTypeDef *hdma)
{
    if(__HAL_DMA_GET_FLAG(hdma, DMA_FLAG_TEIF1_5))
    {
        // Handle Transfer Error
        // Reset buffer or notify error
        __BKPT(0);
    }

    if(__HAL_DMA_GET_FLAG(hdma, DMA_FLAG_FEIF1_5))
    {
        // Handle FIFO Error
        __BKPT(0);
    }

    if(__HAL_DMA_GET_FLAG(hdma, DMA_FLAG_DMEIF1_5))
    {
        // Handle Direct Mode Error
        __BKPT(0);
    }    
}




//-----------------------------------------------------------------------------
//                             enable_swo
//-----------------------------------------------------------------------------
void enable_swo(void) 
{
    // Enable trace and debug blocks in CoreDebug
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Enable ITM and unlock access
    ITM->LAR = 0xC5ACCE55; // Unlock access to ITM registers
    ITM->TCR = 0x0001000D; // Enable ITM with SWO enabled and use TPIU

    // Enable ITM Port 0 for output
    ITM->TER = 0x1;

    // Configure SWO pin (usually PB3 or PB10 depending on the MCU)
    // Example configuration for PB3 as SWO:
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SWJ; // Set to the correct alternate function for SWO
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}




//-----------------------------------------------------------------------------
//                            Start_ADC_DMA
//-----------------------------------------------------------------------------
HAL_StatusTypeDef Start_ADC_DMA(void) 
{
    return HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_result, ADC_BUFFER_SIZE);
}

//-----------------------------------------------------------------------------
//                            Start_ADC_DMA
//-----------------------------------------------------------------------------
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        // Wait until ITM is enabled
        if ((ITM_TRACE_EN & 1) == 0) {
            return 0; // ITM is not enabled
        }
        // Write to ITM Port0 (for SWV)
        ITM_STIMULUS_PORT0 = ptr[i];
    }
    return len;
}

//-----------------------------------------------------------------------------
//                            ITM_Init
//-----------------------------------------------------------------------------
void ITM_Init(void) 
{
    // Preserve other bits in DEMCR and enable only the TRCENA bit
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Set TRCENA, preserving other bits

    // Unlock the ITM
    ITM->LAR = 0xC5ACCE55;

    // Enable ITM with SWOENA and Trace BusID (TCR settings)
    ITM->TCR = 0x0001000D;  // ITMENA + SWOENA

    // Enable stimulus port 0 (used for printf via SWO)
    ITM->TER = 0x00000004;
}



//-----------------------------------------------------------------------------
//                          SystemClock_Config
//-----------------------------------------------------------------------------
void SystemClock_Config(void)
{
  //extern uint32_t SystemCoreClock;
  //SystemCoreClock = 400000000;  // 400 MHz for STM32H7 let HAL_RCC_ClockConfig  set it
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 13;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /** Enable Clock Failure Detection (CFD) for HSE */
    //RCC->CR |= RCC_CR_CFDEN;

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  // FLASH_LATENCY_2
  HAL_StatusTypeDef status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7); 
  if (status != HAL_OK)
  {
    Error_Handler();
  }
  SystemCoreClockUpdate(); 
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);
}

//-----------------------------------------------------------------------------
//                          MX_GPIO_Init
//-----------------------------------------------------------------------------
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};


  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();


 // HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  //GPIO_InitStruct.Pin = SPI2_CS_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}


//-----------------------------------------------------------------------------
//                          MX_ADC1_Init
//-----------------------------------------------------------------------------
static void MX_ADC1_Init(void)
{
    // Changes
    // disabled hadc1.Init.ScanConvMode
    // Enabled continous conversion
    // changed hadc1.Init.EOCSelection from ADC_EOC_SEQ_CONV to ADC_EOC_SINGLE_CONV
    //changed hadc1.Init.ConversionDataManagement from ADC_CONVERSIONDATA_DR to ADC_CONVERSIONDATA_DMA_CIRCULAR
    // removed assignments to hadc1.Init.Oversampling fields
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;     // ADC asynchronous clock
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;          // 16-bit resolution
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;          // Disable scan mode for single channel prev ADC_SCAN_ENABLE
    hadc1.Init.ContinuousConvMode = ENABLE;        // Single conversion mode
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;    // Start conversion with software
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;    // End of conversion after each conversion
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;         // Preserve data on overrun
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;     
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler(); // Initialization Error
    }
    sConfig.Channel = ADC_CHANNEL_3;                   // Select channel 3, adjust if necessary
    sConfig.Rank = ADC_REGULAR_RANK_1;                 // Rank 1 in the regular group
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;    // Shortest sampling time
    sConfig.SingleDiff = ADC_SINGLE_ENDED;             // Single-ended input
    sConfig.OffsetNumber = ADC_OFFSET_NONE;            // No offset
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler(); // Channel configuration Error
    }
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
    HAL_NVIC_SetPriority(ADC_IRQn, IRQ_PRIORITY_ADC, 0);  // Set the interrupt priority
    HAL_NVIC_EnableIRQ(ADC_IRQn);          // Enable the ADC interrupt
}



//-----------------------------------------------------------------------------
//                          MX_DAC1_Init
//-----------------------------------------------------------------------------
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_ENABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  sConfig.DAC_SampleAndHoldConfig.DAC_SampleTime = 0;
  sConfig.DAC_SampleAndHoldConfig.DAC_HoldTime = 0;
  sConfig.DAC_SampleAndHoldConfig.DAC_RefreshTime = 0;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

  /* USER CODE END DAC1_Init 2 */

}

//-----------------------------------------------------------------------------
//                             MX_DMA_Init
//-----------------------------------------------------------------------------
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 
                       IRQ_PRIORITY_ADC, 0);
  HAL_NVIC_EnableIRQ(  DMA1_Stream0_IRQn);
  
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 
                       IRQ_PRIORITY_DMA_SPI_RX, 0);
  HAL_NVIC_EnableIRQ(  DMA1_Stream3_IRQn);
   
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 
                       IRQ_PRIORITY_DMA_SPI_TX, 0);
  HAL_NVIC_EnableIRQ(  DMA1_Stream4_IRQn);


  __HAL_DMA_STREAM_ENABLE_IT(&hdma_spi4_rx, DMA_IT_TE | DMA_IT_FE | DMA_IT_DME | DMA_IT_TC);
  __HAL_DMA_STREAM_ENABLE_IT(&hdma_spi4_tx, DMA_IT_TE | DMA_IT_FE | DMA_IT_DME | DMA_IT_TC);

 // HAL_NVIC_SetPriority(SPI4_IRQn, IRQ_PRIORITY_SPI, 0);
 // HAL_NVIC_EnableIRQ(SPI4_IRQn);
  
}
#if 0
//-----------------------------------------------------------------------------
//                          MX_SPI2_Init
//-----------------------------------------------------------------------------
static void MX_SPI2_Init(void)
{
  //HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

  hspi2.Instance->CR2 = (1 << SPI_CR2_TSIZE_Pos);
  SET_BIT(hspi2.Instance->CFG1, SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);

  __HAL_SPI_ENABLE_IT(&hspi2,  SPI_IT_ERR);

}
#endif

//-----------------------------------------------------------------------------
//                          MX_SPI4_Init
//-----------------------------------------------------------------------------
static void MX_SPI4_Init(void)
{
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // 100 MHz / 16 = 6.25 MHz (under 10 MHz AS5048A max)
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;

#if 0
  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);         /*!< function pointer on Rx ISR               */
  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);         /*!< function pointer on Tx ISR               */
  void (* TxCpltCallback)(struct __SPI_HandleTypeDef *hspi);       /*!< SPI Tx Completed callback          */
  void (* RxCpltCallback)(struct __SPI_HandleTypeDef *hspi);       /*!< SPI Rx Completed callback          */
  void (* TxRxCpltCallback)(struct __SPI_HandleTypeDef *hspi);     /*!< SPI TxRx Completed callback        */
  void (* TxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);   /*!< SPI Tx Half Completed callback     */
  void (* RxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);   /*!< SPI Rx Half Completed callback     */
  void (* TxRxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi); /*!< SPI TxRx Half Completed callback   */
  void (* ErrorCallback)(struct __SPI_HandleTypeDef *hspi);        /*!< SPI Error callback                 */
  void (* AbortCpltCallback)(struct __SPI_HandleTypeDef *hspi);    /*!< SPI Abort callback                 */
  void (* SuspendCallback)(struct __SPI_HandleTypeDef *hspi);      /*!< SPI Suspend callback               */
  void (* MspInitCallback)(struct __SPI_HandleTypeDef *hspi);      /*!< SPI Msp Init callback              */
  void (* MspDeInitCallback)(struct __SPI_HandleTypeDef *hspi);    /*!< SPI Msp DeInit callback            */
#endif


   hspi4.ErrorCallback = HAL_SPI_ErrorCallback;


  
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }

  hspi4.Instance->CR2 = (1 << SPI_CR2_TSIZE_Pos);
  SET_BIT(hspi4.Instance->CFG1, SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
  
  __HAL_SPI_ENABLE_IT(&hspi4,  SPI_IT_ERR);
}

 //-----------------------------------------------------------------------------
 //                          MX_TIM1_Init
 //-----------------------------------------------------------------------------
 void MX_TIM1_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 200; // 200 ticks0;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter      = 0;
  sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter     = 0;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  
  // Force the output to be enabled if using complementary outputs or if the outputs were not properly enabled
  __HAL_TIM_MOE_ENABLE(&htim1);  // Force the main output enable for TIM1

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

//-----------------------------------------------------------------------------
//                          MX_TIM2_Init
//-----------------------------------------------------------------------------
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 200-1; //400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 35-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  // Start Timer 2 base and channel 1 output compare
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
      Error_Handler();  // Check if the timer starts without errors
  }

   if (HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
   {
       Error_Handler();  // Check if Output Compare starts correctly
   }

  Fill_DAC_Buffer(dac_buffer, sizeof(dac_buffer)/sizeof(dac_buffer[0]));  // Define this function as needed
  

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

//-----------------------------------------------------------------------------
//                          MX_TIM8_Init
//-----------------------------------------------------------------------------
 void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();


  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  // Removed HAL_TIM_OC_Init() - was conflicting with PWM mode
  // if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  sMasterConfig.MasterOutputTrigger  = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 200; // 200 ticks0;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter      = 0;
  sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter     = 0;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */




// new
  // Manually enable interrupt after setting up the timer
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);  // Enable update interrupt for TIM8



  // Start the timer in interrupt mode after initialization
 // if (HAL_TIM_Base_Start_IT(&htim8) != HAL_OK)
  //{
  //    Error_Handler();
 // }


  
  // Force the output to be enabled if using complementary outputs or if the outputs were not properly enabled
  __HAL_TIM_MOE_ENABLE(&htim8);  // Force the main output enable for TIM8


  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

//-----------------------------------------------------------------------------
//                          MX_USART2_UART_Init
//-----------------------------------------------------------------------------
static void MX_USART2_UART_Init(void)
{
 // RefreshWatchdog();

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

//-----------------------------------------------------------------------------
//                          MX_USART3_UART_Init
//-----------------------------------------------------------------------------
static void MX_USART3_UART_Init(void)
{
 // RefreshWatchdog();

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

//-----------------------------------------------------------------------------
//                          MX_USB_OTG_FS_PCD_Init
//-----------------------------------------------------------------------------
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}




//-----------------------------------------------------------------------------
//                          Error_Handler
//-----------------------------------------------------------------------------
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  
  __BKPT(0); // Trigger breakpoint for debugging
  
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


//-----------------------------------------------------------------------------
//                          Error_Handler
//
//  @brief  Reports the name of the source file and the source line number
//        where the assert_param error has occurred.
// @param  file: pointer to the source file name
// @param  line: assert_param error line source number
// @retval None
//
//-----------------------------------------------------------------------------
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
   //  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif // USE_FULL_ASSERT 



extern uint32_t _stack_start;   // Start of stack (defined in linker script)
extern uint32_t _end;  // End of stack (defined in linker script)

#if 0
void Stack_Init(void) {
    uint32_t *ptr = &_stack_start;
    while (ptr < &_end) {
        *ptr++ = 0xDEADBEEF;
    }
}
#endif

void Stack_Init(void) {
    uint32_t *ptr = &_end;
    while (ptr < (uint32_t *)0x24080000)
    {
        *ptr++ =0xEFBEADDE; // DEADBEEF
    }
}

#if 0
uint32_t Get_Stack_Usage(void) {
    uint32_t *ptr = &_stack_start;
    uint32_t unused = 0;

    while (*ptr == 0xDEADBEEF && ptr < &_end) {
        ptr++;
        unused++;
    }

    // Calculate used stack in bytes
    return ((&_end - &_stack_start) - unused) * sizeof(uint32_t);
}
#endif


#if 0
void Disable_TIM4_Interrupt(void)
{
    // Check if the TIM4 interrupt is enabled in NVIC, and disable it if it is
     //if (NVIC->ISER[TIM4_IRQn / 32] & (1 << (TIM4_IRQn % 32)))
     {
         // Disable TIM4 interrupt in the NVIC
         HAL_NVIC_DisableIRQ(TIM4_IRQn);
     }
    
     // Disable TIM4 clock if it's not used elsewhere
     //if (__HAL_RCC_TIM4_IS_CLK_ENABLED()) 
     {
         __HAL_RCC_TIM4_CLK_DISABLE();
     }

}
#endif

//-----------------------------------------------------------------------------
//                              main
//-----------------------------------------------------------------------------
int main(void)
{
    Stack_Init();


    HAL_StatusTypeDef status = HAL_Init();
    if(status != HAL_OK)
    {
        __BKPT();
    }
    
    SystemClock_Config();


    
    CPU_CACHE_Enable();

    

    MPU_Config();


    
	//__HAL_RCC_WWDG_CLK_DISABLE();
    // HAL_NVIC_DisableIRQ(WWDG_IRQn);


    // Ensure WWDG is set to the maximum possible counter and disable EWI
    //WWDG1->CFR &= ~WWDG_CFR_EWI;       // Disable Early Wakeup Interrupt
    //WWDG1->CR = WWDG_CR_T | 0x7F;      // Set the counter to max value






	ITM_Init();


  uint32_t timeout = 0xFFFF;

#if 0
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif

  // Call DWT_Init after system clock is set
  DWT_Init();



  //==============================================
  // Enable Cortex-M4 Boot
  //__HAL_RCC_D2CKEN_CLK_ENABLE();

  // Boot CPU2 (Cortex-M4) and set it to STOP mode
  HAL_RCCEx_EnableBootCore(RCC_BOOT_C2);

  // Wait for the CPU2 (Cortex-M4) to enter stop mode
   timeout = 0xFFFF;
   while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));

   if (timeout < 0)
   {
       Error_Handler();
   }
//===================================================

/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();

  //HAL_StatusTypeDef status;
  status = HAL_ADC_RegisterCallback( &hadc1,
		                             HAL_ADC_CONVERSION_COMPLETE_CB_ID,
									 HAL_ADC_ConvCpltCallback);
  if (status != HAL_OK)
  {
      Error_Handler();
  }

  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  //MX_WWDG1_Init();

  // Enable the cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT
  DWT->CYCCNT = 0;                                // Reset the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable the cycle counter

 // MX_SPI2_Init();
 // HAL_SPI_MspInit(&hspi2);

  MX_SPI4_Init();
  HAL_SPI_MspInit(&hspi4);

#if 0
  
  __attribute__((aligned(4))) uint16_t CLEAR = 0x8001;
  __attribute__((aligned(4))) volatile uint16_t result= 0xDEAD;
  if(HAL_OK !=  HAL_SPI_TransmitReceive(&hspi2,
                            (uint8_t*)(&CLEAR),
                            (uint8_t*)(&result),
                            1, HAL_MAX_DELAY))
  {
    result *= 1;
  }

  #endif

  MX_TIM1_Init();
  MX_TIM8_Init();
  TIM1->DIER |= TIM_DIER_UIE; 
  
 
#if 0
  if (hspi2.State == HAL_SPI_STATE_READY || hspi2.State == HAL_SPI_STATE_RESET) 
  {
      status = HAL_SPI_RegisterCallback(&hspi2, HAL_SPI_TX_RX_COMPLETE_CB_ID, SPI_TxRx_completion_callback);
      if(HAL_OK != status) { Error_Handler(); }
  }
#endif

  status = HAL_TIM_Base_Start_IT(&htim1);
  if(HAL_OK != status) { Error_Handler(); }

  status = HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  if(HAL_OK != status) { Error_Handler(); }

  status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  if(HAL_OK != status) { Error_Handler(); }

  // Introduce a 90-degree phase shift for TIM1
  TIM8->CNT = TIM8->ARR / 4;


  status =  HAL_TIM_Base_Start_IT(&htim8) ;
  if (status != HAL_OK) { Error_Handler();}

  status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  if(HAL_OK != status) { Error_Handler(); }

  // PC7 outputs TIM8_CH2 (regular output), not CH2N
  status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  if(HAL_OK != status) { Error_Handler(); }

 // After starting TIM8 CH2 with PWMN_Start, check CCER again:
  uint32_t ccer_after = TIM8->CCER;
  volatile bool cc2e_after = (ccer_after & TIM_CCER_CC2E) != 0;
  volatile bool cc2ne_after = (ccer_after & TIM_CCER_CC2NE) != 0;
  
  // Check CCR2 register values (compare registers) - these determine duty cycle
  volatile uint32_t ccr2_tim8 = TIM8->CCR2;
  volatile uint32_t ccr2_tim1 = TIM1->CCR2;
  
  // Check ARR (auto-reload) values
  volatile uint32_t arr_tim8 = TIM8->ARR;
  volatile uint32_t arr_tim1 = TIM1->ARR;


  //status = HAL_SPI_RegisterCallback(&hspi2, HAL_SPI_RX_COMPLETE_CB_ID, HAL_SPI_RxCpltCallback);
  //if(HAL_OK != status) { Error_Handler(); }


  TIM1->DIER |= TIM_DIER_UIE;  // Enable update interrupt


  // After starting TIM8 CH2, check:
uint32_t ccer = TIM8->CCER;
volatile bool cc2e_enabled = (ccer & TIM_CCER_CC2E) != 0;   // Should be 1
volatile bool cc2ne_enabled = (ccer & TIM_CCER_CC2NE) != 0; // Should be 0

// Compare with TIM1:
uint32_t ccer1 = TIM1->CCER;
volatile bool tim1_cc2e = (ccer1 & TIM_CCER_CC2E) != 0;
volatile bool tim1_cc2ne = (ccer1 & TIM_CCER_CC2NE) != 0;


// Check the polarity bits - this is the key!
volatile bool cc2p_tim8 = (ccer & TIM_CCER_CC2P) != 0;   // TIM8 CH2 polarity
volatile bool cc2p_tim1 = (ccer1 & TIM_CCER_CC2P) != 0;  // TIM1 CH2 polarity

// Also check the full CCER values for comparison
volatile uint32_t ccer_full_tim8 = ccer;
volatile uint32_t ccer_full_tim1 = ccer1;


// Check CCMR1 register - OC2M mode should be the same
volatile uint32_t ccmr1_tim8 = TIM8->CCMR1;
volatile uint32_t ccmr1_tim1 = TIM1->CCMR1;

// Extract OC2M bits (bits 12-14 in CCMR1 for channel 2)
volatile uint32_t oc2m_tim8 = (ccmr1_tim8 & TIM_CCMR1_OC2M) >> 8U;
volatile uint32_t oc2m_tim1 = (ccmr1_tim1 & TIM_CCMR1_OC2M) >> 8U;

// Check BDTR register (dead time configuration)
volatile uint32_t bdtr_tim8 = TIM8->BDTR;
volatile uint32_t bdtr_tim1 = TIM1->BDTR;

// Check CR2 register (output idle states)
volatile uint32_t cr2_tim8 = TIM8->CR2;
volatile uint32_t cr2_tim1 = TIM1->CR2;

// Check if there's a difference in OIS2 (Output Idle State for CH2)
volatile bool ois2_tim8 = (cr2_tim8 & TIM_CR2_OIS2) != 0;
volatile bool ois2_tim1 = (cr2_tim1 & TIM_CR2_OIS2) != 0;


// Check GPIO alternate function for PC7
volatile uint32_t afrl_pc = GPIOC->AFR[0];  // AFR[0] covers pins 0-7
volatile uint32_t pc7_af = (afrl_pc >> (7 * 4)) & 0xF;  // Extract AF for PC7 (bits 28-31)

// PC7 should be AF3 (TIM8_CH2), verify it's not accidentally CH2N routing

if (cc2e_enabled || cc2ne_enabled || tim1_cc2e || tim1_cc2ne || cc2p_tim8 || cc2p_tim1)
		{
			;
		}



  enable_swo();

  // Start DAC with DMA
 // if (Start_ADC_DMA() != HAL_OK) // Ensure Start_ADC_DMA() returns a status
 //  {
 //      Error_Handler();  // Handle any DMA errors specifically
 //  }



//=======================================================================
// spi DMA
  /* Enable D2 SRAM clocks */
   RCC->AHB2ENR |= (RCC_AHB2ENR_D2SRAM1EN | RCC_AHB2ENR_D2SRAM2EN | RCC_AHB2ENR_D2SRAM3EN);
   volatile uint32_t tmpreg = RCC->AHB2ENR;
   (void)tmpreg;  // Ensure clock enable

   /* Set read issuing capability for AXI SRAM if needed (rev Y) */
   if ((DBGMCU->IDCODE & 0xFFFF0000U) < 0x20000000U) {
       *((__IO uint32_t*)0x51008108) = 0x000000001U;
   }
  //=======================================================================

 // Disable_TIM4_Interrupt();  this was because it lloked like domething had enabled timer 4

  // Reset RCC status flags
  RCC->RSR |= RCC_RSR_RMVF;

  // Enable CSS monitor
  //RCC->CR |= RCC_CR_CSSON;


  cpp_main();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // Nothing to do here; everything is handled in interrupts
       __WFI();  // Wait for interrupt (low power)
    
  }
  /* USER CODE END 3 */
}


