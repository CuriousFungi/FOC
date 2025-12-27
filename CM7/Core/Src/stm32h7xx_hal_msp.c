
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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

/* USER CODE BEGIN Includes */
#include "interrupt_priorities.h"

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_pcd.h"
//#include "stm32h7xx_hal_eth.h"
#include "stm32h7xx_hal_hsem.h"
#include "stm32h7xx_hal_pwr_ex.h"
#include "stm32h7xx_hal_dac.h"
#include "stm32h7xx_hal_adc.h"

extern DMA_HandleTypeDef hdma_adc1;

extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi4_tx;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

//-----------------------------------------------------------------------------
//                              HAL_MspInit
//
// 1.  HAL_SYSCFG_EnableVREFBUF function requires the VREFBUF clock to be enabled.
// 2.  The HAL_GetTick function relies on the SysTick timer, which must be properly
//     configured before HAL_SYSCFG_EnableVREFBUF is called.
// 3. Verify that the VREFBUF address is in perheral memory
//-----------------------------------------------------------------------------
#if 0
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_VREF_CLK_ENABLE();

  HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE0);

  // Enable the Internal Voltage Reference buffer
  HAL_SYSCFG_EnableVREFBUF();

  // Configure the internal voltage reference buffer high impedance mode
  HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);

  // Enable SRAM1
  __HAL_RCC_D2SRAM1_CLK_ENABLE();
}
#else
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_VREF_CLK_ENABLE();

  // Configure the internal voltage reference buffer high impedance mode
  HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_VREF_CLK_ENABLE();


  HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE0);

  // Enable the Internal Voltage Reference buffer
  HAL_SYSCFG_EnableVREFBUF();

  HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);


  // Enable SRAM1
  __HAL_RCC_D2SRAM1_CLK_ENABLE();
}

#endif



//-----------------------------------------------------------------------------
//                              HAL_ADC_MspInit
//-----------------------------------------------------------------------------
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hadc->Instance==ADC1)
  {

    // Initializes the peripherals clock
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 1;
    PeriphClkInitStruct.PLL2.PLL2N = 19;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    // Peripheral clock enable
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /**ADC1 GPIO Configuration
    PA6     ------> ADC1_INP3
    PB1     ------> ADC1_INP5
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance                 = DMA1_Stream0;
    hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);
  }

}

//-----------------------------------------------------------------------------
//                              HAL_ADC_MspInit
//-----------------------------------------------------------------------------
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    // Peripheral clock disable
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA6     ------> ADC1_INP3
    PB1     ------> ADC1_INP5
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }
}

//-----------------------------------------------------------------------------
//                              HAL_DAC_MspInit
//-----------------------------------------------------------------------------
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC1)
  {
    // Peripheral clock enable 
    __HAL_RCC_DAC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

//-----------------------------------------------------------------------------
//                              HAL_DAC_MspDeInit
//-----------------------------------------------------------------------------
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
  if(hdac->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspDeInit 0 */

  /* USER CODE END DAC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC12_CLK_DISABLE();

    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

  /* USER CODE BEGIN DAC1_MspDeInit 1 */

  /* USER CODE END DAC1_MspDeInit 1 */
  }

}

//-----------------------------------------------------------------------------
//                              HAL_SPI_MspInit
//-----------------------------------------------------------------------------
// Changed speed from low to high (not very high, no > 10MHz)
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
 
  //=============================================================
  //                           SPI4
  //=============================================================
  if(hspi->Instance==SPI4)
    {
    /* USER CODE BEGIN SPI4_MspInit 0 */
    
    /* USER CODE END SPI4_MspInit 0 */
    
    /** Initializes the peripherals clock
    */
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
      PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
      {
        Error_Handler();
      }
    
      /* Peripheral clock enable */
      __HAL_RCC_SPI4_CLK_ENABLE();
    
      __HAL_RCC_GPIOE_CLK_ENABLE();
      /**SPI4 GPIO Configuration
      PE2     ------> SPI4_SCK
      PE4     ------> SPI4_NSS
      PE5     ------> SPI4_MISO
      PE6     ------> SPI4_MOSI
      */
      GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
      HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
      /* SPI4 DMA Init */
      /* SPI4_RX Init */
      hdma_spi4_rx.Instance                 = DMA1_Stream3;
      hdma_spi4_rx.Init.Request             = DMA_REQUEST_SPI4_RX;
      hdma_spi4_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
      hdma_spi4_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_spi4_rx.Init.MemInc              = DMA_MINC_DISABLE;
      hdma_spi4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_spi4_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
      hdma_spi4_rx.Init.Mode                = DMA_NORMAL;
      hdma_spi4_rx.Init.Priority            = DMA_PRIORITY_HIGH;
      hdma_spi4_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

      hdma_spi4_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma_spi4_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
      hdma_spi4_rx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

      hdma_spi4_rx.XferCpltCallback         = NULL; // HAL_SPI_RxCpltCallback;
      hdma_spi4_rx.XferHalfCpltCallback     = NULL; //HAL_SPI_RxHalfCpltCallback;
      hdma_spi4_rx.XferM1CpltCallback       = NULL;
      hdma_spi4_rx.XferM1HalfCpltCallback   = NULL;
      hdma_spi4_rx.XferErrorCallback        = NULL;  //HAL_SPI_ErrorCallback;
      hdma_spi4_rx.XferAbortCallback        = NULL;  //HAL_DMA_Abort;

      hdma_spi4_rx.StreamBaseAddress        = DMA1_BASE;
      hdma_spi4_rx.StreamIndex              = 3;
      hdma_spi4_rx.DMAmuxChannel            = DMAMUX1_Channel3;
      DMAMUX1_Channel3->CCR                 = (83 << DMAMUX_CxCR_DMAREQ_ID_Pos);  

      if (HAL_DMA_Init(&hdma_spi4_rx) != HAL_OK)
      {
        Error_Handler();
      }
    
      __HAL_LINKDMA(hspi,hdmarx,hdma_spi4_rx);
    
      /* SPI4_TX Init */
      hdma_spi4_tx.Instance                 = DMA1_Stream4;
      hdma_spi4_tx.Init.Request             = DMA_REQUEST_SPI4_TX;
      hdma_spi4_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_spi4_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_spi4_tx.Init.MemInc              = DMA_MINC_DISABLE;
      hdma_spi4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_spi4_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
      hdma_spi4_tx.Init.Mode                = DMA_NORMAL;
      hdma_spi4_tx.Init.Priority            = DMA_PRIORITY_HIGH;
      hdma_spi4_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
      hdma_spi4_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma_spi4_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
      hdma_spi4_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

      hdma_spi4_tx.XferCpltCallback         = NULL;  //HAL_SPI_TxCpltCallback;
      hdma_spi4_tx.XferHalfCpltCallback     = NULL;  //HAL_SPI_TxHalfCpltCallback;
      hdma_spi4_tx.XferM1CpltCallback       = NULL;
      hdma_spi4_tx.XferM1HalfCpltCallback   = NULL;
      hdma_spi4_tx.XferErrorCallback        = NULL;  //HAL_SPI_ErrorCallback;
      hdma_spi4_tx.XferAbortCallback        = NULL;  // HAL_DMA_Abort;
  
      hdma_spi4_tx.StreamBaseAddress        = DMA1_BASE;
      hdma_spi4_tx.StreamIndex              = 4;
      hdma_spi4_tx.DMAmuxChannel            = DMAMUX1_Channel4;
      
      DMAMUX1_Channel4->CCR                 = (84 << DMAMUX_CxCR_DMAREQ_ID_Pos);  

      if (HAL_DMA_Init(&hdma_spi4_tx) != HAL_OK)
      {
        Error_Handler();
      }
    
      __HAL_LINKDMA(hspi,hdmatx,hdma_spi4_tx);
    
      /* SPI4 interrupt Init */
      HAL_NVIC_SetPriority(SPI4_IRQn, IRQ_PRIORITY_SPI, 0);
      HAL_NVIC_EnableIRQ(  SPI4_IRQn);


      HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, IRQ_PRIORITY_DMA_SPI_RX, 0);
      HAL_NVIC_EnableIRQ(  DMA1_Stream3_IRQn);
    
      HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, IRQ_PRIORITY_DMA_SPI_TX, 0);
      HAL_NVIC_EnableIRQ(  DMA1_Stream4_IRQn);
    }
}

//-----------------------------------------------------------------------------
//                              HAL_SPI_MspDeInit
//-----------------------------------------------------------------------------
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
#if 0    
  //=============================================================
  //                           SPI2
  //=============================================================
  if(hspi->Instance==SPI2)
  {
    // Peripheral clock disable
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC2_C     ------> SPI2_MISO
    PC3_C     ------> SPI2_MOSI
    PB10     ------> SPI2_SCK
    PB12     ------> SPI2_NSS
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_12);

    // SPI2 DMA DeInit
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);

    // SPI2 interrupt DeInit
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  }
  else 
#endif
  //=============================================================
  //                           SPI4
  //=============================================================
  if(hspi->Instance==SPI4)
  {
    // Peripheral clock disable 
    __HAL_RCC_SPI4_CLK_DISABLE();

    /**SPI4 GPIO Configuration
    PE2     ------> SPI4_SCK
    PE4     ------> SPI4_NSS
    PE5     ------> SPI4_MISO
    PE6     ------> SPI4_MOSI
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

    // SPI4 DMA DeInit 
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);

    // SPI4 interrupt DeInit
    HAL_NVIC_DisableIRQ(SPI4_IRQn);
  }
}

//-----------------------------------------------------------------------------
//                           HAL_TIM_Base_MspInit
//-----------------------------------------------------------------------------
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    // Peripheral clock enable
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    // TIM1 interrupt Init
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, IRQ_PRIORITY_TIM1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  }
  else if(htim_base->Instance==TIM2)
  {
    // Peripheral clock enable 
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
  else if(htim_base->Instance==TIM8)
  {
    // Peripheral clock enable
    __HAL_RCC_TIM8_CLK_ENABLE();
    
    // TIM8 interrupt Init
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, IRQ_PRIORITY_TIM8, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  }

}

//-----------------------------------------------------------------------------
//                           HAL_TIM_MspPostInit
//-----------------------------------------------------------------------------
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_9|GPIO_PIN_11;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  else if(htim->Instance==TIM2)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(htim->Instance==TIM8)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM8 GPIO Configuration
    PC6     ------> TIM8_CH1
    PC7     ------> TIM8_CH2
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}

//-----------------------------------------------------------------------------
//                           HAL_TIM_Base_MspDeInit
//-----------------------------------------------------------------------------
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    // Peripheral clock disable
    __HAL_RCC_TIM1_CLK_DISABLE();

    // TIM1 interrupt DeInit 
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
  }
  else if(htim_base->Instance==TIM2)
  {
    // Peripheral clock disable
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
  else if(htim_base->Instance==TIM8)
  {
    // Peripheral clock disable 
    __HAL_RCC_TIM8_CLK_DISABLE();

    // TIM8 interrupt DeInit
    HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  }
}

//-----------------------------------------------------------------------------
//                           HAL_UART_MspInit
//-----------------------------------------------------------------------------
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==USART2)
  {
   // Initializes the peripherals clock
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    // Peripheral clock enable 
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA3     ------> USART2_RX
    PD5     ------> USART2_TX
    */
    GPIO_InitStruct.Pin = USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USART_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(USART_TX_GPIO_Port, &GPIO_InitStruct);
  }
  else if(huart->Instance==USART3)
  {
    // Initializes the peripherals clock
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    // Peripheral clock enable 
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
}

//-----------------------------------------------------------------------------
//                           HAL_UART_MspDeInit
//-----------------------------------------------------------------------------
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {
    // Peripheral clock disable
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA3     ------> USART2_RX
    PD5     ------> USART2_TX
    */
    HAL_GPIO_DeInit(USART_RX_GPIO_Port, USART_RX_Pin);

    HAL_GPIO_DeInit(USART_TX_GPIO_Port, USART_TX_Pin);
  }
  else if(huart->Instance==USART3)
  {
    // Peripheral clock disable
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, STLINK_RX_Pin|STLINK_TX_Pin);
  }
}

//-----------------------------------------------------------------------------
//                           HAL_PCD_MspInit
//-----------------------------------------------------------------------------
void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hpcd->Instance==USB_OTG_FS)
  {
    // Initializes the peripherals clock
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    // Enable USB Voltage detector
    HAL_PWREx_EnableUSBVoltageDetector();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USB_OTG_FS GPIO Configuration
    PA8     ------> USB_OTG_FS_SOF
    PA9     ------> USB_OTG_FS_VBUS
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
  }
}

//-----------------------------------------------------------------------------
//                           HAL_PCD_MspDeInit
//-----------------------------------------------------------------------------
void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB_OTG_FS)
  {
    // Peripheral clock disable
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

    /**USB_OTG_FS GPIO Configuration
    PA8     ------> USB_OTG_FS_SOF
    PA9     ------> USB_OTG_FS_VBUS
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12);
  }
}

//-----------------------------------------------------------------------------
//                           HAL_WWDG_MspInit
//-----------------------------------------------------------------------------
void HAL_WWDG_MspInit(WWDG_HandleTypeDef* hwwdg)
{
  if(hwwdg->Instance==WWDG1)
  {
    // Peripheral clock enable
    HAL_RCCEx_WWDGxSysResetConfig(RCC_WWDG1);
    __HAL_RCC_WWDG1_CLK_ENABLE();
  }
}
