/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

#include "main.h"
#include "stm32h7xx_it.h"

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi4_tx;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void hard_fault_handler_c(unsigned int *hardfault_args) {
#if 0
    volatile uint32_t stacked_r0 = hardfault_args[0];
    volatile uint32_t stacked_r1 = hardfault_args[1];
    volatile uint32_t stacked_r2 = hardfault_args[2];
    volatile uint32_t stacked_r3 = hardfault_args[3];
    volatile uint32_t stacked_r12 = hardfault_args[4];
    volatile uint32_t stacked_lr = hardfault_args[5];
    volatile uint32_t stacked_pc = hardfault_args[6];
    volatile uint32_t stacked_psr = hardfault_args[7];
#endif
    __BKPT(0); // Trigger breakpoint for debugging

    // Optionally log or display the register values
    while (1); // Halt system for debugging
}
/* USER CODE END EV */

//          Cortex Processor Interruption and Exception Handlers          

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
#if 0
void NMI_Handler(void)
{
   while (1)
  {
  }
}
#endif
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  //volatile uint32_t hfsr = SCB->HFSR;
  //volatile uint32_t cfsr = SCB->CFSR;
  //volatile uint32_t bfar = SCB->BFAR;  // Bus Fault Address Register, if applicable
  //volatile uint32_t mmfar = SCB->MMFAR;  // Memory Management Fault Address Register

  __asm volatile(
        "TST lr, #4 \n"
        "ITE EQ \n"
        "MRSEQ r0, MSP \n"
        "MRSNE r0, PSP \n"
        "B hard_fault_handler_c \n"
    );
  while (1)
  {
  }
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void MemManage_Handler(void)
{
	  __BKPT(0); // Trigger breakpoint for debugging
  while (1)
  {
  }
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void BusFault_Handler(void)
{
	  __BKPT(0); // Trigger breakpoint for debugging
  while (1)
  {
  }
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void UsageFault_Handler(void)
{
	  __BKPT(0); // Trigger breakpoint for debugging
  while (1)
  {
  }
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void SVC_Handler(void)
{
	  __BKPT(0); // Trigger breakpoint for debugging
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void DebugMon_Handler(void)
{
	  __BKPT(0); // Trigger breakpoint for debugging
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void PendSV_Handler(void)
{
	  __BKPT(0); // Trigger breakpoint for debugging
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void TIM1_UP_IRQHandler(void)
{

  // Don't do this. the results are unpredicable
  //__enable_irq();  // Re-enable global interrupts to allow nesting

  HAL_TIM_IRQHandler(&htim1);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void TIM8_UP_TIM13_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim8);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void SPI4_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi4);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi4_rx);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi4_tx);
}



