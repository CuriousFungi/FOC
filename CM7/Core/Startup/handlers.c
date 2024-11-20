#include "stm32h7xx_hal.h"


#if 1
     // void __attribute__((used))      NMI_Handler
     void NMI_Handler(void) {
         if (RCC->CIFR & RCC_CIFR_HSECSSF) {
             // Clear the CSS flag
             RCC->CICR |= RCC_CICR_HSECSSC;
     
             // Switch to HSI to maintain system operation
             RCC->CFGR &= ~RCC_CFGR_SW;  // Clear clock source bits
             RCC->CFGR |= RCC_CFGR_SW_HSI;  // Switch to HSI

             __BKPT();
             // Optionally log or indicate an error
             Error_Handler();  // Call your error handling routine
         }
     }




	 // void __attribute__((used))      HardFault_Handler
	 // void __attribute__((used))      MemManage_Handler
	 // void __attribute__((used))      BusFault_Handler
	 // void __attribute__((used))      UsageFault_Handler
	 // void __attribute__((used))      SVC_Handler
	 // void __attribute__((used))      DebugMon_Handler
	 // void __attribute__((used))      PendSV_Handler
	 // void __attribute__((used))      SysTick_Handler
	  void __attribute__((used))      WWDG_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      PVD_AVD_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TAMP_STAMP_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      RTC_WKUP_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      FLASH_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      RCC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      EXTI0_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      EXTI1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      EXTI2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      EXTI3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      EXTI4_IRQHandler(void){__BKPT();}
	 // void __attribute__((used))      DMA1_Stream0_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA1_Stream1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA1_Stream2_IRQHandler(void){__BKPT();}
	 // void __attribute__((used))      DMA1_Stream3_IRQHandler(void){__BKPT();}
	 // void __attribute__((used))      DMA1_Stream4_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA1_Stream5_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA1_Stream6_IRQHandler(void){__BKPT();}
	  //void __attribute__((used))      ADC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      FDCAN1_IT0_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      FDCAN2_IT0_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      FDCAN1_IT1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      FDCAN2_IT1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      EXTI9_5_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM1_BRK_IRQHandler(void){__BKPT();}
	 // void __attribute__((used))      TIM1_UP_IRQHandler
	  void __attribute__((used))      TIM1_TRG_COM_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM1_CC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM4_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      I2C1_EV_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      I2C1_ER_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      I2C2_EV_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      I2C2_ER_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SPI1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SPI2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      USART1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      USART2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      USART3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      EXTI15_10_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      RTC_Alarm_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM8_BRK_TIM12_IRQHandler(void){__BKPT();}
	 // void __attribute__((used))      TIM8_UP_TIM13_IRQHandler
	  void __attribute__((used))      TIM8_TRG_COM_TIM14_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM8_CC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA1_Stream7_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      FMC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SDMMC1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM5_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SPI3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      UART4_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      UART5_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM6_DAC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM7_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA2_Stream0_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA2_Stream1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA2_Stream2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA2_Stream3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA2_Stream4_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      ETH_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      ETH_WKUP_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      FDCAN_CAL_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      CM7_SEV_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      CM4_SEV_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA2_Stream5_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA2_Stream6_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA2_Stream7_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      USART6_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      I2C3_EV_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      I2C3_ER_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      OTG_HS_EP1_OUT_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      OTG_HS_EP1_IN_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      OTG_HS_WKUP_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      OTG_HS_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DCMI_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      CRYP_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HASH_RNG_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      FPU_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      UART7_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      UART8_IRQHandler(void){__BKPT();}
	 // void __attribute__((used))      SPI4_IRQHandler
	  void __attribute__((used))      SPI5_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SPI6_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SAI1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      LTDC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      LTDC_ER_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMA2D_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SAI2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      QUADSPI_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      LPTIM1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      CEC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      I2C4_EV_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      I2C4_ER_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SPDIF_RX_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      OTG_FS_EP1_OUT_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      OTG_FS_EP1_IN_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      OTG_FS_WKUP_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      OTG_FS_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMAMUX1_OVR_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HRTIM1_Master_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HRTIM1_TIMA_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HRTIM1_TIMB_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HRTIM1_TIMC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HRTIM1_TIMD_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HRTIM1_TIME_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HRTIM1_FLT_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DFSDM1_FLT0_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DFSDM1_FLT1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DFSDM1_FLT2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DFSDM1_FLT3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SAI3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SWPMI1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM15_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM16_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      TIM17_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      MDIOS_WKUP_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      MDIOS_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      JPEG_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      MDMA_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SDMMC2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HSEM1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HSEM2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      ADC3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      DMAMUX2_OVR_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      BDMA_Channel0_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      BDMA_Channel1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      BDMA_Channel2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      BDMA_Channel3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      BDMA_Channel4_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      BDMA_Channel5_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      BDMA_Channel6_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      BDMA_Channel7_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      COMP1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      LPTIM2_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      LPTIM3_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      LPTIM4_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      LPTIM5_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      LPUART1_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      WWDG_RST_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      CRS_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      ECC_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      SAI4_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      HOLD_CORE_IRQHandler(void){__BKPT();}
	  void __attribute__((used))      WAKEUP_PIN_IRQHandler(void){__BKPT();}
	  #endif

