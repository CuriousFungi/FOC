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

//#define USE_HAL_ADC_REGISTER_CALLBACKS (1)

#include "stm32h7xx.h"

#include "stm32h7xx_hal.h" // Ensure correct HAL is included for your MCU
#include "stm32h7xx_hal_adc.h"


extern void DWT_Init(void);


//#include "stm32h755xx.h"
#define TIM1_UP_TIM16_IRQn       ((IRQn_Type)25)  // Example IRQ number, actual value may vary

#define CORE_CLOCK_HZ 400000000U  // 400 MHz for STM32H755


uint16_t as5048_read_buffer;  // 16-bit buffer for storing the angle data
float filtered_velocity = 0.0f;  // Store filtered velocity value



#ifdef __cplusplus
extern "C" {
#endif

void cpp_main(void);
void foc_iteration(void);      
void wrapper_control_loop_25us(void);
void wrapper_sample_as5048_25us(void);
void wrapper_reinit_dma_for_spi(void);


void RefreshWatchdog(void);
void complete_spi_conversion(void);


#ifdef __cplusplus
}
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
DMA_HandleTypeDef hdma_dac1;
uint16_t dac_buffer[100] = {0};

void Fill_DAC_Buffer(uint16_t * pBuff, uint32_t num_items)
{
	for(int i = 0; i < num_items; i++)
		pBuff[i] = 0;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

WWDG_HandleTypeDef hwwdg1;

/* USER CODE BEGIN PV */


int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2)
    {
    	complete_spi_conversion();
    }
}

// If the duty cycle is 0, HAL_TIM_PWM_PulseFinishedCallback might not be invoked.
// Hybrid approach is to use both
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    
    if(htim->Instance == TIM1 || htim->Instance == TIM8)
    {
        wrapper_sample_as5048_25us();
    }
}

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

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2)
    {
        
        //__HAL_TIM_DISABLE(&htim1);
        //__HAL_TIM_DISABLE(&htim8);
 //       wrapper_reinit_dma_for_spi();
        //__HAL_TIM_ENABLE(&htim8);
        //__HAL_TIM_ENABLE(&htim1);
    }
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
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
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_WWDG1_Init(void);
/* USER CODE BEGIN PFP */
void enable_swo(void) {
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


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void MX_DMA_Init(void) ;
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, IRQ_PRIORITY_ADC, 0);     // ADC
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, IRQ_PRIORITY_DMA_SPI_RX, 0);    // SPI Rx
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, IRQ_PRIORITY_DMA_SPI_TX, 0);    // SPI Tx
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

  
    __HAL_DMA_ENABLE_IT(&hdma_spi2_rx, DMA_IT_TE);  // Enable DMA transfer error interrupt
    __HAL_DMA_ENABLE_IT(&hdma_spi2_rx, DMA_IT_TC);

}



HAL_StatusTypeDef Start_ADC_DMA(void);
extern volatile uint16_t* adc_dma_result;

HAL_StatusTypeDef Start_ADC_DMA(void) 
{
    return HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_result, ADC_BUFFER_SIZE);
}





// Define ITM port 0 register address for printf redirection
#define ITM_STIMULUS_PORT0    (*((volatile unsigned int*)0xE0000000)) 
#define ITM_TRACE_EN          (*((volatile unsigned int*)0xE0000E00))

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




void ITM_Init(void) {
    // Preserve other bits in DEMCR and enable only the TRCENA bit
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Set TRCENA, preserving other bits

    // Unlock the ITM
    ITM->LAR = 0xC5ACCE55;

    // Enable ITM with SWOENA and Trace BusID (TCR settings)
    ITM->TCR = 0x0001000D;  // ITMENA + SWOENA

    // Enable stimulus port 0 (used for printf via SWO)
    ITM->TER = 0x00000004;
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void   SystemClock_Config();
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    #if 0
    MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;  // Disable the MPU
    SCB_CleanInvalidateDCache();
    SCB_DisableDCache();
    #endif

  /* USER CODE BEGIN 1 */

	ITM_Init();


  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  uint32_t timeout = 0xFFFF;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */

#if 0
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */

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

  HAL_StatusTypeDef status;
  status = HAL_ADC_RegisterCallback(&hadc1,  HAL_ADC_CONVERSION_COMPLETE_CB_ID, HAL_ADC_ConvCpltCallback);
  if (status != HAL_OK)
  {
      Error_Handler();
  }
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_WWDG1_Init();
  /* USER CODE BEGIN 2 */

  // Enable the cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT
  DWT->CYCCNT = 0;                                // Reset the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable the cycle counter


  MX_TIM1_Init();
  MX_TIM8_Init();
  TIM1->DIER |= TIM_DIER_UIE; 
  
  MX_SPI2_Init();

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

  status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  if(HAL_OK != status) { Error_Handler(); }

  status = HAL_TIM_OC_Start(&htim8,  TIM_CHANNEL_6);
  if(HAL_OK != status) { Error_Handler(); }


  //status = HAL_SPI_RegisterCallback(&hspi2, HAL_SPI_RX_COMPLETE_CB_ID, HAL_SPI_RxCpltCallback);
  //if(HAL_OK != status) { Error_Handler(); }


  TIM1->DIER |= TIM_DIER_UIE;  // Enable update interrupt


  enable_swo();

  printf("Hello, USART2!\r\n");

  // Start DAC with DMA
 // if (Start_ADC_DMA() != HAL_OK) // Ensure Start_ADC_DMA() returns a status
 //  {
 //      Error_Handler();  // Handle any DMA errors specifically
 //  }


  //__enable_irq();
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  extern uint32_t SystemCoreClock;
  SystemCoreClock = 400000000;  // 400 MHz for STM32H7
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
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



/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
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

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN SPI2_Init 2 */
  // see stm32h7xx_hal_msp.c
  __HAL_LINKDMA(&hspi2, hdmarx, hdma_spi2_rx);  // Link RX DMA
  __HAL_LINKDMA(&hspi2, hdmatx, hdma_spi2_tx);  // Link TX DMA

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
 void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 200; // 200 ticks0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
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

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
 void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();


  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
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
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC3REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_6) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 200; // 200 ticks0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

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

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
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

#undef ENABLE_WATCHDOG
#ifdef ENABLE_WATCHDOG
  //* @param None
  //* @retval None  */
static void MX_WWDG1_Init(void)
{

  /* USER CODE BEGIN WWDG1_Init 0 */

  /* USER CODE END WWDG1_Init 0 */

  /* USER CODE BEGIN WWDG1_Init 1 */

  /* USER CODE END WWDG1_Init 1 */
  hwwdg1.Instance = WWDG1;
  hwwdg1.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg1.Init.Window = 64;
  hwwdg1.Init.Counter = 64;
  hwwdg1.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG1_Init 2 */

  /* USER CODE END WWDG1_Init 2 */

}
void RefreshWatchdog(void)
/**
  * Enable DMA controller clock
  */
{
    HAL_WWDG_Refresh(&hwwdg1);
}
#else
void DisableWatchdogs(void) {
  /* DMA controller clock enable */
    __HAL_RCC_WWDG_CLK_DISABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
    IWDG1->KR = 0x0000;  // Prevents the IWDG from starting


    if ((WWDG1->CR & WWDG_CR_WDGA) != 0) {
        __HAL_RCC_WWDG_CLK_DISABLE();  // Disable clock to WWDG as an extreme measure
    }
}
static void MX_WWDG1_Init(void){DisableWatchdogs();}
void RefreshWatchdog(void){return;}
#endif

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */


#if 0
// For Timer 1, Channel 1 (PE9) and Channel 2 (PE11)
GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

// For Timer 8, Channel 1 (PC6) and Channel 3 (PC8)
GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_8;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
