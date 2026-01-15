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

// Debug variables for current sensing
volatile float g_adc_to_voltage_a_0_5(0.0f);
volatile float g_centered_voltage_a(0.0f);
volatile float g_adc_to_voltage_b_0_5(0.0f);
volatile float g_centered_voltage_b(0.0f);
volatile float g_current_a_filtered(0.0f);
volatile float g_current_b_filtered(0.0f);
volatile uint16_t g_adc_raw_a(0);
volatile uint16_t g_adc_raw_b(0);
volatile uint32_t g_adc_read_count(0);  // Count ADC reads to verify function is called
volatile uint32_t g_adc_timeout_a_count(0);  // Count ADC channel A timeouts
volatile uint32_t g_adc_timeout_b_count(0);  // Count ADC channel B timeouts
volatile float g_adc_voltage_a_debug(0.0f);  // Detailed voltage debug
volatile float g_adc_voltage_b_debug(0.0f);
volatile float g_current_a_unfiltered(0.0f);  // Current before filtering
volatile float g_current_b_unfiltered(0.0f);  // Current before filtering
volatile float g_calibrated_zero_a(0.0f);     // Auto-detected zero point for channel A
volatile float g_calibrated_zero_b(0.0f);     // Auto-detected zero point for channel B
volatile uint32_t g_calibration_complete(0);  // 0=not done, 1=success, 2=failed (out of range)
volatile float g_calibration_min_a(5.0f);     // Minimum voltage seen during calibration (channel A)
volatile float g_calibration_max_a(0.0f);     // Maximum voltage seen during calibration (channel A)
volatile float g_calibration_min_b(5.0f);     // Minimum voltage seen during calibration (channel B)
volatile float g_calibration_max_b(0.0f);     // Maximum voltage seen during calibration (channel B)

// ADC channel scan debug variables (to find which pins are actually connected)
volatile uint16_t g_adc_ch0(0), g_adc_ch1(0), g_adc_ch2(0), g_adc_ch4(0), g_adc_ch6(0), g_adc_ch7(0);
volatile float g_adc_ch0_v(0.0f), g_adc_ch1_v(0.0f), g_adc_ch2_v(0.0f), g_adc_ch4_v(0.0f), g_adc_ch6_v(0.0f), g_adc_ch7_v(0.0f);

// Backward compatibility aliases for old variable names (for existing plots/monitors)
volatile float& g_centered_voltage_a_absp925 = g_centered_voltage_a;
volatile float& g_centered_voltage_b_absp925 = g_centered_voltage_b;



// Declare the function to get the buffer address from C++
extern uint32_t* get_timestamp_buffer_address(void);

static StepperMotor stepper = StepperMotor(
                                     &hspi4,        //  sensor spi
                                     50,             //  number of pole pairs (was 100, corrected to 50)
                                     1.4f,           //  phase resistance
                                     1.0f,          // TODO: determine real  _KV,
                                     0.0032f,    //4.0f,          // mH inductance
                                     24.0f,         // voltage limit
                                     24.0f,         // power supply voltage limit L298N
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

//=============================================================================
//                      ACS712 CURRENT SENSING IMPLEMENTATION
//=============================================================================
//
// Hardware: Two ACS712-05B Hall-effect current sensors measure phase A and B
//           currents of the 3-phase motor.
//
// ADC Configuration:
//   - ADC1 with DMA in circular mode
//   - 16-bit resolution (65536 counts)
//   - 5V reference voltage
//   - Channel 0: Phase A current sensor
//   - Channel 1: Phase B current sensor
//
// ACS712-05B Specifications:
//   - Sensitivity: 185 mV/A (0.185 V/A)
//   - Zero current output: 2.5V nominal (typically 2.45V - 2.55V)
//   - Output voltage range: 0.5V (@-5A) to 4.5V (@+5A)
//   - Bandwidth: 80 kHz
//   - Noise: ~21mV RMS
//
// Signal Processing Chain:
//   1. ADC DMA → adc_dma_result[] (continuous updates)
//   2. update_amperage() converts ADC counts → voltage → current
//   3. Low-pass filtering reduces noise
//   4. loopFOC() transforms currents: Phase A,B → Clarke → Park → Id, Iq
//
// Calibration:
//   - Zero-current offsets must be measured with motor OFF
//   - Use calibrate_current_sensor_zero_point() function
//   - Update ZERO_CURRENT_VOLTAGE_A and ZERO_CURRENT_VOLTAGE_B constants
//
// Troubleshooting:
//   - Check g_adc_raw_a/b to verify ADC is updating
//   - Check g_adc_to_voltage_a/b to verify voltage conversion (should be 0.5V-4.5V)
//   - Check g_centered_voltage_a/b to verify zero-point (should be ~0V at no load)
//   - Check g_current_a/b_filtered to see final current values
//   - Use check_adc_dma_health() to verify DMA is working
//
//=============================================================================

struct winding_currents
{
  float winding_amperage_a;
  float winding_amperage_b;
};

// Forward declarations
void read_adc_channels_polled(uint16_t* value_a, uint16_t* value_b);

//=============================================================================
//                    AUTO-CALIBRATION SYSTEM
//=============================================================================
//
// The current sensor zero-point calibration is FULLY AUTOMATIC and requires
// NO manual intervention. It runs at startup before motor enable.
//
// HOW IT WORKS:
// 1. PWM timers are initialized by HAL with 0% duty cycle
// 2. H-bridge FETs remain disabled (no gate drive voltage)
// 3. No current flows through motor windings
// 4. ACS712 sensors read their natural zero-point voltage
// 5. 500 samples averaged over 1 second for noise immunity
//
// SAFETY:
// - Motor enable() is called AFTER calibration completes
// - H-bridge hardware ensures no conduction at 0% PWM
// - No possibility of motor movement during calibration
//
// MONITORING:
// Check these variables in STM32CubeMonitor after startup:
//   g_calibration_complete  : 0=not run, 1=success, 2=failed
//   g_calibrated_zero_a     : Should be 3.5V to 4.5V (typical ~4.0V)
//   g_calibrated_zero_b     : Should be 3.5V to 4.5V (typical ~3.8V)
//   g_calibration_min_a     : Minimum voltage during cal (check for noise)
//   g_calibration_max_a     : Maximum voltage during cal (check for noise)
//   g_calibration_min_b     : Minimum voltage during cal
//   g_calibration_max_b     : Maximum voltage during cal
//
// TROUBLESHOOTING:
// If g_calibration_complete = 2 (failed):
//   - Check if g_calibrated_zero_a or g_calibrated_zero_b is < 1.0V
//     → ACS712 not powered (check 5V supply)
//   - Check if values > 4.5V
//     → Wrong sensor type or wiring issue
//   - Check min/max spread > 0.5V
//     → Excessive noise or current leakage through H-bridge
//
//=============================================================================

//-----------------------------------------------------------------------------
//                    calibrate_current_sensor_zero_point (OLD/UNUSED)
//
// Legacy DMA-based calibration - NOT USED (DMA disabled for SPI4 compatibility)
// Kept for reference only
//-----------------------------------------------------------------------------
float calibrate_current_sensor_zero_point(uint8_t channel, uint32_t num_samples = 1000)
{
    const float MAX_16BIT_ADC_COUNT = 65535.0f;
    const float V_REF = 5.0f;

    float sum = 0.0f;
    for (uint32_t i = 0; i < num_samples; i++)
    {
        float voltage = (static_cast<float>(adc_dma_result[channel]) / MAX_16BIT_ADC_COUNT) * V_REF;
        sum += voltage;
        HAL_Delay(1);  // 1ms between samples
    }

    return sum / static_cast<float>(num_samples);
}

//-----------------------------------------------------------------------------
//                    calibrate_current_sensor_zero_point_polled
//
// Calibrates current sensor zero-point using polled ADC reads (not DMA)
//
// This function is called at startup BEFORE motor enable, so PWM is at 0%
// and H-bridge FETs are disabled. No manual intervention required.
//
// Takes 500 samples over 1 second and averages to find zero-current voltage.
// Results stored in g_calibrated_zero_a and g_calibrated_zero_b.
//-----------------------------------------------------------------------------
void calibrate_current_sensor_zero_point_polled(void)
{
    const float MAX_16BIT_ADC_COUNT = 65535.0f;
    const float V_REF = 5.0f;
    const uint32_t NUM_SAMPLES = 500;  // Average over 500 samples

    float sum_a = 0.0f;
    float sum_b = 0.0f;
    float min_a = 5.0f;
    float max_a = 0.0f;
    float min_b = 5.0f;
    float max_b = 0.0f;

    for (uint32_t i = 0; i < NUM_SAMPLES; i++)
    {
        uint16_t raw_a = 0;
        uint16_t raw_b = 0;
        read_adc_channels_polled(&raw_a, &raw_b);

        float voltage_a = (static_cast<float>(raw_a) / MAX_16BIT_ADC_COUNT) * V_REF;
        float voltage_b = (static_cast<float>(raw_b) / MAX_16BIT_ADC_COUNT) * V_REF;

        sum_a += voltage_a;
        sum_b += voltage_b;

        // Track min/max to detect noise or unexpected current flow
        if (voltage_a < min_a) min_a = voltage_a;
        if (voltage_a > max_a) max_a = voltage_a;
        if (voltage_b < min_b) min_b = voltage_b;
        if (voltage_b > max_b) max_b = voltage_b;

        HAL_Delay(2);  // 2ms between samples (total 1 second calibration)
    }

    g_calibrated_zero_a = sum_a / static_cast<float>(NUM_SAMPLES);
    g_calibrated_zero_b = sum_b / static_cast<float>(NUM_SAMPLES);

    // Store min/max for debugging
    g_calibration_min_a = min_a;
    g_calibration_max_a = max_a;
    g_calibration_min_b = min_b;
    g_calibration_max_b = max_b;

    // Validate calibration results (ACS712 zero-point should be 1V to 4.5V)
    // If out of range, sensor may be unpowered or disconnected
    if (g_calibrated_zero_a < 1.0f || g_calibrated_zero_a > 4.5f ||
        g_calibrated_zero_b < 1.0f || g_calibrated_zero_b > 4.5f)
    {
        g_calibration_complete = 2;  // Failed - values out of expected range
    }
    else
    {
        g_calibration_complete = 1;  // Success
    }
}

//=============================================================================
//                      HARDWARE DIAGNOSTIC GUIDE
//=============================================================================
//
// IF SEEING CONSTANT -5.0A CURRENT WITH RAW ADC = 0:
//
// Your STM32H755 ADC pins are configured as:
//   PA6 = ADC1_INP3 (Channel 3) -> Current Sensor A
//   PB1 = ADC1_INP5 (Channel 5) -> Current Sensor B
//
// DIAGNOSTIC STEPS:
//
// 1. CHECK ACS712 POWER:
//    - Measure VCC pin on ACS712 -> should be 5.0V
//    - Measure GND pin on ACS712 -> should be 0V
//    - If no power, check 5V supply to sensors
//
// 2. CHECK ACS712 OUTPUT VOLTAGE:
//    - With NO current flowing (motor off)
//    - Measure ACS712 VOUT pin with multimeter
//    - Should read approximately 2.5V (zero-current output)
//    - If 0V: ACS712 not powered or damaged
//    - If 5V: ACS712 saturated or wiring issue
//
// 3. CHECK STM32 CONNECTION:
//    - Verify ACS712 Sensor A VOUT -> STM32 PA6
//    - Verify ACS712 Sensor B VOUT -> STM32 PB1
//    - Use continuity test or check schematic
//
// 4. MONITOR DEBUG VARIABLES:
//    - Run scan_all_adc_channels() at startup (enabled below)
//    - Check g_adc_ch0_v through g_adc_ch7_v
//    - Look for which channel reads ~2.5V (that's your ACS712!)
//    - If no channels read ~2.5V, ACS712 sensors not connected/powered
//
// 5. IF WRONG PINS:
//    - Update MX_ADC1_Init() in main.c to use correct channels
//    - Update read_adc_channels_polled() to match
//
//=============================================================================

//-----------------------------------------------------------------------------
//                    check_adc_dma_health
//
// Verifies that ADC DMA is actually updating the buffer
// Useful for debugging if current readings appear frozen
//
// @return true if ADC DMA is working, false if buffer is stale
//-----------------------------------------------------------------------------
bool check_adc_dma_health(void)
{
    static uint16_t prev_value_a = 0;
    static uint16_t prev_value_b = 0;
    static uint32_t stale_count = 0;

    // Check if ADC values have changed since last check
    bool values_changed = (adc_dma_result[0] != prev_value_a) ||
                         (adc_dma_result[1] != prev_value_b);

    if (!values_changed)
    {
        stale_count++;
        if (stale_count > 100)  // 100 consecutive stale readings = problem
        {
            return false;  // ADC DMA is not working
        }
    }
    else
    {
        stale_count = 0;
        prev_value_a = adc_dma_result[0];
        prev_value_b = adc_dma_result[1];
    }

    return true;  // ADC DMA appears to be working
}

//-----------------------------------------------------------------------------
//                    read_adc_channels_polled
//
// Reads both ADC channels efficiently using polling instead of DMA
// This avoids DMA1 bandwidth conflict with SPI4 (encoder)
//
// TIMING: Each conversion takes ~2-3us with 8.5 cycle sampling
//         Total time for 2 channels: ~5-6us
//         This is acceptable for 100us (10kHz) FOC loop timing
//
// NOTE: Must match channels configured in MX_ADC1_Init() - currently Channel 3 and 5
//
// @param value_a - Pointer to store channel A (first channel) value
// @param value_b - Pointer to store channel B (second channel) value
//-----------------------------------------------------------------------------
void read_adc_channels_polled(uint16_t* value_a, uint16_t* value_b)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure and read first channel (Channel 3 - matches MX_ADC1_Init)
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
    {
        *value_a = HAL_ADC_GetValue(&hadc1);
    }
    else
    {
        *value_a = 0;
        g_adc_timeout_a_count++;  // Track failures
    }
    HAL_ADC_Stop(&hadc1);

    // Configure and read second channel (Channel 5 - matches MX_ADC1_Init)
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
    {
        *value_b = HAL_ADC_GetValue(&hadc1);
    }
    else
    {
        *value_b = 0;
        g_adc_timeout_b_count++;  // Track failures
    }
    HAL_ADC_Stop(&hadc1);
}

//-----------------------------------------------------------------------------
//                    scan_all_adc_channels
//
// Scans all ADC channels to find which ones are actually connected
// This helps diagnose pin assignment issues
//-----------------------------------------------------------------------------
void scan_all_adc_channels(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    // Test common ADC channels (0-19 for STM32H7)
    uint32_t test_channels[] = {
        ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
        ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7,
        ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11,
        ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15,
        ADC_CHANNEL_16, ADC_CHANNEL_17, ADC_CHANNEL_18, ADC_CHANNEL_19
    };

    // Store results (only showing first 8 for monitoring)
    volatile uint16_t channel_values[20] = {0};
    volatile float channel_voltages[20] = {0.0f};

    for (int i = 0; i < 20; i++)
    {
        sConfig.Channel = test_channels[i];
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            channel_values[i] = HAL_ADC_GetValue(&hadc1);
            channel_voltages[i] = (channel_values[i] / 65535.0f) * 5.0f;
        }
        HAL_ADC_Stop(&hadc1);
        HAL_Delay(5);
    }

    // Put results in global variables for monitoring
    // Look for channels reading ~2.5V (ACS712 zero-current output)
    g_adc_raw_a = channel_values[3];  // Channel 3
    g_adc_raw_b = channel_values[5];  // Channel 5
    g_adc_voltage_a_debug = channel_voltages[3];
    g_adc_voltage_b_debug = channel_voltages[5];

    // Channels 0-7 for additional debugging
    extern volatile uint16_t g_adc_ch0, g_adc_ch1, g_adc_ch2, g_adc_ch4, g_adc_ch6, g_adc_ch7;
    extern volatile float g_adc_ch0_v, g_adc_ch1_v, g_adc_ch2_v, g_adc_ch4_v, g_adc_ch6_v, g_adc_ch7_v;

    g_adc_ch0 = channel_values[0];
    g_adc_ch1 = channel_values[1];
    g_adc_ch2 = channel_values[2];
    g_adc_ch4 = channel_values[4];
    g_adc_ch6 = channel_values[6];
    g_adc_ch7 = channel_values[7];

    g_adc_ch0_v = channel_voltages[0];
    g_adc_ch1_v = channel_voltages[1];
    g_adc_ch2_v = channel_voltages[2];
    g_adc_ch4_v = channel_voltages[4];
    g_adc_ch6_v = channel_voltages[6];
    g_adc_ch7_v = channel_voltages[7];
}

//-----------------------------------------------------------------------------
//                    reconfigure_adc_for_polled_mode
//
// Reconfigures ADC from DMA mode to polled mode
// CRITICAL: MX_ADC1_Init() configures for DMA, but we need polled mode
//-----------------------------------------------------------------------------
volatile uint32_t g_adc_reconfig_status(0);  // 0=not done, 1=success, 2=failed
volatile uint32_t g_adc_calibration_status(0);  // 0=not done, 1=success, 2=failed

void reconfigure_adc_for_polled_mode(void)
{
    // Disable ADC before reconfiguration
    HAL_ADC_Stop(&hadc1);

    // Reconfigure for polled mode (not DMA)
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;  // Use data register, not DMA
    hadc1.Init.ContinuousConvMode = DISABLE;  // Single conversion mode for polled reads
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;  // Single channel at a time
    hadc1.Init.NbrOfConversion = 1;  // Only one channel at a time for polled mode

    // Re-initialize ADC with new settings
    HAL_StatusTypeDef status = HAL_ADC_Init(&hadc1);
    if (status != HAL_OK)
    {
        g_adc_reconfig_status = 2;  // Failed
        Error_Handler();
    }
    else
    {
        g_adc_reconfig_status = 1;  // Success
    }
}

//-----------------------------------------------------------------------------
//                    test_adc_readings
//
// Diagnostic function to verify ADC is reading correctly
// Call this at startup to check if ADC values are reasonable
//-----------------------------------------------------------------------------
void test_adc_readings(void)
{
    uint16_t test_a = 0;
    uint16_t test_b = 0;

    // Read ADC multiple times and average
    const int NUM_TESTS = 10;
    uint32_t sum_a = 0;
    uint32_t sum_b = 0;

    for (int i = 0; i < NUM_TESTS; i++)
    {
        read_adc_channels_polled(&test_a, &test_b);
        sum_a += test_a;
        sum_b += test_b;
        HAL_Delay(10);
    }

    uint16_t avg_a = sum_a / NUM_TESTS;
    uint16_t avg_b = sum_b / NUM_TESTS;

    float voltage_a = (avg_a / 65535.0f) * 5.0f;
    float voltage_b = (avg_b / 65535.0f) * 5.0f;

    // Store in debug variables for monitoring
    g_adc_raw_a = avg_a;
    g_adc_raw_b = avg_b;
    g_adc_voltage_a_debug = voltage_a;
    g_adc_voltage_b_debug = voltage_b;

    // Expected: voltage_a and voltage_b should be around 2.5V at zero current
    // If both are 0V or 5V, ADC is not reading correctly
}

extern "C" winding_currents update_amperage(void)
{
    // ACS712-05B Current Sensor Specifications:
    // - Sensitivity: 185 mV/A (0.185 V/A)
    // - Zero current output: 2.5V nominal (calibrated to actual values below)
    // - Output range: 0.5V (-10A) to 4.5V (+10A) for 20A version
    // - For 5A version: 0.5V at -5A, 2.5V at 0A, 4.5V at +5A

    const float MAX_16BIT_ADC_COUNT = 65535.0f;  // 16-bit ADC
    const float V_REF = 5.0f;  // Reference voltage
    const float ACS712_SENSITIVITY_V_PER_A = 0.185f;  // Volts per Ampere (was incorrectly named)

    // Calibrated zero-current voltage offsets (measured at 0A)
    // CRITICAL: These should be measured with motor OFF and no current flowing
    // Use g_calibrated_zero_a and g_calibrated_zero_b from auto-calibration if available
    const float ZERO_CURRENT_VOLTAGE_A = (g_calibrated_zero_a > 0.1f) ? g_calibrated_zero_a : 2.56f;
    const float ZERO_CURRENT_VOLTAGE_B = (g_calibrated_zero_b > 0.1f) ? g_calibrated_zero_b : 2.59f;

    // Low-pass filter for noise reduction (exponential moving average)
    static float filtered_current_a = 0.0f;
    static float filtered_current_b = 0.0f;
    const float FILTER_ALPHA = 0.3f;  // 0.0 = heavy filter, 1.0 = no filter

    // Read both ADC channels efficiently using polling to avoid DMA1 bandwidth conflict
    // SPI4 encoder DMA has priority - polled ADC allows encoder to maintain timing
    uint16_t adc_raw_a = 0;
    uint16_t adc_raw_b = 0;
    read_adc_channels_polled(&adc_raw_a, &adc_raw_b);

    g_adc_read_count++;  // Debug: verify function is called

    // Capture raw ADC values for debugging
    g_adc_raw_a = adc_raw_a;
    g_adc_raw_b = adc_raw_b;

    // Convert ADC counts to voltage (0V to 5V)
    float adc_to_voltage_a = (static_cast<float>(adc_raw_a) / MAX_16BIT_ADC_COUNT) * V_REF;
    float adc_to_voltage_b = (static_cast<float>(adc_raw_b) / MAX_16BIT_ADC_COUNT) * V_REF;

    // Center the voltage around 0A (remove DC offset)
    float centered_voltage_a = adc_to_voltage_a - ZERO_CURRENT_VOLTAGE_A;
    float centered_voltage_b = adc_to_voltage_b - ZERO_CURRENT_VOLTAGE_B;

    // Convert voltage to current using ACS712 sensitivity
    // I = (V_out - V_zero) / Sensitivity
    float winding_amps_a = centered_voltage_a / ACS712_SENSITIVITY_V_PER_A;
    float winding_amps_b = centered_voltage_b / ACS712_SENSITIVITY_V_PER_A;

    // Debug: capture unfiltered currents
    g_current_a_unfiltered = winding_amps_a;
    g_current_b_unfiltered = winding_amps_b;

    // Apply low-pass filter to reduce noise
    filtered_current_a = (FILTER_ALPHA * winding_amps_a) + ((1.0f - FILTER_ALPHA) * filtered_current_a);
    filtered_current_b = (FILTER_ALPHA * winding_amps_b) + ((1.0f - FILTER_ALPHA) * filtered_current_b);

    // Clamp to sensor's maximum range (±5A for ACS712-05B)
    filtered_current_a = fmaxf(fminf(filtered_current_a, 5.0f), -5.0f);
    filtered_current_b = fmaxf(fminf(filtered_current_b, 5.0f), -5.0f);

    // Update debug variables
    g_adc_to_voltage_a_0_5 = adc_to_voltage_a;
    g_centered_voltage_a = centered_voltage_a;
    g_adc_to_voltage_b_0_5 = adc_to_voltage_b;
    g_centered_voltage_b = centered_voltage_b;
    g_current_a_filtered = filtered_current_a;
    g_current_b_filtered = filtered_current_b;

    // Additional debug for troubleshooting -5A issue
    g_adc_voltage_a_debug = adc_to_voltage_a;
    g_adc_voltage_b_debug = adc_to_voltage_b;

    return {
             .winding_amperage_a = filtered_current_a,
             .winding_amperage_b = filtered_current_b
           };
}

// MOVED TO GLOBAL SCOPE to avoid stack/memory corruption issues
volatile float g_cmd_rps(1.0f);  // Initialize to match rps
volatile float rps = 1.0f; //0.5f;  // Changed from static to volatile global
static uint32_t last_ramp_time = 0;
static bool ramp_initialized = false;

// Debug variables for ramp monitoring
volatile float g_ramp_current_rps(0.0f);
volatile uint32_t g_ramp_update_count(0);
volatile uint32_t g_ramp_elapsed_us(0);
volatile float g_ramp_target_set(0.0f);
volatile float g_rps_static_debug(0.0f);

void update_ramp(void)
{
    // Initialize ramp timing on first call after kickstart
    // Without this, elapsed_us will be huge (micros() - 0) and cause issues
    if (!ramp_initialized)
    {
        last_ramp_time = micros();
        ramp_initialized = true;
        // Keep rps at initial value (0.5) - will ramp up from there

        // CRITICAL: Set target immediately on first call
        g_cmd_rps = rps;
        stepper.update_target_rad_per_sec(rps);
        g_ramp_target_set = rps;
    }

     // CLOSED-LOOP: Ultra-fast ramp for immediate response
     // Ramp at 100 rad/s² (10.0 rad/s increase per 100ms)
     uint32_t current_time = micros();

     // OVERFLOW-SAFE: Handle micros() overflow (occurs every 71.6 minutes)
     uint32_t elapsed_us;
     if (current_time >= last_ramp_time) {
         // Normal case: no overflow
         elapsed_us = current_time - last_ramp_time;
     } else {
         // Overflow occurred: calculate wrapped time
         elapsed_us = (UINT32_MAX - last_ramp_time) + current_time;
     }

     // Debug: Always update these to see if function is being called
     g_ramp_current_rps = rps;
     g_ramp_elapsed_us = elapsed_us;

     // Update ramp every 100ms for smooth acceleration
     if (elapsed_us >= 100000)  // 100ms = 0.1 seconds
     {
         last_ramp_time = current_time;

         // VELOCITY LIMIT for unloaded motor
         // With 20V supply, 1.4Ω resistance, 0.3A no-load current, 50 pole pairs:
         // Max speed ≈ (V_supply - I×R) / (ω_e × flux) ≈ 65 rad/s mechanical
         // Using 45 rad/s as safe operating limit with margin
         const float MAX_TARGET_VELOCITY = 300.0f;  // Achievable with proper feedforward

         if (rps < MAX_TARGET_VELOCITY)
         {
            rps += 2.0f;    // 10.0 rad/s per 100ms = 100 rad/s² acceleration (ULTRA-FAST startup)

            // Clamp to maximum target velocity
            if (rps > MAX_TARGET_VELOCITY)
            {
                rps = MAX_TARGET_VELOCITY;
            }
         }

         g_ramp_update_count++;
      }

      // Debug: Always show current rps value
      g_rps_static_debug = rps;


      g_cmd_rps = rps;
      stepper.update_target_rad_per_sec(rps);
      g_ramp_target_set = rps;

}


extern "C"
void wrapper_control_loop_25us(void)
{
    //if(is_foc_initialized)
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
        winding_currents result = update_amperage();
        stepper.loopFOC(result.winding_amperage_a, result.winding_amperage_b);
        update_ramp();
    }
    else
    {
        // DISABLED: This was causing periodic motor stops every 7 seconds
        // When is_foc_initialized becomes false, this resets rps to 0
        // causing the motor to stop and restart
        // rps = 0.0f;
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
    extern volatile float g_spi_buffer_changed;
    extern volatile float g_spi_buffer_is_dead;
    extern volatile float g_angle_delta_per_read;

    static uint16_t last_received_value = 0;
    static float last_angle = 0.0f;



    stepper.set_async_read_complete();

#if 1
    const uint16_t ERROR_BIT(0x4000);
    const uint16_t PARITY_BIT(0x8000);

    // Ensure memory ordering with a Data Memory Barrier
    __DMB();
    // Ensure data synchronization with a Data Synchronization Barrier
    __DSB();
    SCB_InvalidateDCache_by_Addr ((uint32_t *)&AS5048A::m_spi_as5048_rx_buff, 2);

    volatile uint16_t received_value  = AS5048A::m_spi_as5048_rx_buff[0];

    // BYTE SWAP: STM32 DMA stores bytes in reversed order for AS5048A
    // received_value = ((received_value & 0xFF) << 8) | ((received_value >> 8) & 0xFF);

    //----------------------------------------------
    // Check if value changed from last time
    g_spi_buffer_changed = (received_value != last_received_value) ? 1.0f : 0.0f;

    // Check if value is 0xDEAD (in-progress marker)
    g_spi_buffer_is_dead = (received_value == 0xDEAD) ? 1.0f : 0.0f;
    // Calculate angle delta (how much did it move?)
    if (received_value != 0xDEAD && received_value != last_received_value)
    {
        const float TWO_PI = 6.28318530718f;
        float current_angle = (static_cast<float>(received_value & 0x3FFF) * TWO_PI) / 16384.0f;
        float delta = current_angle - last_angle;
        // Unwrap
        const float PI = 3.14159265359f;
        if (delta > PI) delta -= TWO_PI;
        else if (delta < -PI) delta += TWO_PI;
        g_angle_delta_per_read = delta;  // Should be small (±0.2 rad at most)
        last_angle = current_angle;
    }
    else
    {
        g_angle_delta_per_read = 0.0f;  // No change
    }

    last_received_value = received_value;
    //----------------------------------------

    // Store validated copy for non-ISR consumers (alignment, etc.)
    // Only store if not 0xDEAD (in-progress marker)
    if (received_value != 0xDEAD)
    {
        stepper.store_validated_reading(received_value);
    }

    // CRITICAL: Always update buffers to ensure g_as5048_angle and g_as5048_velocity are updated
    // Even if error bit is set, we still want to see the angle data
    //if (!(received_value & ERROR_BIT))
    {
       stepper.update_buffers(received_value & ~0xC000, micros());
    }

    #if 0
    if (ERROR_BIT == (ERROR_BIT & received_value))
    {
        // Error bit detected - disable and re-enable SPI to clear error state
        __HAL_SPI_DISABLE(&hspi4);

        // Small delay to ensure disable takes effect
        for(volatile int i = 0; i < 10; i++);

        // CRITICAL: Reset HAL state machine
        hspi4.State = HAL_SPI_STATE_READY;
        hspi4.ErrorCode = HAL_SPI_ERROR_NONE;

        // Re-enable SPI to recover from error
        __HAL_SPI_ENABLE(&hspi4);

        stepper.spi_reset_in_progress();
        //stepper.set_async_read_complete();
    }
    #endif

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

  // NOTE: ADC DMA is DISABLED to avoid DMA1 bandwidth conflict with SPI4 encoder
  // All three peripherals (ADC, SPI4_RX, SPI4_TX) share DMA1, causing encoder timing issues
  // Solution: Use polled ADC reads in update_amperage() instead
  // This allows encoder (critical for velocity) to have full DMA bandwidth

  // CRITICAL: Reconfigure ADC from DMA mode (MX_ADC1_Init default) to polled mode
  // This is why ADC was reading 0 even though hardware has 2.5V!
  reconfigure_adc_for_polled_mode();

  // CRITICAL: Run ADC calibration for accurate readings
  // This must be done AFTER reconfiguration and before any ADC conversions
  HAL_StatusTypeDef cal_status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  if (cal_status == HAL_OK)
  {
      g_adc_calibration_status = 1;  // Success
  }
  else
  {
      g_adc_calibration_status = 2;  // Failed - ADC readings may be inaccurate
  }
  HAL_Delay(10);  // Wait for calibration to complete

  // =========================================================================
  // AUTO-CALIBRATION: Zero-Point Current Sensor Calibration
  // =========================================================================
  // This calibration is SAFE and fully automatic:
  //   - PWM timers initialized with 0% duty cycle (H-bridge FETs disabled)
  //   - Motor is not enabled yet (enable() called later in initFOC())
  //   - No current flows through motor windings
  //   - Takes 1 second to average 500 ADC samples per channel
  //
  // Results stored in:
  //   - g_calibrated_zero_a (expected: 3.5V to 4.5V)
  //   - g_calibrated_zero_b (expected: 3.5V to 4.5V)
  //
  // These calibrated values are automatically used by update_amperage()
  // to remove DC offset and calculate true AC current.
  // =========================================================================
  #if 1  // Set to 0 to disable auto-calibration
  HAL_Delay(500);  // Wait for ADC to stabilize after reconfiguration
  calibrate_current_sensor_zero_point_polled();
  HAL_Delay(100);

  // CALIBRATION COMPLETE - Check these variables in STM32CubeMonitor:
  //
  // g_calibration_complete:  1 = Success, 2 = Failed (out of range 1.0-4.5V)
  // g_calibrated_zero_a:     Expected ~3.5-4.5V (measured zero-current voltage)
  // g_calibrated_zero_b:     Expected ~3.5-4.5V (measured zero-current voltage)
  // g_calibration_min_a:     Min voltage seen (should be close to average)
  // g_calibration_max_a:     Max voltage seen (should be close to average)
  // g_calibration_min_b:     Min voltage seen
  // g_calibration_max_b:     Max voltage seen
  //
  // If (max - min) > 0.5V during calibration → excessive noise or current leakage
  // If values < 1.0V → ACS712 not powered (check 5V supply to sensors)
  // If values > 4.5V → wrong sensor type or wiring error
  //
  // The calibrated zero-points are automatically used by update_amperage()
  // to calculate true AC current with DC offset removed.
  #endif

  // =========================================================================
  // CRITICAL: ADC CHANNEL SCAN FOR HARDWARE DIAGNOSTICS
  // =========================================================================
  // This scans ALL ADC channels to find which pins have ACS712 sensors
  // connected. Look for channels reading ~2.5V in debug variables.
  //
  // EXPECTED: g_adc_voltage_a_debug (ch3/PA6) and g_adc_voltage_b_debug (ch5/PB1)
  //           should both be ~2.5V if ACS712 sensors are connected and powered.
  //
  // IF BOTH ARE 0V:
  //   - ACS712 sensors not powered (check 5V supply)
  //   - ACS712 outputs not connected to PA6/PB1
  //   - Check g_adc_ch0_v through g_adc_ch7_v to find which pin has ~2.5V
  //
  // HARDWARE CHECK:
  //   1. Measure ACS712 VCC with multimeter -> should be 5.0V
  //   2. Measure ACS712 VOUT with multimeter -> should be ~2.5V (no current)
  //   3. Verify continuity from ACS712 VOUT to STM32 PA6 and PB1
  // =========================================================================
  scan_all_adc_channels();

  // Test ADC readings before starting FOC
  // This will populate g_adc_raw_a/b and g_adc_voltage_a/b_debug
  // Check these values - should be around 2.5V (32768 counts) at zero current
  test_adc_readings();

  is_foc_initialized = stepper.initFOC();

  // FOC iteration moved to control_loop_25us() for deterministic hardware timing
  // uint32_t last_foc_time = micros();

  while (1)
  {
      // Main loop - FOC now called from TIM8 interrupt via control_loop_25us()
      // uint32_t current_time = micros();
      // uint32_t elapsed = current_time - last_foc_time;

      // if(elapsed >= MICROSECONDS_PER_ITERATION)
      // {
      //     foc_iteration();
      //     last_foc_time = current_time;
      // }

      // Keep main loop alive but idle - FOC runs in interrupt
      HAL_Delay(1);
  }
}
