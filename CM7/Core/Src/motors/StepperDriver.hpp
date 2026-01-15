#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal_conf.h"
extern void Error_Handler(void);
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
extern void MX_TIM1_Init(void);
extern void MX_TIM8_Init(void);


#ifdef __cplusplus
}
#endif

#include <stdio.h>
#include <string.h>
#include <string>
#include <stdint.h>
#include <math.h>

#include "clamp.hpp"
#include "limit.hpp"


extern UART_HandleTypeDef huart2;

//#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern volatile float g_dutycycle_1A;
extern volatile float g_dutycycle_1B;
extern volatile float g_dutycycle_2A;
extern volatile float g_dutycycle_2b;

// Debug variables for D6 (PC7, TIM8_CH2) issue - can be monitored in real-time
extern volatile uint32_t g_debug_ccr2b_written;
extern volatile uint32_t g_debug_ccr2b_actual;
extern volatile float g_debug_duty2b;
extern volatile bool g_debug_cc2ne_enabled;
extern volatile uint32_t g_debug_ccer_value;
extern volatile float g_debug_U_beta;
extern volatile float g_debug_U_beta_clamped;

// Debug variables for D7 (PC6, TIM8_CH1) for comparison
extern volatile uint32_t g_debug_ccr2a_written;
extern volatile uint32_t g_debug_ccr2a_actual;
extern volatile float g_debug_duty2a;

// Additional debug variables for duty cycle calculation
extern volatile bool g_debug_U_beta_positive;
extern volatile float g_debug_duty_cycle_beta;
extern volatile float g_debug_hifactor_2;
extern volatile float g_debug_lofactor_2;



typedef struct {
    float Kp;
    float Ki;
    float integral;
    float integral_max;  // Anti-windup limit
} PIController;

//=============================================================================
//                          StepperDriver Class
//=============================================================================
class StepperDriver
{
    public:

        //---------------------------------------------------------------------
        //                        CTor
        //---------------------------------------------------------------------
        explicit
        StepperDriver(
                            TIM_HandleTypeDef* p_htim_1,
                            TIM_HandleTypeDef* p_htim_2,
                            float              voltage_limit,
                            float              power_supply_voltage,
                            uint32_t           timer_channel_phase_1A,
                            uint32_t           timer_channel_phase_1B,
                            uint32_t           timer_channel_phase_2A,
                            uint32_t           timer_channel_phase_2B

        )
        :    m_p_htim_phase_1(p_htim_1)
        ,    m_p_htim_phase_2(p_htim_2)
        ,    m_power_supply_voltage(power_supply_voltage)
        ,    m_timer_channel_phase_1A(timer_channel_phase_1A)
        ,    m_timer_channel_phase_1B(timer_channel_phase_1B)
        ,    m_timer_channel_phase_2A(timer_channel_phase_2A)
        ,    m_timer_channel_phase_2B(timer_channel_phase_2B)
        ,    m_duty_cycle_limit(0.0f, 1.0f)
        ,    m_voltage_supply_limit(0.0f, m_power_supply_voltage)
        //   the voltage limit needs to be within the power supply's ability
        ,    m_voltage_limit( m_voltage_supply_limit.result(voltage_limit))
        ,    m_initialized(false)
        {
         //   set_frequency_hz(20000);
        	 m_initialized = true;


#if 0
            HAL_StatusTypeDef status;

            MX_TIM1_Init();

            __HAL_RCC_TIM1_CLK_ENABLE();

            status = HAL_TIM_Base_Start(&htim1);

            if(HAL_OK != status) { Error_Handler(); }


            // Start PWM channels on TIM1
            status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

            if(HAL_OK != status) { Error_Handler(); }

            status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

            if(HAL_OK != status) { Error_Handler(); }

            // Introduce a 90-degree phase shift for TIM1
            TIM8->CNT = TIM8->ARR / 4;


            MX_TIM8_Init();

              __HAL_RCC_TIM8_CLK_ENABLE();

            status = HAL_TIM_Base_Start(&htim8);

            if(HAL_OK != status) { Error_Handler(); }


            // Start PWM channels on TIM8
            status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

            if(HAL_OK != status) { Error_Handler(); }

            status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

            if(HAL_OK != status) { Error_Handler(); }

#endif



            return;
        }
        //---------------------------------------------------------------------
        //                          init
        //---------------------------------------------------------------------
        void init()
        {
            #if 1
            set_dutycycles( 0.0f,
                            0.0f,
                            0.0f,
                            0.0f);
            #endif

            m_initialized = true;
        }

        //---------------------------------------------------------------------
        //                          enable
        //---------------------------------------------------------------------
        void enable()
        {
            init();

 #if 0
 // moved to main and this has a TIM5 ref from other board
            char  msg[] = {"enable\r\n"};
            HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(msg), strlen(msg), HAL_MAX_DELAY);

            //uint32_t status;

            __HAL_RCC_TIM1_CLK_ENABLE();
            __HAL_RCC_TIM8_CLK_ENABLE();

            if (m_p_htim_phase_1->State == HAL_TIM_STATE_RESET)
            {
                  HAL_StatusTypeDef
                  status = HAL_TIM_Base_Start(m_p_htim_phase_1);                  handle_status(status);
                  status = HAL_TIM_PWM_Start( m_p_htim_phase_1, TIM_CHANNEL_1);   handle_status(status);
                  status = HAL_TIM_PWM_Start( m_p_htim_phase_1, TIM_CHANNEL_2);   handle_status(status);
            }

            if (m_p_htim_phase_2->State == HAL_TIM_STATE_RESET)
            {
                  TIM5->CNT = TIM5->ARR / 4; // delay 90 degrees

                  HAL_StatusTypeDef
                  status = HAL_TIM_Base_Start(m_p_htim_phase_2);                  handle_status(status);
                  status = HAL_TIM_PWM_Start( m_p_htim_phase_2, TIM_CHANNEL_1);   handle_status(status);
                  status = HAL_TIM_PWM_Start( m_p_htim_phase_2, TIM_CHANNEL_2);   handle_status(status);
            }
  #endif

        }

        //---------------------------------------------------------------------
        //                          disable
        //---------------------------------------------------------------------
        void disable()
        {
            init();

            HAL_TIM_PWM_Stop(m_p_htim_phase_1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(m_p_htim_phase_1, TIM_CHANNEL_2);

            HAL_TIM_PWM_Stop(m_p_htim_phase_2, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(m_p_htim_phase_2, TIM_CHANNEL_2);
        }


        //---------------------------------------------------------------------
        //                         set_frequency_hz
        //---------------------------------------------------------------------
        void set_frequency_hz(uint32_t frequency_hz)
        {
            uint32_t timerFreq = HAL_RCC_GetPCLK1Freq();   // Get timer clock frequency
            uint32_t period    = timerFreq / frequency_hz; // Calculate period

            __HAL_TIM_SET_AUTORELOAD(m_p_htim_phase_1, period - 1);  // Set auto-reload value
            __HAL_TIM_SET_AUTORELOAD(m_p_htim_phase_2, period - 1);  // Set auto-reload value

            m_initialized = true;
        }

        void delay_microseconds(volatile uint32_t microseconds)
         {
             uint32_t clk_cycle_start = DWT->CYCCNT;
             uint32_t clk_cycle_end   = clk_cycle_start + microseconds * (HAL_RCC_GetHCLKFreq() / 1000000);
             while (DWT->CYCCNT < clk_cycle_end);

             return;
         }



        void set_pwm_duty_cycle(float U_alpha,
                                        float U_beta,
                                        float hifactor_1,
                                        float lofactor_1,
                                        float hifactor_2,
                                        float lofactor_2)
        {
            // Debug: Capture driver initialized status
            g_driver_initialized = m_initialized ? 1.0f : 0.0f;

            if(!m_initialized)
            {
                return;
            }

            float duty_cycle_1A(0.0f);
            float duty_cycle_1B(0.0f);
            float duty_cycle_2A(0.0f);
            float duty_cycle_2B(0.0f);

            // DEBUG: Track U_beta value to understand why duty_cycle_2B is non-zero
            g_debug_U_beta = U_beta;  // Store original value

            // Debug: Capture values before clamping
            g_U_alpha_before_clamp = U_alpha;
            g_U_beta_before_clamp = U_beta;
            g_voltage_limit_debug = m_voltage_limit;

            U_alpha = symetric_clamp(U_alpha, m_voltage_limit);
            U_beta  = symetric_clamp(U_beta,  m_voltage_limit);

            // WORKAROUND: If U_beta has a DC offset (positive most of the time),
            // we can add a phase shift by swapping the sign or adding an offset
            // This is a temporary fix - the root cause should be fixed in the FOC calculation
            // TEST: Try inverting U_beta to see if this centers it better
            // U_beta = -U_beta;  // Uncomment to test inversion

            g_debug_U_beta_clamped = U_beta;  // Store clamped value

            // Debug: Capture calculation steps
            g_power_supply_voltage_debug = m_power_supply_voltage;

            float duty_cycle_alpha = m_duty_cycle_limit.result(
                                                fabs(U_alpha)/m_power_supply_voltage);

            float duty_cycle_beta  = m_duty_cycle_limit.result(
                                                fabs(U_beta) /m_power_supply_voltage);

            g_duty_cycle_alpha_raw = duty_cycle_alpha;
            g_duty_cycle_beta_raw = duty_cycle_beta;

            //-------------------------------------------------
            // Only energize one side of alpha channel H-bridge
            // CRITICAL: Use >= 0.0f to ensure zero and positive go to 1B, negative goes to 1A
            // Initialize both to zero first
            duty_cycle_1A = 0.0f;
            duty_cycle_1B = 0.0f;

            // CRITICAL: Check U_alpha sign to determine which channel should be active
            // When U_alpha >= 0: 1B should be active, 1A should be 0
            // When U_alpha < 0: 1A should be active, 1B should be 0
            if( U_alpha >= 0.0f )
            {
                // U_alpha is positive or zero: activate 1B, 1A must be zero
                duty_cycle_1B = duty_cycle_alpha * hifactor_1;
                duty_cycle_1A = 0.0f;  // CRITICAL: Force 1A to zero
            }
            else  // U_alpha < 0.0f
            {
                // U_alpha is negative: activate 1A, 1B must be zero
                duty_cycle_1A = duty_cycle_alpha * lofactor_1;
                duty_cycle_1B = 0.0f;  // CRITICAL: Force 1B to zero
            }

            // FINAL SAFETY CHECK: Double-check that inactive channel is exactly 0.0f
            // This is redundant but ensures no floating point issues
            if(U_alpha >= 0.0f)
            {
                if(duty_cycle_1A != 0.0f) duty_cycle_1A = 0.0f;  // Force 1A to zero
            }
            else
            {
                if(duty_cycle_1B != 0.0f) duty_cycle_1B = 0.0f;  // Force 1B to zero
            }
            //-------------------------------------------------
            // Only energize one side of beta channel H-bridge
            // CRITICAL: Use >= 0.0f to ensure zero and positive go to 2B, negative goes to 2A
            // DEBUG: Track which branch we're taking
            g_debug_duty_cycle_beta = duty_cycle_beta;
            g_debug_hifactor_2 = hifactor_2;
            g_debug_lofactor_2 = lofactor_2;

            // Initialize both to zero first
            duty_cycle_2A = 0.0f;
            duty_cycle_2B = 0.0f;

            // CRITICAL: Check U_beta sign to determine which channel should be active
            // When U_beta >= 0: D6 (2B) should be active, D7 (2A) should be 0
            // When U_beta < 0: D7 (2A) should be active, D6 (2B) should be 0
            if( U_beta >= 0.0f )
            {
                g_debug_U_beta_positive = true;
                // U_beta is positive or zero: activate D6 (2B), D7 (2A) must be zero
                duty_cycle_2B = duty_cycle_beta * hifactor_2;
                duty_cycle_2A = 0.0f;  // CRITICAL: Force 2A to zero
            }
            else  // U_beta < 0.0f
            {
                g_debug_U_beta_positive = false;
                // U_beta is negative: activate D7 (2A), D6 (2B) must be zero
                duty_cycle_2A = duty_cycle_beta * lofactor_2;
                duty_cycle_2B = 0.0f;  // CRITICAL: Force 2B to zero
            }

            // FINAL SAFETY CHECK: Double-check that inactive channel is exactly 0.0f
            // This is redundant but ensures no floating point issues
            if(U_beta >= 0.0f)
            {
                if(duty_cycle_2A != 0.0f) duty_cycle_2A = 0.0f;  // Force 2A to zero
            }
            else
            {
                if(duty_cycle_2B != 0.0f) duty_cycle_2B = 0.0f;  // Force 2B to zero
            }
            //-------------------------------------------------

            // Debug: Capture final duty cycles before hardware write
            g_duty_1A_before_hw = duty_cycle_1A;
            g_duty_1B_before_hw = duty_cycle_1B;
            g_duty_2A_before_hw = duty_cycle_2A;
            g_duty_2B_before_hw = duty_cycle_2B;

            g_dutycycle_1A = duty_cycle_1A;
            g_dutycycle_1B = duty_cycle_1B;
            g_dutycycle_2A = duty_cycle_2A;
            g_dutycycle_2b = duty_cycle_2B;

            set_dutycycles( duty_cycle_1A,
                            duty_cycle_1B,
                            duty_cycle_2A,
                            duty_cycle_2B);

            return;
        }




        //---------------------------------------------------------------------
        //                         set_pwm_duty_cycle
        //---------------------------------------------------------------------
        void set_pwm_duty_cycle(float U_alpha, float U_beta)
        {
            if(!m_initialized)
            {
                return;
            }

            float duty_cycle_1A(0.0f);
            float duty_cycle_1B(0.0f);
            float duty_cycle_2A(0.0f);
            float duty_cycle_2B(0.0f);

            U_alpha = symetric_clamp(U_alpha, m_voltage_limit);
            U_beta  = symetric_clamp(U_beta,  m_voltage_limit);

            float duty_cycle_alpha = m_duty_cycle_limit.result(
                                                fabs(U_alpha)/m_power_supply_voltage);

            float duty_cycle_beta  = m_duty_cycle_limit.result(
                                                fabs(U_beta) /m_power_supply_voltage);



            //-------------------------------------------------
            // Only energize one side of alpha channel H-bridge
            if( U_alpha > 0.0f )
            {
               duty_cycle_1B = duty_cycle_alpha;
               duty_cycle_1A = 0.0f;  // Explicitly ensure other side is zero
            }
            else
            {
               duty_cycle_1A = duty_cycle_alpha;
               duty_cycle_1B = 0.0f;  // Explicitly ensure other side is zero
            }
            //-------------------------------------------------
            // Only energize one side of beta channel H-bridge
            if( U_beta > 0.0f )
            {
                duty_cycle_2B = duty_cycle_beta;
                duty_cycle_2A = 0.0f;  // Explicitly ensure other side is zero
            }
            else
            {
               duty_cycle_2A = duty_cycle_beta;
               duty_cycle_2B = 0.0f;  // Explicitly ensure other side is zero
            }
            //-------------------------------------------------

            set_dutycycles( duty_cycle_1A,
                            duty_cycle_1B,
                            duty_cycle_2A,
                            duty_cycle_2B);

            return;
        }

        //---------------------------------------------------------------------
        //                     get_ccr_value_phase_1
        //---------------------------------------------------------------------
        uint32_t get_ccr_value_phase_1(float duty_cycle)
        {
            // Explicitly handle zero or negative duty cycles
            if(duty_cycle <= 0.0f)
            {
                return 0;
            }

            // Safety check: ensure timer handle is valid
            if(m_p_htim_phase_1 == nullptr || m_p_htim_phase_1->Instance == nullptr)
            {
                return 0;
            }

            uint32_t autoReloadValue = __HAL_TIM_GET_AUTORELOAD(m_p_htim_phase_1);
            float    ccr_f(duty_cycle * static_cast<float>(autoReloadValue));
            uint32_t ccr_value (static_cast<uint32_t>(ccr_f));

            // Clamp to ARR to prevent overflow
            if(ccr_value > autoReloadValue)
            {
                ccr_value = autoReloadValue;
            }

            return ccr_value;
        }

        //---------------------------------------------------------------------
        //                      get_ccr_value_phase_2
        //---------------------------------------------------------------------
        uint32_t get_ccr_value_phase_2(float duty_cycle)
        {
            // Explicitly handle zero or negative duty cycles
            if(duty_cycle <= 0.0f)
            {
                return 0;
            }

            // Safety check: ensure timer handle is valid
            if(m_p_htim_phase_2 == nullptr || m_p_htim_phase_2->Instance == nullptr)
            {
                return 0;
            }

            uint32_t autoReloadValue = __HAL_TIM_GET_AUTORELOAD(m_p_htim_phase_2);
            float    ccr_f(duty_cycle * static_cast<float>(autoReloadValue));
            uint32_t ccr_value (static_cast<uint32_t>(ccr_f));

            // Clamp to ARR to prevent overflow
            if(ccr_value > autoReloadValue)
            {
                ccr_value = autoReloadValue;
            }

            return ccr_value;
        }


        void handle_status( uint32_t status)
        {
     #if 0
            // TODO: HAL_StatusTypeDef
            if(HAL_OK == status)
            {
               char  msg[] = {"OK\r\n"};
               HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(msg ), strlen(msg ), HAL_MAX_DELAY);
            }
            else
            {
                char char_buffer[128];
                sprintf(char_buffer, "Error: %ld\r\n", status);
                HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(char_buffer), strlen(char_buffer), HAL_MAX_DELAY);
            }
      #endif
        }


        //---------------------------------------------------------------------
        //                       set_dutycycles
        //---------------------------------------------------------------------
        void set_dutycycles(float _1A, float _1B, float _2A, float _2B)
        {
            // Safety check: ensure driver is initialized
            if(!m_initialized)
            {
                return;
            }

            // Safety check: ensure timer handles are valid before proceeding
            if(m_p_htim_phase_1 == nullptr || m_p_htim_phase_1->Instance == nullptr ||
               m_p_htim_phase_2 == nullptr || m_p_htim_phase_2->Instance == nullptr)
            {
                return;  // Exit early if timers not initialized
            }

            // DEBUG: Simple marker to verify function is being called
            volatile static uint32_t call_count = 0;
            call_count++;
            (void)call_count;  // Prevent optimization

            uint32_t ccr_value_1A = get_ccr_value_phase_1(_1A);
            uint32_t ccr_value_1B = get_ccr_value_phase_1(_1B);

            uint32_t ccr_value_2A = get_ccr_value_phase_2(_2A);
            uint32_t ccr_value_2B = get_ccr_value_phase_2(_2B);


            //char msg[] = {"set_dutycycles\r\n"};
            //HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(msg), strlen(msg), HAL_MAX_DELAY);

#if 1

            //-----------------------------------------------------------

			#if 0
            if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK) {
                Error_Handler();  // Handle error if stopping fails
            }
            if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2) != HAL_OK) {
                Error_Handler();  // Handle error if stopping fails
            }

            if (HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1) != HAL_OK) {
                Error_Handler();  // Handle error if stopping fails
            }
            if (HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2) != HAL_OK) {
                Error_Handler();  // Handle error if stopping fails
            }

            // Stop the base timer if needed
            if (HAL_TIM_Base_Stop(&htim1) != HAL_OK) {
            	Error_Handler();  // Handle error
            }
            if (HAL_TIM_Base_Stop(&htim8) != HAL_OK) {
            		Error_Handler();  // Handle error
            }
			#endif
            //-----------------------------------------------------------

            // Temporarily comment out to test if this is causing the crash
            // If system runs without crashing, the issue is in these calls
            __HAL_TIM_SET_COMPARE(m_p_htim_phase_1,
                                  m_timer_channel_phase_1A,
                                  ccr_value_1A);

            __HAL_TIM_SET_COMPARE(m_p_htim_phase_1,
                                  m_timer_channel_phase_1B,
                                  ccr_value_1B);

            // Use direct register writes for both TIM8 channels for consistency
            // This ensures both channels are written the same way
            if(m_p_htim_phase_2 != nullptr && m_p_htim_phase_2->Instance != nullptr)
            {
                // Simply write CCR values - let PWM mode handle the output
                // When CCR = 0, PWM mode should output low
                m_p_htim_phase_2->Instance->CCR1 = ccr_value_2A;  // D7 (PC6, TIM8_CH1)
                m_p_htim_phase_2->Instance->CCR2 = ccr_value_2B;  // D6 (PC7, TIM8_CH2)
            }
            else
            {
                // Fallback to HAL macro if handle is invalid
                __HAL_TIM_SET_COMPARE(m_p_htim_phase_2,
                                      m_timer_channel_phase_2A,
                                      ccr_value_2A);
                __HAL_TIM_SET_COMPARE(m_p_htim_phase_2,
                                      m_timer_channel_phase_2B,
                                      ccr_value_2B);
            }

            //-----------------------------------------------------------
            // DIAGNOSTIC: Verify CCR values and timer configuration for D6 (PC7, TIM8_CH2)
            // This helps debug why D6 stays high when it should be low
            // Can monitor these variables in debugger watch window (no breakpoints needed)
            #if 1  // Enabled for debugging - monitor variables in watch window
            {
                // Safety check: only access registers if timer is initialized
                if(m_p_htim_phase_2 != nullptr && m_p_htim_phase_2->Instance != nullptr)
                {
                    // 1. Check if CCR2 is actually being set to 0
                    volatile uint32_t actual_ccr2 = m_p_htim_phase_2->Instance->CCR2;
                    volatile uint32_t actual_ccr1 = m_p_htim_phase_2->Instance->CCR1;

                    // 2. Check CCER register - verify CC2E is enabled and CC2NE (complementary) is disabled
                    volatile uint32_t ccer_tim8 = m_p_htim_phase_2->Instance->CCER;
                    volatile bool cc2e = (ccer_tim8 & TIM_CCER_CC2E) != 0;      // Should be 1
                    volatile bool cc2ne = (ccer_tim8 & TIM_CCER_CC2NE) != 0;     // Should be 0 (complementary disabled)
                    volatile bool cc2p = (ccer_tim8 & TIM_CCER_CC2P) != 0;       // Polarity bit

                    // 3. Check CCMR1 register - verify OC2M is PWM1 mode (0b110 = 6)
                    volatile uint32_t ccmr1_tim8 = m_p_htim_phase_2->Instance->CCMR1;
                    volatile uint32_t oc2m = (ccmr1_tim8 & TIM_CCMR1_OC2M) >> 8U; // Should be 6 for PWM1

                    // 4. Check BDTR register - verify MOE (Main Output Enable) and OSSR/OSSI
                    volatile uint32_t bdtr_tim8 = m_p_htim_phase_2->Instance->BDTR;
                    volatile bool moe = (bdtr_tim8 & TIM_BDTR_MOE) != 0;
                    volatile bool ossr = (bdtr_tim8 & TIM_BDTR_OSSR) != 0;
                    volatile bool ossi = (bdtr_tim8 & TIM_BDTR_OSSI) != 0;

                    // 5. Check GPIO configuration for PC7 (only if GPIOC is accessible)
                    volatile uint32_t pc7_af = 0;
                    volatile uint32_t pc7_mode = 0;
                    if(__HAL_RCC_GPIOC_IS_CLK_ENABLED())
                    {
                        pc7_af = (GPIOC->AFR[0] >> (7 * 4)) & 0xF; // Should be 3 (AF3_TIM8)
                        pc7_mode = (GPIOC->MODER >> (7 * 2)) & 0x3; // Should be 2 (Alternate function)
                    }

                    // DEBUG VARIABLES: These are global so you can monitor them continuously
                    // Add to watch window and use "Live Watch" feature in your debugger
                    extern volatile uint32_t g_debug_ccr2b_written;
                    extern volatile uint32_t g_debug_ccr2b_actual;
                    extern volatile float g_debug_duty2b;
                    extern volatile bool g_debug_cc2ne_enabled;
                    extern volatile uint32_t g_debug_ccer_value;

                    // Update global debug variables (can be monitored in real-time)
                    g_debug_ccr2b_written = ccr_value_2B;
                    g_debug_ccr2b_actual = actual_ccr2;
                    g_debug_duty2b = _2B;
                    g_debug_cc2ne_enabled = cc2ne;
                    g_debug_ccer_value = ccer_tim8;

                    // Also track phase_2A (D7) for comparison
                    extern volatile uint32_t g_debug_ccr2a_written;
                    extern volatile uint32_t g_debug_ccr2a_actual;
                    extern volatile float g_debug_duty2a;
                    g_debug_ccr2a_written = ccr_value_2A;
                    g_debug_ccr2a_actual = actual_ccr1;  // TIM8_CH1 is CCR1
                    g_debug_duty2a = _2A;


                    // Optional: Log problem cases via UART (uncomment if needed)
                    #if 0
                    if(_2B == 0.0f && actual_ccr2 != 0)
                    {
                        // Duty cycle is 0 but register isn't - log this
                        extern UART_HandleTypeDef huart2;
                        char msg[128];
                        sprintf(msg, "D6 Issue: duty=%.3f, ccr_written=%lu, ccr_actual=%lu, cc2ne=%d\r\n",
                                _2B, ccr_value_2B, actual_ccr2, cc2ne ? 1 : 0);
                        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
                    }
                    #endif

                    // Prevent optimization
                    (void)actual_ccr1;
                    (void)cc2e;
                    (void)cc2p;
                    (void)oc2m;
                    (void)moe;
                    (void)ossr;
                    (void)ossi;
                    (void)pc7_af;
                    (void)pc7_mode;
                }
            }
            #endif
            //-----------------------------------------------------------
			#if 0
            HAL_StatusTypeDef status;

            __HAL_RCC_TIM1_CLK_ENABLE();


            // If needed, start the base timer again
            status = HAL_TIM_Base_Start(&htim1);

            if(HAL_OK != status) { Error_Handler(); }

            // Restart TIM1 PWM channels
            status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

            if(HAL_OK != status) { Error_Handler(); }

            status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

            if(HAL_OK != status) { Error_Handler(); }



            __HAL_RCC_TIM8_CLK_ENABLE();

            status = HAL_TIM_Base_Start(&htim8);

            if(HAL_OK != status) { Error_Handler(); }

            // Restart TIM8 PWM channels
            status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

            if(HAL_OK != status) { Error_Handler(); }

            status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

            if(HAL_OK != status) { Error_Handler(); }




            // Force the main output enable for TIM1 and TIM8 if using advanced features
            __HAL_TIM_MOE_ENABLE(&htim1);
            __HAL_TIM_MOE_ENABLE(&htim8);
			#endif

            //-----------------------------------------------------------



            // Trigger update event if preload is enabled
            //TIM1->EGR |= TIM_EGR_UG;
#else
           // __HAL_TIM_SET_COMPARE(m_p_htim_phase_1, m_timer_channel_phase_1A, 500);  // 50% duty cycle
           // __HAL_TIM_SET_COMPARE(m_p_htim_phase_1, m_timer_channel_phase_1B, 500);  // 50% duty cycle
           // __HAL_TIM_SET_COMPARE(m_p_htim_phase_2, m_timer_channel_phase_2A, 500);  // 50% duty cycle
           // __HAL_TIM_SET_COMPARE(m_p_htim_phase_2, m_timer_channel_phase_2B, 500);  // 50% duty cycle


            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, htim1.Init.Period / 2);  // 50% for TIM1 CH1
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, htim1.Init.Period / 2);  // 50% for TIM1 CH2
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, htim8.Init.Period / 2);  // 50% for TIM8 CH1
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, htim8.Init.Period / 2);  // 50% for TIM8 CH3



#endif

        }

    private:

         TIM_HandleTypeDef* m_p_htim_phase_1;
         TIM_HandleTypeDef* m_p_htim_phase_2;

         float              m_power_supply_voltage;
         uint32_t           m_timer_channel_phase_1A;
         uint32_t           m_timer_channel_phase_1B;
         uint32_t           m_timer_channel_phase_2A;
         uint32_t           m_timer_channel_phase_2B;

         Limit<float>       m_duty_cycle_limit;
         Limit<float>       m_voltage_supply_limit;

         float              m_voltage_limit;
         bool               m_initialized;
    };

#endif
