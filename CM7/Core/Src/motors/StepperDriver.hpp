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

            status = HAL_TIM_OC_Start(&htim8,  TIM_CHANNEL_6);

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



            // Only energize one side of alpha channel H-bridge
            if( U_alpha > 0.0f )
            {
               duty_cycle_1B = duty_cycle_alpha * hifactor_1;               
            }
            else
            {
               duty_cycle_1A = duty_cycle_alpha * lofactor_1;  
            }
            
            // Only energize one side of beta channel H-bridge
            if( U_beta > 0.0f )
            {
                duty_cycle_2B = duty_cycle_beta * hifactor_2;
            }
            else
            {
               duty_cycle_2A = duty_cycle_beta * lofactor_2;
            }
            
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



            // Only energize one side of alpha channel H-bridge
            if( U_alpha > 0.0f )
            {
               duty_cycle_1B = duty_cycle_alpha;               
            }
            else
            {
               duty_cycle_1A = duty_cycle_alpha;  
            }
            
            // Only energize one side of beta channel H-bridge
            if( U_beta > 0.0f )
            {
                duty_cycle_2B = duty_cycle_beta;
            }
            else
            {
               duty_cycle_2A = duty_cycle_beta;
            }

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
            uint32_t autoReloadValue = __HAL_TIM_GET_AUTORELOAD(m_p_htim_phase_1);
            float    ccr_f(duty_cycle * static_cast<float>(autoReloadValue));
            uint32_t ccr_value (static_cast<uint32_t>(ccr_f));
            
            return ccr_value;
        }

        //---------------------------------------------------------------------
        //                      get_ccr_value_phase_2
        //---------------------------------------------------------------------
        uint32_t get_ccr_value_phase_2(float duty_cycle)
        {
            uint32_t autoReloadValue = __HAL_TIM_GET_AUTORELOAD(m_p_htim_phase_2);
            float    ccr_f(duty_cycle * static_cast<float>(autoReloadValue));
            uint32_t ccr_value (static_cast<uint32_t>(ccr_f));
            
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
           
            __HAL_TIM_SET_COMPARE(m_p_htim_phase_1, 
                                  m_timer_channel_phase_1A, 
                                  ccr_value_1A);
            
            __HAL_TIM_SET_COMPARE(m_p_htim_phase_1, 
                                  m_timer_channel_phase_1B, 
                                  ccr_value_1B);
                                            
            __HAL_TIM_SET_COMPARE(m_p_htim_phase_2, 
                                  m_timer_channel_phase_2A, 
                                  ccr_value_2A);
               
            __HAL_TIM_SET_COMPARE(m_p_htim_phase_2,
                                  m_timer_channel_phase_2B,
                                  ccr_value_2B);
            

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
