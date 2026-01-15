// TODO: move  determine_sensor_direction and align sensor into higher level classes



#include <limits>
#include <math.h>

#include "foc_utils.hpp"
#include "AS5048A.hpp"
#include "EnumValue.hpp"
#include "clamp.hpp"
#include "ElapsedTime.hpp"

#include "StepperMotor.hpp"

 #define USE_GRADUAL_INTEGRATION_ENABLE 1

extern "C"
{
    extern void update_buffers(uint16_t new_angle, uint32_t new_timestamp);
    extern float calculate_velocity_from_buffer(void);
}

// Forward declaration of winding_currents struct from main_cpp.cpp
struct winding_currents {
    float winding_amperage_a;
    float winding_amperage_b;
};
extern "C" winding_currents update_amperage(void);



//=============================================================================
//                          DEBUG VARIABLE ORGANIZATION
//=============================================================================
//
// Variables are organized into three categories:
//
// 1. ESSENTIAL MONITORING - Key signals for STM32CubeMonitor
//    These are the primary signals you should monitor for motor control
//
// 2. INTERNAL VARIABLES - Used by driver and control loops
//    These are needed for the code to function but are less critical for
//    real-time monitoring
//
// 3. EXTERNAL VARIABLES - Defined elsewhere (sensors, etc.)
//
// To reduce clutter in STM32CubeMonitor, focus on the ESSENTIAL variables.
// The INTERNAL variables are available if you need to debug deeper issues.
//
//=============================================================================

// External sensor variables
extern volatile float g_as5048_angle;
extern volatile float g_as5048_velocity;

//=============================================================================
// ESSENTIAL MONITORING VARIABLES - Keep these for STM32CubeMon
//=============================================================================
//
// RECOMMENDED MONITORING SET for STM32CubeMonitor:
//
// 1. PWM Duty Cycles (0.0-1.0):
//    - g_dutycycle_1A, g_dutycycle_1B, g_dutycycle_2A, g_dutycycle_2b
//    These should show smooth sinusoidal variations
//
// 2. Voltages:
//    - g_U_alpha_debug, g_U_beta_debug (Alpha/Beta frame voltages)
//    - g_Uq_to_setPhaseVoltage (Q-axis voltage command)
//
// 3. Angles:
//    - g_cached_encoder_angle (Raw encoder electrical angle)
//    - g_blended_angle_after_rate_limit (Final commutation angle)
//    - g_angle_to_setPhaseVoltage (Angle sent to inverse Park)
//
// 4. Speed & Control:
//    - g_target_rps_to_cl_controller (Target speed command)
//    - g_as5048_velocity (Measured speed from Kalman filter)
//
// 5. Currents:
//    - g_amperage_q, g_amperage_d (DQ frame currents)
//
//=============================================================================

// PWM Duty Cycles (0.0 to 1.0)
volatile float g_dutycycle_1A(0.0f);
volatile float g_dutycycle_1B(0.0f);
volatile float g_dutycycle_2A(0.0f);
volatile float g_dutycycle_2b(0.0f);
volatile float g_driver_initialized(0.0f);

// Alpha/Beta Frame Voltages
volatile float g_U_alpha_debug(0.0f);
volatile float g_U_beta_debug(0.0f);

// Control Inputs
volatile float g_Uq_to_setPhaseVoltage(0.0f);
volatile float g_angle_to_setPhaseVoltage(0.0f);

// Target Speed & Ramp
volatile float g_target_rps_to_cl_controller(0.0f);

// Encoder Angles
volatile float g_cached_encoder_angle(0.0f);

volatile float g_kickstart_start_angle_elec(0.0f);
volatile float g_kickstart_end_angle_elec(0.0f);
volatile float g_kickstart_actual_rotation_elec(0.0f);
volatile float g_kickstart_target_angle_elec(0.0f);
volatile float g_kickstart_angle_error(0.0f);
volatile float g_kickstart_voltage_applied(0.0f);
volatile float g_kickstart_just_completed(0.0f);

volatile float g_update_target_called(0.0f);
volatile float g_angle_difference_accum_vs_encoder(0.0f);
volatile float g_accumulated_angle(0.0f);
volatile uint32_t g_accumulate_call_count(0);
volatile float g_accumulated_angle_source(0.0f);
volatile float g_encoder_mode_locked(0.0f);
volatile float g_motor_stop_reason(0.0f);  // 1=epsilon, 2=disabled, 3=enabled_false, 4=loopFOC_skip
volatile uint32_t g_cl_function_entry_count(0);
volatile uint32_t g_cl_function_exit_count(0);
volatile float g_target_when_stopped(0.0f);
volatile float g_actual_target_when_stopped(0.0f);
volatile uint32_t g_stop_count(0);
volatile float g_m_target_when_checked(0.0f);
volatile float g_cmd_rps_when_checked(0.0f);
volatile float g_m_enabled_status(0.0f);
volatile float g_debug_kickstart_active(0.0f);
volatile float g_debug_align(0.0f);

// Back-EMF and Inductance Debug Variables
volatile float g_debug_inductance_value(0.0f);
volatile float g_debug_mag_flux_linkage_q(0.0f);
volatile float g_debug_back_emf_q_axis(0.0f);
volatile float g_debug_perm_magnet_flux(0.0f);
volatile float g_debug_measured_elec_rad_per_sec(0.0f);
volatile float g_debug_measured_mech_rad_per_sec(0.0f);
volatile float g_debug_amperage_q_for_bemf(0.0f);


// NEW: Capture discontinuity that causes stall
volatile float g_transition_accumulated_angle_after(0.0f);
volatile float g_transition_angle_jump(0.0f);
volatile float g_transition_voltage_q_before(0.0f);
volatile float g_transition_voltage_q_after(0.0f);
volatile float g_transition_blend_factor(0.0f);
volatile float g_transition_electrical_angle_sent(0.0f);
volatile float g_integral_freeze_active(0.0f);

// Kalman phase offset for encoder mode correction (now handled by AngleTransitionManager)

volatile float g_debug_kalman_elec_at_trans(0.0f);
volatile float g_debug_kalman_phase_error(0.0f);



// Currents
volatile float g_amperage_q(0.0f);
volatile float g_amperage_d(0.0f);

//=============================================================================
// INTERNAL VARIABLES - Used by driver and control loops
//=============================================================================

// Internal duty cycle calculation variables (needed by StepperDriver.hpp)
volatile float g_U_alpha_before_clamp(0.0f);
volatile float g_U_beta_before_clamp(0.0f);
volatile float g_voltage_limit_debug(0.0f);
volatile float g_duty_cycle_alpha_raw(0.0f);
volatile float g_duty_cycle_beta_raw(0.0f);
volatile float g_power_supply_voltage_debug(0.0f);
volatile float g_duty_1A_before_hw(0.0f);
volatile float g_duty_1B_before_hw(0.0f);
volatile float g_duty_2A_before_hw(0.0f);
volatile float g_duty_2B_before_hw(0.0f);
volatile float g_debug_U_beta = 0.0f;
volatile float g_debug_U_beta_clamped = 0.0f;
volatile bool g_debug_U_beta_positive = false;
volatile float g_debug_duty_cycle_beta = 0.0f;
volatile float g_debug_hifactor_2 = 0.0f;
volatile float g_debug_lofactor_2 = 0.0f;

// Current sensing
volatile float g_winding_amps_a(0.0f);
volatile float g_winding_amps_b(0.0f);
volatile float g_current_a_raw(0.0f);
volatile float g_current_b_raw(0.0f);
volatile float g_current_electrical_angle(0.0f);

// Control loop variables
volatile uint32_t g_update_speed_cl_call_count(0);
volatile float g_velocity_error_for_debug(0.0f);
volatile float g_base_voltage_q_feedforward(0.0f);
volatile float g_feedback_voltage_correction(0.0f);
volatile float g_velocity_error_integral(0.0f);
volatile float g_velocity_correction_p(0.0f);
volatile float g_velocity_correction_i(0.0f);
volatile float g_feedforward_current_q(0.0f);
volatile float g_velocity_before_filter(0.0f);
volatile float g_velocity_spike_reject_active(0.0f);
volatile float g_desired_voltage_q_before_clamp(0.0f);
volatile float g_sensor_mech_angle_for_cl(0.0f);
volatile float g_sensor_elec_angle_for_cl(0.0f);
volatile uint32_t g_motor_enabled_status(0);
volatile float g_Ud_to_setPhaseVoltage(0.0f);
volatile uint32_t g_setPhaseVoltage_call_count(0);
volatile float g_m_voltage_q_in_controller(0.0f);

// POST-KICKSTART DIAGNOSTICS
volatile float g_m_target_debug(0.0f);
volatile float g_electrical_angle_to_setPhaseVoltage(0.0f);
volatile float g_actual_target_rps_debug(0.0f);

// Transition debug variables
volatile float g_transition_accumulated_angle_before(0.0f);
volatile float g_transition_m_current_electrical_before(0.0f);
volatile float g_transition_g_as5048_angle(0.0f);
volatile float g_transition_g_cached_encoder_angle(0.0f);
volatile float g_transition_mech_angle_from_sensor(0.0f);
volatile uint32_t g_transition_timestamp(0);
volatile uint32_t g_transition_loopfoc_count(0);
volatile uint32_t g_transition_as5048_update_count(0);

volatile float g_transition_velocity_error(0.0f);
volatile float g_transition_integral_term(0.0f);
volatile float g_debug_accumulated_at_transition(0.0f);
volatile float g_debug_encoder_at_transition(0.0f);
volatile float g_debug_phase_mismatch(0.0f);

volatile uint32_t g_transition_elapsed_us(0);

// New transition debug variables
volatile float g_debug_last_accumulated_at_trans(0.0f);
volatile float g_debug_angle_error_at_trans(0.0f);
volatile uint32_t g_debug_transition_time(0);

// Sensor variables
volatile float g_sensor_raw_angle_radians(0.0f);
volatile float g_sensor_mechanical_phase_angle(0.0f);
volatile uint32_t g_sensor_read_count(0);
volatile float g_encoder_mech_angle_raw(0.0f);
volatile float g_encoder_elec_angle_raw(0.0f);
volatile float g_calibration_offset_elec(0.0f);
volatile float g_angle_used_in_loopFOC(0.0f);
volatile float g_angle_used_in_voltage_commutation(0.0f);
volatile float g_encoder_elec_after_offset(0.0f);

// Alignment variables (used during startup)
volatile float g_align_voltage_active(0.0f);
volatile float g_align_angle_to_inverse_park(0.0f);
volatile float g_align_Uq_commanded(0.0f);
volatile float g_align_Ud_commanded(0.0f);
volatile float g_align_angle_commanded(0.0f);
volatile float g_align_U_alpha_generated(0.0f);
volatile float g_align_U_beta_generated(0.0f);

// Hybrid startup variables
volatile float g_closed_loop_elec_before_blend(0.0f);
volatile float g_blended_angle_before_rate_limit(0.0f);
volatile float g_rate_limiter_active(0.0f);
volatile float g_hybrid_blend_factor(0.0f);
volatile float g_hybrid_open_loop_angle(0.0f);
volatile float g_hybrid_closed_loop_angle(0.0f);
volatile float g_hybrid_measured_speed(0.0f);
volatile float g_angle_change_limited(0.0f);
volatile uint32_t g_loopfoc_update_count(0);

volatile float g_phase_advance_for_commutation(0.0f);
volatile float g_integration_gain(0.0f);

// Angle transition blending variables
volatile float g_encoder_blend_factor(0.0f);        // 0.0=accumulated, 1.0=encoder
volatile uint32_t g_encoder_blend_start_time(0);    // Microsecond timestamp
volatile float g_accumulated_angle_pre_blend(0.0f); // Accumulated angle before blend
volatile float g_encoder_angle_pre_blend(0.0f);     // Encoder angle before blend
volatile float g_blended_result(0.0f);              // Final blended angle

//volatile float g_transition_velocity_error(0.0f);
//volatile float g_transition_integral_term(0.0f);

volatile float g_debug_use_accumulated = 0.0f;
volatile float g_debug_time_since_startup_sec = 0.0f;
volatile uint32_t g_debug_startup_begin_time = 0;


// Transition Kalman debug
volatile float g_debug_kalman_mech(0.0f);
volatile float g_debug_kalman_elec_unbounded(0.0f);
volatile float g_debug_angle_error_unwrapped(0.0f);
volatile float g_debug_angle_error_wrapped(0.0f);
volatile float g_debug_accumulated_before_add(0.0f);
volatile float g_debug_accumulated_after_add(0.0f);
volatile float g_debug_accumulated_after_normalize(0.0f);

volatile float g_debug_kalman_phase_offset(0.0f);


volatile uint32_t startup_begin_time(0);

// Internal control variables
volatile float g_mech_angle_for_foc(0.0f);
volatile float g_elec_angle_for_foc(0.0f);
volatile uint32_t g_loopfoc_counter(0);
volatile float g_measured_velocity_for_control(0.0f);
volatile float g_debug_ramped_speed(0.0f);
volatile float g_phase_advance_electrical(0.0f);
volatile float g_phase_advance_degrees(0.0f);
volatile uint32_t g_debug_epsilon_trigger_count(0);
volatile uint32_t g_angle_zero_skip_counter(0);
volatile float g_delta_seconds_to_cl_controller(0.0f);
volatile float g_debug_actual_target_rps(0.0f);
volatile float g_target_rad_per_sec(0.0f);
volatile float g_ramped_target_rps(0.0f);
volatile uint32_t g_loop_counter(0);
volatile uint32_t g_speed_update_counter(0);
volatile float g_velocity_after_clamp(0.0f);
volatile float g_accumulated_mech_rad(0.0f);
volatile float g_electrical_rad_ref(0.0f);
volatile float g_velocity_correction(0.0f);
volatile float g_target_elec_rad(0.0f);
volatile float g_delta_angle(0.0f);


// Inverse Park transform variables
volatile float    g_U_alpha(0.0f);
volatile float    g_U_beta(0.0f);
volatile float    g_debug_electric_angle_input(0.0f);
volatile float    g_debug_validated_angle_at_cosf(0.0f);
volatile bool     g_motor_enabled(false);
volatile float    g_angle_zero_path_used(0.0f);
volatile float    g_angle_zero_Uq_input(0.0f);
volatile float    g_angle_zero_Ud_input(0.0f);
volatile float    g_angle_zero_U_alpha_calc(0.0f);
volatile float    g_angle_zero_U_beta_calc(0.0f);
volatile uint32_t g_park_transform_counter(0);
volatile float    g_debug_angle_when_zero(0.0f);
volatile float    g_park_sin(0.0f);
volatile float    g_park_cos(0.0f);
volatile float    g_debug_electric_angle_for_sincos(0.0f);
volatile float    g_debug_m_U_beta_before_assign(0.0f);
volatile float    g_debug_m_U_beta_at_assign(0.0f);

// Additional inverse Park transform debug variables
volatile float    g_debug_Uq_input(0.0f);
volatile float    g_debug_Ud_input(0.0f);
volatile float    g_debug_U_beta_calc_sa(0.0f);
volatile float    g_debug_U_beta_calc_ca(0.0f);
volatile float    g_debug_U_beta_term1(0.0f);
volatile float    g_debug_U_beta_term2(0.0f);
volatile float    g_debug_sa_for_term1(0.0f);
volatile float    g_debug_Ud_for_term1(0.0f);
volatile float    g_debug_ca_for_term2(0.0f);
volatile float    g_debug_Uq_for_term2(0.0f);
volatile float    g_debug_ca_at_calc(0.0f);
volatile float    g_debug_sa_at_calc(0.0f);
volatile float    g_debug_Ud_at_calc(0.0f);
volatile float    g_debug_Uq_at_calc(0.0f);
volatile float    g_clean_U_beta_for_plot(0.0f);
volatile float    g_debug_term1_direct(0.0f);
volatile float    g_debug_term2_direct(0.0f);
volatile float    g_debug_term1_verify(0.0f);
volatile float    g_debug_term2_verify(0.0f);
volatile float    g_debug_m_U_beta_calc_direct(0.0f);
volatile float    g_debug_m_U_beta_calc_verify(0.0f);
volatile float    g_debug_m_U_beta_after_calc(0.0f);
volatile float    g_debug_m_U_beta_diff(0.0f);
volatile float    g_debug_angle_before_cos(0.0f);
volatile float    g_debug_a_sin_before_wrap(0.0f);
volatile float    g_debug_a_sin_after_wrap(0.0f);
volatile float    g_debug_cos_via_sinf(0.0f);
volatile float    g_debug_cos_angle_before_wrap(0.0f);
volatile float    g_debug_cos_angle_before_wrap_check(0.0f);
volatile bool     g_debug_cos_angle_needs_wrap(false);
volatile float    g_debug_cos_angle_after_wrap(0.0f);
volatile float    g_debug_cos_angle_before_sinf(0.0f);
volatile float    g_debug_sinf_result(0.0f);
volatile float    g_debug_cosf_result(0.0f);

// Driver debug variables
volatile uint32_t g_debug_ccr2b_written = 0;
volatile uint32_t g_debug_ccr2b_actual = 0;
volatile float g_debug_duty2b = 0.0f;
volatile bool g_debug_cc2ne_enabled = false;
volatile uint32_t g_debug_ccer_value = 0;
volatile uint32_t g_debug_ccr2a_written = 0;
volatile uint32_t g_debug_ccr2a_actual = 0;
volatile float g_debug_duty2a = 0.0f;

// Sensor/alignment debug variables
volatile uint32_t g_get_electric_angle_call_count(0);
volatile float g_as5048_error_flags(0.0f);
volatile float g_align_mech_angle_0(0.0f);
volatile float g_align_retries_0(0.0f);
volatile float g_align_read_success_0(0.0f);
volatile float g_align_fetch_return_0(0.0f);
volatile float g_as5048_angle_at_align_0(0.0f);
volatile float g_align_mech_angle_90(0.0f);
volatile float g_align_retries_90(0.0f);
volatile float g_align_read_success_90(0.0f);
volatile float g_align_fetch_return_90(0.0f);
volatile float g_as5048_angle_at_align_90(0.0f);
volatile float g_align_mech_delta(0.0f);
volatile float g_align_mech_zero(0.0f);
volatile float g_align_retries_zero(0.0f);
volatile float g_align_read_success_zero(0.0f);
volatile float g_align_fetch_return_zero(0.0f);
volatile float g_as5048_angle_at_align_zero(0.0f);
volatile float g_align_offset_calculated(0.0f);
volatile float g_align_direction_inversion(0.0f);
volatile float g_fetch_radians_raw_count_debug(0.0f);
volatile float g_fetch_radians_result_debug(0.0f);
volatile float g_fetch_radians_success_count(0.0f);
volatile float g_fetch_radians_fail_count(0.0f);

// Miscellaneous internal variables
volatile float    g_voltage_q(0.0f);
volatile float    g_voltage_d(0.0f);
volatile uint16_t g_sensor_offset_u16(0);

//=============================================================================

//-----------------------------------------------------------------------------
//                              normalize_angle
//-----------------------------------------------------------------------------
float normalize_angle(float angle)
{
    //const float MAX_ANGLE = 2.0f * M_PI;
    //while (angle >= MAX_ANGLE) angle -= MAX_ANGLE;
    //while (angle < 0.0f) angle += MAX_ANGLE;
    //return angle;
    return _normalizeAngle(angle);;
}

//-----------------------------------------------------------------------------
//                          smooth
//-----------------------------------------------------------------------------
float smooth(float current_value, float previous_value, float alpha)
{
    // Ensure alpha is between 0 and 1; 0 means no smoothing, 1 means full smoothing (no change)
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    // Apply smoothing: blend current and previous values
    return (alpha * current_value) + ((1.0f - alpha) * previous_value);
}

//-----------------------------------------------------------------------------
//                        compute_inverse_park_transform
//
// Function uses sine approximation
//    regular  sin +  cos ~300us    (no memory usaage)
//    approx  _sin + _cos ~110us    (400Byte ~ 20% of memory)
//
//! @param Uq
//! @param Ud
//! @param electric_angle
//-----------------------------------------------------------------------------
#include <cmath>
void sincos(float a, float* s, float* c)
{
  *s = sinf(a);
  *c = cosf(a);
}


unsigned long prev_us = 0;


//=============================================================================
//                          StepperMotor Class
//=============================================================================
StepperMotor::StepperMotor(SPI_HandleTypeDef* hspi,
                                int                pole_pairs,
                                float              phase_resistance,
                                float              KV,
                                float              phase_inductance,
                                float              voltage_limit,
                                float              power_supply_voltage,
                                TIM_HandleTypeDef* p_htim_phase_1,
                                TIM_HandleTypeDef* p_htim_phase_2,
                                uint32_t           timer_channel_phase_1A,
                                uint32_t           timer_channel_phase_1B,
                                uint32_t           timer_channel_phase_2A,
                                uint32_t           timer_channel_phase_2B
)
:     MY_PI( 3.14159265358979323846f)
,     HALF_PI(        1.57079632679f)
,     TWO_PI(         6.28318530718f)
,     THREE_PI(       9.42477796077f)
,     THREE_HALVES_PI(4.71238898038f)
,     RAD_PER_SEC_TO_REV_PER_MIN(60.0f/TWO_PI)
,     NUM_POLE_PAIRS(static_cast<float>(pole_pairs))
,     KV_RPM_PER_VOLT(KV*_SQRT2)  // save back emf constant KV = 1/K_bemf
,     PHASE_RESISTANCE(phase_resistance)
,     PHASE_RESISTANCE_INVERSE(1.0f/PHASE_RESISTANCE)
,     PHASE_INDUCTANCE(phase_inductance)
,     PHASE_INDUCTANCE_INVERSE(1.0f/PHASE_INDUCTANCE)

,     PHASE_RESISTANCE_1KHZ(24.29f)    // LCR measured AC impedance at 1kHz
,     PHASE_RESISTANCE_10KHZ(212.00f)  // LCR measured AC impedance at 10kHz
,     PHASE_INDUCTANCE_1KHZ(0.0039f)   // 3.9 mH --> 0.0039 H
,     PHASE_INDUCTANCE_10KHZ(0.0032f)  // 3.2 mH --> 0.0032 H

,     MIN_ALIGN_ANGLE_DETECT_MOVEMENT(0.1f)       // Minimum angle to detect movement, adjust as needed
,     MAX_SENSOR_ANGLE(TWO_PI)     // Assuming 360 degree range for the sensor
,     NUM_STEPS(100)
,     ROTATION_ANGLE(MY_PI / 2.0f)  // 90 degrees in radians
,     STEP_SIZE(ROTATION_ANGLE / static_cast<float>(NUM_STEPS) )
,     SQUARE_ROOT_OF_3_INVERSE(1.0f/static_cast<float>(compile_time_sqrt(3.0)))
,     PERM_MAGNET_FLUX_LINKAGE(0.015f) // 5 milliWebers
,     m_sensor(hspi)
,     m_driver(p_htim_phase_1,
               p_htim_phase_2,
               voltage_limit,
               power_supply_voltage,
               timer_channel_phase_1A,
               timer_channel_phase_1B,
               timer_channel_phase_2A,
               timer_channel_phase_2B
               )
,     m_U_alpha(0.0f)
,     m_U_beta(0.0f)
,     m_prev_open_loop_timestamp_us(0)
//------------------------------------------
//    from FOCMotor
//------------------------------------------
,   m_target(0.0f)
,   m_target_prev(0.0)
,   m_target_prev_timestamp(0l)
,   m_feed_forward_velocity(0.0f)
,   m_shaft_angle(0.0f)
,   m_shaft_rad_per_sec(0.0f)
,   m_omega_mechanical_rps(0.0f)
,   m_current_electrical_radians(0.0f)
,   m_current_sp(0.0f)
,   m_shaft_velocity_target(0.0f)
,   m_shaft_angle_target(0.0f)
,   m_voltage()
,   m_voltage_prev()
,   m_amperage()
,   m_amperage_prev()
,   m_voltage_bemf(0)
,   m_voltage_sensor_align(0.2f * voltage_limit)  //12.0f)   //power_supply_voltage)
,   m_velocity_index_search(DEF_INDEX_SEARCH_TARGET_VELOCITY)

,   m_voltage_limit(voltage_limit)
,   m_current_limit(DEF_CURRENT_LIM)
,   m_velocity_limit(DEF_VEL_LIM)     //   maximum angular velocity to be used for positioning
,   m_enabled(false)
,   m_motor_status(FOC_MOTOR_STATUS::UNINITIALIZED)
,   m_foc_modulation(FOC_MODULATION_TYPE::SINE_PWM)
,   m_modulation_centered(1)

,   m_torque_control(TORQUE_CONTROL_TYPE::VOLTAGE)  // FOC_CURRENT
,   m_motion_control(MOTION_CONTROL_TYPE::CL_VELOCITY)  //  TORQUE) //OL_VELOCITY  CL_VELOCITY

,   m_PID_amperage_q( DEF_PID_CURR_P,
                      DEF_PID_CURR_I,
                      DEF_PID_CURR_D,
                      DEF_PID_CURR_RAMP,
                      DEF_POWER_SUPPLY)

,   m_PID_amperage_d( DEF_PID_CURR_P,
                      DEF_PID_CURR_I,
                      DEF_PID_CURR_D,
                      DEF_PID_CURR_RAMP,
                      DEF_POWER_SUPPLY)

,   m_PID_velocity(   0.005f,   // P 0.05  .2
                      0.01f,  // I  0.1  20
                      0.005f,  // D .02   .001
                      DEF_PID_VEL_RAMP,
                      100) //DEF_PID_VEL_LIMIT)

,   m_PID_angle(      DEF_P_ANGLE_P,
                      0,
                      0,
                      0,
                      DEF_VEL_LIM)

,   m_LPF_current_winding_a(  DEF_CURR_FILTER_Tf)
,   m_LPF_current_winding_b(  DEF_CURR_FILTER_Tf)

,   m_LPF_current_q(  DEF_CURR_FILTER_Tf)
,   m_LPF_current_d(  DEF_CURR_FILTER_Tf)
,   m_LPF_velocity(   0.5f)
,   m_LPF_angle(      0.0002f)
,   m_LPF_back_emf(   0.05f)

//,   m_motion_downsample(DEF_MOTION_DOWNSMAPLE)
//,   m_motion_cnt(0)
,   m_sensor_offset(0.0f)
,   m_radian_offset_to_electric_zero(NOT_SET)
,   m_sensor_direction(Direction::UNKNOWN)

// TODO: move to another class
,   m_monitor_downsample(DEF_MON_DOWNSMAPLE)
,   m_monitor_start_char('\0')
,   m_monitor_end_char('\0')
,   m_monitor_separator('\t')
,   m_monitor_decimals(4)
,   monitor_variables(_MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE)
//
,   m_p_current_sense(nullptr)
,   m_current_offset_a(100)
,   m_current_offset_b(100)
,   m_hifactor_a(1.0f)
,   m_lofactor_a(1.0f)
,   m_hifactor_b(1.0f)
,   m_lofactor_b(1.0f)
,   m_commanded_speed_radians_per_sec(0.0f)
,   m_commanded_angle_radians(0.0f)
,   m_measured_speed_radians_per_sec(0.0f)
,   m_measured_angle_radians(0.0f)

,   m_target_voltage_q(0.0f)
,   m_accumulated_mechanical_radians(0.0f)
,   m_kickstart_active(false)
,   m_kickstart_finished(false)
,   m_force_use_encoder(false)
,   m_encoder_mode_locked(false)
,   m_current_sample()
{
    m_voltage.q = 0.0f;
    m_voltage.d = 0.0f;
    setPhaseVoltage(0.0f, 0.0f, 0.0f);

    return;
}


void StepperMotor::update_samples(uint16_t raw_count)
{

    m_sensor.update_buffers(raw_count, micros());
}



//-----------------------------------------------------------------------------
//                               init
//-----------------------------------------------------------------------------
void StepperMotor::init()
{
    m_motor_status       = FOC_MOTOR_STATUS::INITIALIZING;

    m_PID_angle.limit      = m_velocity_limit;

   // HAL_Delay(500);
    enable();   // enable motor
   // HAL_Delay(500);

    m_motor_status = FOC_MOTOR_STATUS::UNCALIBRATED;
}

//-----------------------------------------------------------------------------
//                                  disable
//-----------------------------------------------------------------------------
void StepperMotor::disable()
{
  m_driver.disable();

  m_enabled = false;

  // DEBUG: Track when motor gets disabled
  extern volatile float g_motor_stop_reason;
  extern volatile uint32_t g_stop_count;
  g_motor_stop_reason = 2.0f;  // Motor explicitly disabled
  g_stop_count++;
}

//-----------------------------------------------------------------------------
//                                  enable
//-----------------------------------------------------------------------------
void StepperMotor::enable()
{
  m_driver.enable();

  m_enabled = true;
}

//-----------------------------------------------------------------------------
//                                  resistance
//-----------------------------------------------------------------------------
float StepperMotor::resistance(float electical_radians_per_second)
{
//return PHASE_RESISTANCE_10KHZ;

    const float ONE_KHZ_N_RPS(1000  * TWO_PI);
    const float TEN_KHZ_N_RPS(10000 * TWO_PI);


    const float SLOPE = (PHASE_RESISTANCE_10KHZ - PHASE_RESISTANCE_1KHZ)
                      / (TEN_KHZ_N_RPS - ONE_KHZ_N_RPS);

    float resistance(0.0f);

    if(electical_radians_per_second < ONE_KHZ_N_RPS)
    {
       resistance = PHASE_RESISTANCE_1KHZ;
    }
    else
    if(electical_radians_per_second > TEN_KHZ_N_RPS)
    {
       resistance = PHASE_RESISTANCE_10KHZ;
    }
    else
    {
       resistance = PHASE_RESISTANCE_1KHZ
                  + (SLOPE * electical_radians_per_second);
    }

    return resistance;
}

//-----------------------------------------------------------------------------
//                                  inductance
//-----------------------------------------------------------------------------
float StepperMotor::inductance(float electical_radians_per_second)
{
  //  return PHASE_INDUCTANCE_10KHZ;

    const float ONE_KHZ_N_RPS(1000  * TWO_PI);
    const float TEN_KHZ_N_RPS(10000 * TWO_PI);

    const float SLOPE = (PHASE_INDUCTANCE_10KHZ - PHASE_INDUCTANCE_1KHZ)
                      / (TEN_KHZ_N_RPS - ONE_KHZ_N_RPS);


    float inductance(0.0f);

    if(electical_radians_per_second <= ONE_KHZ_N_RPS)
    {
        inductance = PHASE_INDUCTANCE_1KHZ;
    }
    else
    if(electical_radians_per_second >= TEN_KHZ_N_RPS)
    {
        inductance = PHASE_INDUCTANCE_10KHZ;
    }
    else
    {
        inductance = PHASE_INDUCTANCE_1KHZ
                   + SLOPE * (electical_radians_per_second - ONE_KHZ_N_RPS);
    }

    return inductance;
}

//-----------------------------------------------------------------------------
//                                  smooth_voltage_adjustment
//-----------------------------------------------------------------------------
float StepperMotor::smooth_voltage_adjustment( float current_voltage,
                                                          float desired_voltage,
                                                          float smoothing_rate,
                                                          float delta_t)
{
    return current_voltage + (desired_voltage - current_voltage) * smoothing_rate * delta_t;
}

//-----------------------------------------------------------------------------
//                                  smooth_voltage_adjustment
//-----------------------------------------------------------------------------
float StepperMotor::calculate_smoothing_rate(float target_electrical_rps)
{
    const float LOW_SPEED_HZ(3.0f);
    const float LOW_SPEED_THRESHOLD(LOW_SPEED_HZ * TWO_PI);

    // select rates so they sum to 1.0
    const float LOW_SPEED_SMOTHING_RATE(0.1f);           // 0.1-0.5 .3
    const float HIGH_SPEED_SMOOTHING_RATE(0.9f);         // 0.5-0.9 .7


    if (target_electrical_rps < LOW_SPEED_THRESHOLD)
    {
        return LOW_SPEED_SMOTHING_RATE;
    }
    else
    {
        return HIGH_SPEED_SMOOTHING_RATE;
    }
}

//-----------------------------------------------------------------------------
//                                initFOC
//
// return true on success
//-----------------------------------------------------------------------------
bool  StepperMotor::initFOC()
{
    init(); // TODO: should this be 'reset'?

    bool success(true);


    m_motor_status = FOC_MOTOR_STATUS::UNCALIBRATED;

    // CRITICAL: Set kickstart_active flag BEFORE alignment starts
    // This prevents update_ramp() from running during alignment and kickstart
    // Without this, rps ramps up to 80 during alignment, causing issues at transition
    //m_kickstart_active = true;

    // align motor if necessary
    // alignment necessary for encoders!
    // sensor and motor alignment - can be skipped
    // by setting motor.sensor_direction and motor.zero_electric_angle
 // disabled to debug reset  HAL_Delay(500);

update_target_rad_per_sec(10.0);

    HAL_Delay(100);

    extern volatile float g_debug_align;
    g_debug_align = 1.0f;
    success &= alignSensor(); // bitwise intentional

    //success = true;

#if 0
    //m_shaft_angle = m_sensor.read_angle_radians_from_buffer();
    // TBV Is a read really needed here?
    float radians;
    if(!m_sensor.fetch_radians(radians))
    {
        return false;
    }
    m_shaft_angle = radians;
#endif

    if(success)
    {
    	m_motor_status = FOC_MOTOR_STATUS::READY;
    	enable();  // Ensure motor stays enabled after alignment

    	// KICKSTART: Force motor through 90° rotation to break static friction
    	// This gets the motor moving so closed-loop velocity control can take over
        // Note: m_kickstart_active already set to true at start of initFOC()

        m_kickstart_active = true;
    	kickstartMotor();

        m_kickstart_active = false;
        m_kickstart_finished = true;
        m_force_use_encoder = true;

        extern volatile float g_debug_kickstart_active;
        g_debug_kickstart_active = m_kickstart_active ? 1.0f : 0.0f;  // Copy actual value


    }
    else
    {
    	m_motor_status = FOC_MOTOR_STATUS::CALIBRATION_FAILED;
        m_kickstart_active = false;  // Clear flag even on failure

        extern volatile float g_debug_kickstart_active;
        g_debug_kickstart_active = m_kickstart_active ? 1.0f : 0.0f;  // Copy actual value

        disable();
    }

    return success;
}

#if 0
//-----------------------------------------------------------------------------
//                        determine_sensor_direction
//-----------------------------------------------------------------------------
bool StepperMotor::determine_sensor_direction()
{
       bool  success(true);

//return true;

       float mechanical_angle(0.0f);

       float initial_angle = m_sensor.read_angle_radians();

       // Command a 90 degree rotation over 100 steps
       for (uint32_t i = 0; i <= NUM_STEPS; i++ )
       {
           mechanical_angle = initial_angle  +  (i * STEP_SIZE);

           setPhaseVoltage(m_voltage_sensor_align,
                           0.0f,
                           _normalizeAngle(mechanical_to_electrical_radians(mechanical_angle)));

           HAL_Delay(10);
       }

       HAL_Delay(100);
       m_voltage.q = 0.0f;
       m_voltage.d = 0.0f;
       setPhaseVoltage(0.0f, 0.0f,  0.0f);

       float final_angle = m_sensor.read_angle_radians();

       float delta_angle = final_angle - initial_angle;

       // Check for rollover
       if (delta_angle < -MAX_SENSOR_ANGLE + ROTATION_ANGLE)
       {
           delta_angle += MAX_SENSOR_ANGLE;
       }


       if (fabs(delta_angle) < MIN_ALIGN_ANGLE_DETECT_MOVEMENT)
       {
           // Movement too small to determine direction
           success = false;
           m_sensor_direction = Direction::UNKNOWN;
       }

       else if (delta_angle > 0)
       {
          m_sensor_direction = Direction::CW;
       }
       else
       {
         m_sensor_direction = Direction::CCW;
       }

       m_sensor.invert_output(delta_angle < 0.0f);

       return success;
}
#endif

//bool StepperMotor::is_clockwise()
//{
//    return Direction::CW == m_sensor_direction;
//}


//-----------------------------------------------------------------------------
//                            alignSensor
//
// Encoder alignment to electrical 0 angle
//
// return true on success
//-----------------------------------------------------------------------------
//  Disabled on 12.2.2025
// bool StepperMotor::alignSensor()
// {
//   const uint32_t TWO_MILLISECONDS(2);

//   if(true) //(Direction::UNKNOWN == m_sensor_direction)
//   {
//     #if 0
// 	// We don't have a zero position sensor
//     // If using a zero index sensor
//     if(needsSearch()) // TODO: need a clearer func name
//     {
//         success = absoluteZeroSearch();
//     }

//     // exit if index not found
//     if(!success)
//     {
//         return success;
//     }
//     #endif

//     // find natural direction
//     // move one electrical revolution forward
//     // ramp up from 1.5Pi to 3.5Pi
//     float electric_angle;
//     float mid_angle = 0.0f;

//     // Initial Rotation (270 degrees CW)
//     for (int i = 0; i <=500; i++ )
//     {
//         electric_angle = THREE_HALVES_PI + TWO_PI * i / 500.0f;

//         setPhaseVoltage(m_voltage_sensor_align, 0.0f, electric_angle);

//         if(!m_sensor.fetch_radians(mid_angle))
//         {
//             //return false;
//         }

// 	    HAL_Delay(TWO_MILLISECONDS);
//     }

//     //  mid_angle represents the sensor's reading after
//     //  270 degree clockwise motor rotation.
//     if(!m_sensor.fetch_radians(mid_angle))
//     {
//         //return false;
//     }




//     float end_angle = 0.0f;

//     // Reverse Rotation (270 degrees CCW)
//     for (int i = 500; i >=0; i-- )
//     {
//         electric_angle = THREE_HALVES_PI + TWO_PI * i / 500.0f ;

//         setPhaseVoltage(m_voltage_sensor_align, 0.0f, electric_angle);

//         if(!m_sensor.fetch_radians(end_angle))
//         {
//             //return false;
//         }

// 	    HAL_Delay(TWO_MILLISECONDS);
//     }

//     if(!m_sensor.fetch_radians(end_angle))
//     {
//         //return false;
//     }

//     // zero applied voltages
//     setPhaseVoltage(0, 0, 0);

//     HAL_Delay(200);

//     // determine the direction the sensor moved
//     if (mid_angle == end_angle)
//     {
//       return 0; // failed calibration
//     }
//     else
//     if (mid_angle < end_angle)
//     {
//         m_sensor_direction = Direction::CCW;
//     }
//     else
//     {
//         m_sensor_direction = Direction::CW;
//     }

//     // Set direction flag
//     m_sensor.invert_output(mid_angle < end_angle);

//   }


//   // zero electric angle not known
//   if(NOT_SET == m_radian_offset_to_electric_zero)
//   {
//       // align the electrical phases of the motor and sensor
//       // set angle -90(270 = 3PI/2) degrees
//       //float holding_angle_electrical = _normalizeAngle(
//       //                                      mechanical_to_electrical_radians(
//       //                                                THREE_HALVES_PI));

//       //float holding_angle_electrical = 0.0f;

//   //    setPhaseVoltage(m_voltage_sensor_align, 0,  holding_angle_electrical);

//    //   HAL_Delay(200);


//       // get the m_amperage zero electric angle
//       m_sensor_offset                  = m_sensor.read_radians_with_direction();
//       m_radian_offset_to_electric_zero = mechanical_to_electrical_radians(m_sensor_offset);
//       //m_radian_offset_to_electric_zero = get_electric_angle_radians_v2(); //get_electric_angle_radians();

//       g_sensor_offset_u16 = m_sensor.get_count();

//       HAL_Delay(20);

//       // stop everything
//       setPhaseVoltage(0.0f, 00.0f, 0.0f);

//       HAL_Delay(200);
//   }

//   return true; //success;
// }

// refactored on 12.2.2025
// bool StepperMotor::alignSensor()
// {
//     const uint32_t TWO_MILLISECONDS(2U);
//     const int32_t  NUM_STEPS(500);

//     bool success = true;

//     // ------------------------------------------------------------------------
//     // 1) Determine encoder direction relative to applied electrical rotation
//     // ------------------------------------------------------------------------
//     float mid_mechanical_radians = 0.0f;
//     float end_mechanical_radians = 0.0f;

//     // Forward electrical sweep: 1.5π → 1.5π + 2π
//     for (int32_t i = 0; i <= NUM_STEPS; ++i)
//     {
//         const float sweep_fraction =
//             static_cast<float>(i) / static_cast<float>(NUM_STEPS);

//         const float electrical_angle_radians =
//             THREE_HALVES_PI + TWO_PI * sweep_fraction;

//         // NOTE: electrical_angle_radians is already electrical; do NOT call
//         // mechanical_to_electrical_radians() here.
//         setPhaseVoltage(m_voltage_sensor_align,
//                         0.0f,
//                         electrical_angle_radians);

//         (void)m_sensor.fetch_radians(mid_mechanical_radians);

//         HAL_Delay(TWO_MILLISECONDS);
//     }

//     // Reverse electrical sweep: 1.5π + 2π → 1.5π
//     for (int32_t i = NUM_STEPS; i >= 0; --i)
//     {
//         const float sweep_fraction =
//             static_cast<float>(i) / static_cast<float>(NUM_STEPS);

//         const float electrical_angle_radians =
//             THREE_HALVES_PI + TWO_PI * sweep_fraction;

//         setPhaseVoltage(m_voltage_sensor_align,
//                         0.0f,
//                         electrical_angle_radians);

//         (void)m_sensor.fetch_radians(end_mechanical_radians);

//         HAL_Delay(TWO_MILLISECONDS);
//     }

//     if (mid_mechanical_radians < end_mechanical_radians)
//     {
//         // Sensor increases when the field rotates "forward"
//         m_sensor_direction = Direction::CCW;
//         m_sensor.invert_output(true);
//     }
//     else
//     {
//         // Sensor decreases when the field rotates "forward"
//         m_sensor_direction = Direction::CW;
//         m_sensor.invert_output(false);
//     }

//     // ------------------------------------------------------------------------
//     // 2) Determine electrical zero while holding a fixed stator field
//     // ------------------------------------------------------------------------
//     if (NOT_SET == m_radian_offset_to_electric_zero)
//     {
//         // Hold a known electrical angle (0 rad is fine as long as we are consistent)
//         const float holding_electrical_angle_radians = 0.0f;

//         setPhaseVoltage(m_voltage_sensor_align,
//                         0.0f,
//                         holding_electrical_angle_radians);

//         HAL_Delay(200U);

//         // Read the mechanical angle at which the rotor aligns to that field
//         const float mechanical_zero_radians =
//             m_sensor.read_radians_with_direction();

//         // Store raw mechanical angle for debugging/telemetry
//         m_sensor_offset = mechanical_zero_radians;

//         // Convert that mechanical angle to its electrical equivalent;
//         // this is our "electrical zero" offset for later use:
//         //   electrical_now = mech_to_elec(sensor_mech_now) - m_radian_offset_to_electric_zero
//         m_radian_offset_to_electric_zero =
//             mechanical_to_electrical_radians(mechanical_zero_radians);

//         g_sensor_offset_u16 = m_sensor.get_count();

//         HAL_Delay(20U);

//         // Stop driving the motor
//         setPhaseVoltage(0.0f, 0.0f, 0.0f);

//         HAL_Delay(200U);
//     }

//     return success;
// }

// Simplified on 12.2.2025
bool StepperMotor::alignSensor()
{
    const uint32_t SETTLE_MS           = 10U;
    const float    ALIGN_ELECTRICAL_0  = 0.0f;        // electrical angle for "zero" field
    const float    ALIGN_ELECTRICAL_DT = HALF_PI;     // +90 deg electrical step

    float mech_angle_0   = 0.0f;
    float mech_angle_90  = 0.0f;

    // CRITICAL: Disable direction inversion during calibration
    // We need RAW encoder readings to properly determine which direction it moves
    m_sensor.invert_output(false);

    // Read and clear any error flags in the AS5048 sensor before starting alignment
    extern volatile float g_as5048_error_flags;
    g_as5048_error_flags = m_sensor.get_errors();
    m_sensor.clear_error();
    HAL_Delay(10);  // Wait for error clear to complete

    // Give the async SPI reads time to populate the buffer after TIM1 starts
    // Increased to 5000ms (5 seconds) to ensure SPI system is fully operational
    // At 40kHz TIM1 rate, this allows 200,000 SPI reads to complete
    //HAL_Delay(5000);

    // ------------------------------------------------------------------------
    // 1) Snap rotor to a fixed electrical angle and measure mechanical angle
    // ------------------------------------------------------------------------
    g_align_voltage_active = 1.0f;  // Signal that alignment voltage is active
    g_align_Uq_commanded = m_voltage_sensor_align;
    g_align_Ud_commanded = 0.0f;
    g_align_angle_commanded = ALIGN_ELECTRICAL_0;
    g_align_angle_to_inverse_park = ALIGN_ELECTRICAL_0;  // Capture angle being sent

    extern volatile float g_debug_align;
    g_debug_align = 2.0f;
    setPhaseVoltage(m_voltage_sensor_align,
                    0.0f,
                    ALIGN_ELECTRICAL_0);

    g_align_U_alpha_generated = m_U_alpha;
    g_align_U_beta_generated = m_U_beta;

    HAL_Delay(SETTLE_MS);

    // Turn off motor voltage before reading encoder to avoid electrical noise
    g_align_voltage_active = 0.0f;  // Signal that voltage is turned off
    setPhaseVoltage(0.0f, 0.0f, 0.0f);
    HAL_Delay(10);  // Wait for magnetic field to fully stabilize (motor currents to decay)

    // Keep retrying until we get valid data (not 0xDEAD or error bit)
    // The buffer may contain stale initialization data
    bool read_success = false;
    int retry_count = 0;
    for (int i = 0; i < 100; i++)  // Up to 100 attempts
    {
        retry_count++;
        // Use fetch_validated_reading to avoid racing with ISR
        if (m_sensor.fetch_validated_reading(mech_angle_0))
        {
            read_success = true;
            break;  // Got valid data, stop retrying
        }
        HAL_Delay(10);  // Wait 25ms between attempts (40Hz rate)
    }

    // Debug: capture first angle measurement IMMEDIATELY after fetch_radians
    extern volatile float g_align_mech_angle_0;
    extern volatile float g_align_retries_0;
    extern volatile float g_align_read_success_0;
    extern volatile float g_align_fetch_return_0;
    extern volatile float g_as5048_angle_at_align_0;
    extern volatile float g_as5048_angle;
    g_align_mech_angle_0 = mech_angle_0;  // Capture the actual parameter value returned
    g_align_retries_0 = retry_count;
    g_align_read_success_0 = read_success ? 1.0f : 0.0f;
    g_align_fetch_return_0 = read_success ? 1.0f : 0.0f;
    g_as5048_angle_at_align_0 = mech_angle_0;  // Use the parameter, not the global

    if (!read_success)
    {
        // If sensor read fails after retries, bail out
        setPhaseVoltage(0.0f, 0.0f, 0.0f);
        g_debug_align = -2.0f;

        return false;
    }

    // ------------------------------------------------------------------------
    // 2) Move stator field +90 electrical degrees and measure again
    // ------------------------------------------------------------------------
    g_align_voltage_active = 1.0f;  // Signal that alignment voltage is active
    g_align_Uq_commanded = m_voltage_sensor_align;
    g_align_Ud_commanded = 0.0f;
    g_align_angle_commanded = ALIGN_ELECTRICAL_0 + ALIGN_ELECTRICAL_DT;
    g_align_angle_to_inverse_park = ALIGN_ELECTRICAL_0 + ALIGN_ELECTRICAL_DT;  // Capture angle being sent

    g_debug_align = 3.0f;
    setPhaseVoltage(m_voltage_sensor_align,
                    0.0f,
                    ALIGN_ELECTRICAL_0 + ALIGN_ELECTRICAL_DT);

    g_align_U_alpha_generated = m_U_alpha;
    g_align_U_beta_generated = m_U_beta;

    HAL_Delay(SETTLE_MS);

    // Turn off motor voltage before reading encoder to avoid electrical noise
    g_align_voltage_active = 0.0f;  // Signal that voltage is turned off
    setPhaseVoltage(0.0f, 0.0f, 0.0f);
    HAL_Delay(10);  // Wait for magnetic field to fully stabilize (motor currents to decay)

    // Keep retrying until we get valid data (not 0xDEAD or error bit)
    read_success = false;
    retry_count = 0;
    for (int i = 0; i < 100; i++)  // Up to 100 attempts
    {
        retry_count++;
        // Use fetch_validated_reading to avoid racing with ISR
        if (m_sensor.fetch_validated_reading(mech_angle_90))
        {
            read_success = true;
            break;  // Got valid data, stop retrying
        }
        HAL_Delay(10);  // Wait 25ms between attempts (40Hz rate)
    }

    // Debug: capture second angle measurement IMMEDIATELY after fetch_radians
    extern volatile float g_align_mech_angle_90;
    extern volatile float g_align_retries_90;
    extern volatile float g_align_read_success_90;
    extern volatile float g_align_fetch_return_90;
    extern volatile float g_as5048_angle_at_align_90;
    g_align_mech_angle_90 = mech_angle_90;  // Capture the actual parameter value returned
    g_align_retries_90 = retry_count;
    g_align_read_success_90 = read_success ? 1.0f : 0.0f;
    g_align_fetch_return_90 = read_success ? 1.0f : 0.0f;
    g_as5048_angle_at_align_90 = mech_angle_90;  // Use the parameter, not the global

    if (!read_success)
    {
        setPhaseVoltage(0.0f, 0.0f, 0.0f);
        g_debug_align = -3.0f;

        return false;
    }

    // ------------------------------------------------------------------------
    // 3) Determine direction from RAW encoder readings
    // ------------------------------------------------------------------------
    // We applied +90° electrical field rotation (positive direction in stator frame)
    // The rotor should follow this field rotation
    // Measure how the RAW encoder responded (before any inversion)
    //
    // Physical interpretation:
    //   - If raw encoder INCREASED: encoder naturally tracks positive field rotation
    //     → No inversion needed, encoder direction matches FOC convention
    //   - If raw encoder DECREASED: encoder counts backward relative to field rotation
    //     → Inversion needed to make encoder match FOC convention
    //
    float mech_delta = mech_angle_90 - mech_angle_0;

    // Handle wraparound: if delta is large negative, encoder wrapped through 0
    if (mech_delta < -MY_PI)
    {
        mech_delta += TWO_PI;
    }
    else if (mech_delta > MY_PI)
    {
        mech_delta -= TWO_PI;
    }

    // Debug: capture delta calculation
    extern volatile float g_align_mech_delta;
    g_align_mech_delta = mech_delta;

    // Determine if encoder needs inversion
    // FOC convention: positive electrical field rotation → positive mechanical angle increase
    // If encoder DECREASED when we advanced field +90°, we need to invert it
    bool need_inversion = (mech_delta < 0.0f);

    // Set direction enum for diagnostic/debugging purposes
    // Note: CW/CCW here refers to encoder behavior, not absolute motor direction
    // The enum value represents the SIGN that should be applied to encoder readings
    //
    // Direction::CW (+1):   Encoder naturally increases with positive field rotation
    //                       No inversion needed, use encoder values as-is
    //
    // Direction::CCW (-1):  Encoder naturally decreases with positive field rotation
    //                       Inversion needed (TWO_PI - angle) to match FOC convention
    //
    if (mech_delta > 0.0f)
    {
        m_sensor_direction = Direction::CW;   // Encoder naturally aligned with FOC convention
    }
    else
    {
        m_sensor_direction = Direction::CCW;  // Encoder counts backward, needs inversion
    }

    // CRITICAL: Apply direction inversion based on calibration result
    // This will be used for ALL subsequent encoder reads (after calibration completes)
    // The inversion is applied in get_mechanical_phase_angle_radians()
    // From this point forward, all angle reads will be corrected to match FOC convention
    m_sensor.invert_output(need_inversion);

    // ------------------------------------------------------------------------
    // 4) Re-snap at electrical zero and compute electrical offset
    // ------------------------------------------------------------------------
    g_align_voltage_active = 1.0f;  // Signal that alignment voltage is active
    g_align_Uq_commanded = m_voltage_sensor_align;
    g_align_Ud_commanded = 0.0f;
    g_align_angle_commanded = ALIGN_ELECTRICAL_0;
    g_align_angle_to_inverse_park = ALIGN_ELECTRICAL_0;  // Capture angle being sent

    g_debug_align = 4.0f;
    setPhaseVoltage(m_voltage_sensor_align,
                    0.0f,
                    ALIGN_ELECTRICAL_0);

    g_align_U_alpha_generated = m_U_alpha;
    g_align_U_beta_generated = m_U_beta;

    HAL_Delay(SETTLE_MS);

    // Turn off motor voltage before reading encoder to avoid electrical noise
    g_align_voltage_active = 0.0f;  // Signal that voltage is turned off
    setPhaseVoltage(0.0f, 0.0f, 0.0f);
    HAL_Delay(10);  // Wait for magnetic field to fully stabilize (motor currents to decay)

    // Read mechanical angle (direction inversion NOW applied based on calibration)
    float mech_zero = 0.0f;

    // Keep retrying until we get valid data (not 0xDEAD or error bit)
    read_success = false;
    retry_count = 0;
    for (int i = 0; i < 100; i++)  // Up to 100 attempts
    {
        retry_count++;
        // Use fetch_validated_reading to avoid racing with ISR
        if (m_sensor.fetch_validated_reading(mech_zero))
        {
            read_success = true;
            break;  // Got valid data, stop retrying
        }
        HAL_Delay(10);  // Wait 25ms between attempts (40Hz rate)
    }

    // Debug: capture final mechanical angle IMMEDIATELY after fetch_radians
    extern volatile float g_align_mech_zero;
    extern volatile float g_align_retries_zero;
    extern volatile float g_align_read_success_zero;
    extern volatile float g_align_fetch_return_zero;
    extern volatile float g_as5048_angle_at_align_zero;
    g_align_mech_zero = mech_zero;  // Capture the actual parameter value returned
    g_align_retries_zero = retry_count;
    g_align_read_success_zero = read_success ? 1.0f : 0.0f;
    g_align_fetch_return_zero = read_success ? 1.0f : 0.0f;
    g_as5048_angle_at_align_zero = mech_zero;  // Use the parameter, not the global

    if (!read_success)
    {
        setPhaseVoltage(0.0f, 0.0f, 0.0f);
        g_debug_align = -4.0f;

        return false;
    }

    // Store for diagnostics
    m_sensor_offset = mech_zero;

    // Convert that mechanical angle to electrical; this is the offset we
    // subtract from mech_to_elec() everywhere:
    //
    //   electrical_now = mech_to_elec(sensor_mech_now) - m_radian_offset_to_electric_zero
    //
    m_radian_offset_to_electric_zero =
        mechanical_to_electrical_radians(mech_zero);

    // Debug: capture calculated offset and direction setting
    extern volatile float g_align_offset_calculated;
    extern volatile float g_align_direction_inversion;
    g_align_offset_calculated = m_radian_offset_to_electric_zero;
    g_align_direction_inversion = need_inversion ? 1.0f : 0.0f;

    g_sensor_offset_u16 = m_sensor.get_count();

    HAL_Delay(10U);

    // ------------------------------------------------------------------------
    // 5) Stop driving the motor and initialize open-loop angle accumulator
    // ------------------------------------------------------------------------
    g_align_voltage_active = 0.0f;  // Signal that voltage is turned off

    g_debug_align = 5.0f;
    setPhaseVoltage(0.0f, 0.0f, 0.0f);

    // CRITICAL: Initialize accumulated angle to match rotor's actual position
    // The rotor is now aligned to electrical angle 0, so set mechanical accumulator
    // to the mechanical angle that corresponds to electrical 0
    // Note: mech_zero already has direction correction applied
    m_accumulated_mechanical_radians = mech_zero;

    HAL_Delay(SETTLE_MS);

    return true;
}

//-----------------------------------------------------------------------------
//                              kickstartMotor
//
// Force motor through a 90 degree rotation to break static friction
// This gets the motor moving so closed-loop velocity control can take over
//-----------------------------------------------------------------------------
#if 0
void StepperMotor::kickstartMotor()
{
    // KICKSTART: Force rotation through 270 degrees (like alignment did)
    // This breaks static friction and gets the motor moving
    // Use same voltage and timing as alignment for strong, reliable startup

    const float KICKSTART_VOLTAGE = m_voltage_sensor_align;  // Same as alignment (~5V)
    const int NUM_STEPS = 50;              // 50 steps over 270 degrees
    const float ANGLE_INCREMENT = (3.0f * HALF_PI) / NUM_STEPS;  // 270° / 50 steps = 5.4° per step
    //const int STEP_DELAY_MS = 50;          // 50ms per step = 2.5 seconds total (strong pulses)
    const uint32_t STEP_DELAY_US = 50000;  // 50ms in microseconds

    // Get current mechanical angle
    float start_angle = m_sensor.get_mechanical_phase_angle_radians();

    // CRITICAL: Direction must match what closed-loop will use!
    // CCW means sensor WAS inverted, so now it counts forward - use positive direction
    // CW means sensor counts naturally forward - use positive direction
    // Always use positive direction (target velocity is positive)
    float direction_sign = 1.0f;

    uint32_t step_start_time = micros();
    int current_step = 0;

    // Step through 270 degrees in correct direction
    while (current_step <= NUM_STEPS)
    {
        uint32_t current_time = micros();

        // OVERFLOW-SAFE: Handle micros() overflow
        uint32_t elapsed_us;
        if (current_time >= step_start_time) {
            elapsed_us = current_time - step_start_time;
        } else {
            // Overflow occurred
            elapsed_us = (UINT32_MAX - step_start_time) + current_time;
        }

        if (elapsed_us >= STEP_DELAY_US)
        {
            float mech_angle = start_angle + (direction_sign * current_step * ANGLE_INCREMENT);
            float elec_angle = mechanical_to_electrical_radians(mech_angle);
            elec_angle = _normalizeAngle(elec_angle);

            // Apply voltage at this angle
           setPhaseVoltage(KICKSTART_VOLTAGE, 0.0f, elec_angle);

            current_step++;
            step_start_time = current_time;
        }
        // Yield to interrupts without blocking
        __WFI();  // Wait For Interrupt (low power)

    }

    // NO GAP - closed-loop takes over immediately
    // Motor should now be moving slightly - ready for closed-loop
}
#endif

void StepperMotor::kickstartMotor()
{
    // CLOSED-LOOP KICKSTART: Force rotation through 270 degrees using encoder feedback
    // This breaks static friction and gets the motor moving
    // KEY IMPROVEMENT: Uses ACTUAL rotor position (from encoder) instead of commanded position
    // This eliminates angle discontinuity when transitioning to closed-loop speed control

    const float    BASE_KICKSTART_VOLTAGE = m_voltage_sensor_align;  // Base voltage (~5V)
    const float    TARGET_ROTATION_MECHANICAL = 1.25f; //3.0f * HALF_PI;  // 270 degrees electrical
    const float    TARGET_ROTATION_ELECTRICAL = NUM_POLE_PAIRS * TARGET_ROTATION_MECHANICAL;
    const uint32_t TOTAL_DURATION_US = 25000; //00;  // 2.5 seconds total (same as before)
    const uint32_t UPDATE_INTERVAL_US = 10000;  // 10ms update rate (100Hz)


    // Calculate target final angle
   // float target_final_angle_elec = m_current_electrical_radians + TARGET_ROTATION_ELECTRICAL;

    //const float target_velocity_elec = TARGET_ROTATION_ELECTRICAL / (TOTAL_DURATION_US * 1.0e-6f);


    float start_angle_elec = m_current_electrical_radians;
    uint32_t start_time = micros();
    uint32_t last_update_time = start_time;

    // Closed-loop position control during kickstart
    while ((micros() - start_time) < TOTAL_DURATION_US)
    {
        m_kickstart_active = true;

        uint32_t current_time = micros();

        if ((current_time - last_update_time) >= UPDATE_INTERVAL_US)
        {
            // Calculate where we WANT to be (ramp target position)
            // float elapsed_seconds = (current_time - start_time) * 1.0e-6f;
            // float target_angle_elec = start_angle_elec + (target_velocity_elec * elapsed_seconds);
            // target_angle_elec = _normalizeAngle(target_angle_elec);
            // Constant acceleration trajectory from rest
            float elapsed_seconds = (current_time - start_time) * 1.0e-6f;
            float total_duration_s = TOTAL_DURATION_US * 1.0e-6f;
            float time_fraction = elapsed_seconds / total_duration_s;  // 0.0 to 1.0

            // Parabolic position profile: θ(t) = θ₀ + Δθ × ½ × (t/T)²
            // This gives v(t) = (Δθ/T) × (t/T), reaching v_final = Δθ/T at end
            float target_angle_elec = start_angle_elec + TARGET_ROTATION_ELECTRICAL * 0.5f * time_fraction * time_fraction;
            target_angle_elec = _normalizeAngle(target_angle_elec);

            // Expose for debugging
            g_accumulated_angle = target_angle_elec;


            // Read where we ACTUALLY are (from cached encoder electrical)
            float actual_angle_elec = m_current_electrical_radians;

            // Calculate position error (handle wraparound)
            float angle_error = target_angle_elec - actual_angle_elec;

            // Unwrap angle error (handle 0/2π wraparound)
            if (angle_error > MY_PI)
                angle_error -= TWO_PI;
            else if (angle_error < -MY_PI)
                angle_error += TWO_PI;

            // Position control: Add voltage proportional to error
            // Kp = 2.0 means 2V per radian of error
            const float Kp_position = 2.0f;
            float voltage_correction = Kp_position * angle_error;

            // Clamp correction to reasonable range
            voltage_correction = fmaxf(fminf(voltage_correction, 5.0f), -5.0f);

            // Total voltage = base + correction
            float total_voltage = BASE_KICKSTART_VOLTAGE + voltage_correction;
            total_voltage = fmaxf(fminf(total_voltage, m_voltage_limit), 0.5f);  // Min 0.5V

            // CRITICAL: Apply voltage at ACTUAL rotor angle (not target angle)
            // This ensures continuous tracking as rotor moves
            setPhaseVoltage(total_voltage, 0.0f, actual_angle_elec);

            last_update_time = current_time;
        }

        // Yield to interrupts without blocking
        __WFI();  // Wait For Interrupt (low power)
    }

    // Signal that kickstart just completed (for ramped_speed initialization)
    g_kickstart_just_completed = 10.0;
   // startup_begin_time = micros();

    // SMOOTH HANDOFF: The last voltage command used g_cached_encoder_angle
    // When closed-loop takes over, it will also use g_cached_encoder_angle
    // Result: NO ANGLE DISCONTINUITY at transition!

}






#if 0
//-----------------------------------------------------------------------------
//                          absoluteZeroSearch
//
// Encoder alignment the absolute zero angle - to the index pin
//
// return true if search is complete
//
// Just return since we don't have an index pin
//-----------------------------------------------------------------------------
bool StepperMotor::absoluteZeroSearch()
{
    // Push limits
    float limit_vel    = m_velocity_limit;
    float limit_volt   = m_voltage_limit;

    m_velocity_limit   = m_velocity_index_search;
    m_voltage_limit    = m_voltage_sensor_align;

    // search the absolute zero with small velocity
    m_shaft_angle      = 0.0f;

    while(needsSearch() && m_shaft_angle < TWO_PI)
    {
      angleOpenloop(THREE_PI);

      // update is important for some sensors not to loose count
      // not needed for the search
      //m_sensor.update();
    }

    // disable motor
    setPhaseVoltage(0.0f, 0.0f, 0.0f); // TODO: refactor name

    // Pop limits
    m_velocity_limit = limit_vel;
    m_voltage_limit  = limit_volt;

    return !needsSearch();
}
#endif

//-----------------------------------------------------------------------------
//                read_angle_radians_from_buffer_with_offset
//-----------------------------------------------------------------------------
float StepperMotor::read_angle_radians_from_buffer_with_offset()
{
  //return m_sensor.read_angle_radians_from_buffer() - m_sensor_offset;
  return m_sensor.read_radians_with_direction() - m_sensor_offset;
}

//-----------------------------------------------------------------------------
//                               loopFOC
//
// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
//-----------------------------------------------------------------------------
float test_shaft_radians = 0.0f;

void StepperMotor::loopFOC(float winding_amperage_a, float winding_amperage_b)
{
    g_loopfoc_counter++;

    // ALWAYS update cached encoder angle (needed after kickstart)
    float mech_angle_from_buffer = m_sensor.read_angle_radians_from_buffer() - m_sensor_offset;
    float encoder_angle_from_get_electric = mechanical_to_electrical_radians(mech_angle_from_buffer) - m_radian_offset_to_electric_zero;
    encoder_angle_from_get_electric = normalize_radians(encoder_angle_from_get_electric);

    m_current_electrical_radians =  normalize_radians(encoder_angle_from_get_electric);

    //g_cached_encoder_angle = m_current_electrical_radians; // debug output

    // Get Kalman-filtered electrical angle for smooth FOC operation
    float kalman_electrical_radians = m_sensor.get_kalman_electrical_angle_radians(
        m_sensor_offset,
        m_radian_offset_to_electric_zero,
        NUM_POLE_PAIRS);

    g_cached_encoder_angle = kalman_electrical_radians; // debug output


    g_loopfoc_update_count++;


    extern volatile float g_debug_kickstart_active;
    g_debug_kickstart_active = m_kickstart_active ? 1.0f : 0.0f;  // Copy actual value

    if (m_kickstart_active)
    //if (!m_kickstart_finished)
    {
        extern volatile float g_debug_kickstart_active;
        g_debug_kickstart_active = 1.0f;

        // Still update currents for monitoring but don't command voltages
        transformCurrents( winding_amperage_a,
                           winding_amperage_b,
                           m_current_electrical_radians,
                           m_amperage.d,
                           m_amperage.q);
        return;
    }


    // Update global debug variables for monitoring
    g_winding_amps_a = winding_amperage_a;
    g_winding_amps_b = winding_amperage_b;

    const float SECONDS_PER_MICROSECOND( 0.000001f); // TODO move to class level
    const float MICROSECONDS_PER_FRAME (static_cast<float>(MICROSECONDS_PER_ITERATION));
    const float DELTA_T_SECONDS(MICROSECONDS_PER_FRAME * SECONDS_PER_MICROSECOND);



#if 0  // Enable sensor read for closed-loop operation
    if(m_sensor.fetch_radians(shaft_radians))
    {
         // divide offset by range of 5 volts, then take half
         //float current_offset_a = m_current_offset_a.update(winding_amperage_a);
         //float current_offset_b = m_current_offset_b.update(winding_amperage_b);


         // FIXED: Remove 0.5× scaling - current values are already correct
         // Current filtering is already applied in update_amperage()

         // Transform phase currents to dq frame using sensor angle
         transformCurrents( winding_amperage_a,
                            winding_amperage_b,
                            shaft_radians,
                            m_amperage.d,
                            m_amperage.q);

         test_shaft_radians = shaft_radians;

    }
#else  // SKIP sensor read for open-loop operation - SPI timing causes jitter
   // FIXED: Remove 0.5× scaling - current values are already correct from update_amperage()
   // FIXED: Use actual electrical angle for proper dq transformation

   // CRITICAL: Cache encoder angle at start of control cycle
   // This same angle will be used for both Park (current) and Inverse Park (voltage)
   // to ensure perfect synchronization and eliminate phase mismatch

   //float encoder_angle_from_get_electric = get_electric_angle_radians();
   // NON-BLOCKING: Read directly from DMA buffer instead of blocking SPI call
   // This prevents the 3-second SPI lockup seen in Saleae capture
   //float mech_angle_from_buffer = m_sensor.read_angle_radians_from_buffer() - m_sensor_offset;
   //float encoder_angle_from_get_electric = mechanical_to_electrical_radians(mech_angle_from_buffer) - m_radian_offset_to_electric_zero;
   encoder_angle_from_get_electric = normalize_radians(encoder_angle_from_get_electric);

   // Phase advance compensation for processing delays
   // Total delay: SPI read (~25μs) + decimation (0-100μs) + computation (~10-20μs)
   // Estimate: ~50-75μs typical delay from encoder sample to PWM output
   const float PROCESSING_DELAY_SECONDS = 75e-6f;  // 75 microseconds

   // Get current velocity from Kalman filter (mechanical rad/s)
   float mechanical_velocity = m_sensor.get_mechanical_velocity_rad_per_sec();

   // Calculate phase advance in electrical radians
   float mechanical_phase_advance = mechanical_velocity * PROCESSING_DELAY_SECONDS;
   float electrical_phase_advance = mechanical_to_electrical_radians(mechanical_phase_advance);

   // Apply phase advance to compensate for processing delay
   //float current_electrical_angle = m_current_electrical_radians + electrical_phase_advance;
   float current_electrical_angle = kalman_electrical_radians + electrical_phase_advance;
   current_electrical_angle = _normalizeAngle(current_electrical_angle);

   // Debug: expose phase advance for monitoring
   extern volatile float g_phase_advance_electrical;
   extern volatile float g_phase_advance_degrees;
   g_phase_advance_electrical = electrical_phase_advance;
   g_phase_advance_degrees = electrical_phase_advance * (180.0f / M_PI);

   // Transform phase currents to dq frame using actual motor angle
    transformCurrents( winding_amperage_a,
                       winding_amperage_b,
                       kalman_electrical_radians, //m_current_electrical_radians,  // remove phase advance current_electrical_angle,
                       m_amperage.d,
                       m_amperage.q);

    // Update debug variables for current monitoring
    g_current_a_raw = winding_amperage_a;
    g_current_b_raw = winding_amperage_b;
    g_current_electrical_angle = current_electrical_angle;
    g_angle_used_in_loopFOC = current_electrical_angle;  // Debug: angle for Park transform
    g_amperage_d = m_amperage.d;
    g_amperage_q = m_amperage.q;
#endif

    // Continue with motion control even if sensor read fails
    switch (m_motion_control)
      {
        case MOTION_CONTROL_TYPE::TORQUE:

             break;

        case MOTION_CONTROL_TYPE::CL_ANGLE:

             update_position_closed_loop(m_target, DELTA_T_SECONDS);

             break;


        case MOTION_CONTROL_TYPE::CL_VELOCITY:

            extern volatile float g_debug_kickstart_active;
            g_debug_kickstart_active = 0.0f;

             // Use m_target as set by update_target_rad_per_sec() or move()
             // No longer hardcoding target speed - respect user's target
             g_m_target_debug = m_target;  // Capture m_target before calling update_speed_closed_loop

             // DEBUG: Track m_enabled status and stop reasons
             //extern volatile float g_motor_stop_reason;
             extern volatile float g_m_enabled_status;
             g_m_enabled_status = m_enabled ? 1.0f : 0.0f;

             // DISABLED: Don't stop motor even if m_enabled is false
             // Something is setting m_enabled to false and we need to find what
             // For now, bypass the check to keep motor running
             /*
             if (!m_enabled) {
                 g_motor_stop_reason = 3.0f;  // Motor m_enabled = false
                 // Set zero voltage when disabled
                 m_voltage.q = 0.0f;
                 m_voltage.d = 0.0f;
                 setPhaseVoltage(0.0f, 0.0f, 0.0f);
                 return;  // Skip rest of processing
             }
             */

             update_speed_closed_loop(m_target,  DELTA_T_SECONDS);

             break;

        case MOTION_CONTROL_TYPE::OL_VELOCITY:

             update_speed_open_loop(m_target,  DELTA_T_SECONDS);

             break;

        case MOTION_CONTROL_TYPE::OL_ANGLE:

             update_position_open_loop(m_target,  DELTA_T_SECONDS);

             break;

        default:
             ; // NOP
    }

}

//-----------------------------------------------------------------------------
//                             move
//
// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or voltage loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
//
// voltage and current elements are in quatrature/Direct coordinate system
//-----------------------------------------------------------------------------
void StepperMotor::update_target_rad_per_sec(float rps)
{
    extern volatile float g_update_target_called;
    g_update_target_called = rps;

    m_target = rps;
}

//-----------------------------------------------------------------------------
//                                 move
//-----------------------------------------------------------------------------
void StepperMotor::move(float new_target)
{
    const float SECONDS_PER_MICROSECOND( 0.000001f); // TODO move to class level
    const float MICROSECONDS_PER_FRAME (static_cast<float>(MICROSECONDS_PER_ITERATION));
    const float DELTA_T_SECONDS(MICROSECONDS_PER_FRAME * SECONDS_PER_MICROSECOND);

    float delta_target      = new_target - m_target_prev;
    m_target_prev           = new_target;


 #if 0
    m_sensor.update();

    m_shaft_angle = get_filtered_shaft_angle();

    m_omega_mechanical_rps = shaft_radians_per_second();

    // calculate back-emf m_voltage if KV_rating available U_bemf = vel*(1/KV)
    float rpm = m_omega_mechanical_rps * RAD_PER_SEC_TO_REV_PER_MIN;
    m_voltage_bemf = rpm/KV_RPM_PER_VOLT;
#endif
    // Ohm's law
    // m_amperage appears to be set but not used
    //    m_amperage.q = (m_voltage.q - m_voltage_bemf) / resistance();

    // choose control loop
    switch (m_motion_control)
    {
      case MOTION_CONTROL_TYPE::TORQUE:

           // In this context, the new_target has units of volts
           // The quadrature voltage controls the torque

           update_torque_open_loop(new_target, DELTA_T_SECONDS, delta_target);

           break;


      case MOTION_CONTROL_TYPE::CL_ANGLE:
      {

           m_target = new_target;
      }
           break;


      case MOTION_CONTROL_TYPE::CL_VELOCITY:
      {

          // m_target = new_target;

           m_amperage_prev.d = m_amperage.d; // = 0.0f;
           m_amperage_prev.q = m_amperage.q; // = 0.0f;
           m_voltage_prev.d  = m_voltage.d; //  = 0.0f;
           m_voltage_prev.q  = m_voltage.q; //  = 0.0f;
      }
           break;

      case MOTION_CONTROL_TYPE::OL_VELOCITY:
      {
           m_target = new_target;
#if 0
           const float DELTA_T(0.01f);
           float expected_change = m_target * DELTA_T;
           float angle_cmd = m_sensor.get_angle_radians() + expected_change;
           float norm_cmd  = normalize_radians(angle_cmd); // not converting to electrical yet

           float mag_flux_linkage_q            = PHASE_INDUCTANCE * m_amperage.q;
           float back_emf_q_axis               = m_omega_mechanical_rps
                                               * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

           float voltage_q = m_current_limit * resistance()
                           + fabs(back_emf_q_axis);

           m_voltage.q     = symetric_clamp(voltage_q, m_voltage_limit);
           m_voltage.d     = 0.0f;

           setPhaseVoltage(m_voltage.q, m_voltage.d, norm_cmd);

           m_amperage.q = (m_voltage.q - fabs(back_emf_q_axis))/resistance();
 #endif
      }
           break;

      case MOTION_CONTROL_TYPE::OL_ANGLE:
      {
           m_target = new_target;
      }
           break;

      default:
           ; // NOP
  }
}

//-----------------------------------------------------------------------------
//                        update_torque_open_loop
//-----------------------------------------------------------------------------
void StepperMotor::update_torque_open_loop(
                                             float target_quadrature_voltage,
                                             float delta_t,
                                             float delta_target)
{

      // divide by zero guard
      float delta_I_d = (fabs(delta_t) > 0.0f)
                      ? delta_target/delta_t
                      : 0.0f;

      float electrical_rps = mechanical_to_electrical_radians(m_omega_mechanical_rps);

      float voltage_q =  target_quadrature_voltage * resistance(0.0f)
                      +  inductance(electrical_rps) * delta_I_d
                      +  electrical_rps * m_voltage_bemf;

      m_voltage.q = symetric_clamp( voltage_q,
                                    m_voltage_limit);

      // Assume current_d = 0
      // current_q comes from the pid this is the target and affects torque
      //
      //  voltage_d =  phase_resistance * target_Current_d
      //            +  phase_inductance * delta_current_d/delta_t
      //            -  omega * phase_inductance * current_q
      //
      // with current_d = 0, then delta_current_d/delta_t = 0
      // current_q = target
      //
      //  voltage_d =  -  omega * phase_inductance * target

      float voltage_d =  -inductance(electrical_rps)
                      *   mechanical_to_electrical_radians(m_omega_mechanical_rps)
                      *   m_target;                                // Amperes

      m_voltage.d = symetric_clamp( voltage_d,
                                    m_voltage_limit);

     setPhaseVoltage(m_voltage.q, m_voltage.d, get_electric_angle_radians());

}


// #if 0 //backup
// /-----------------------------------------------------------------------------
// //                          update_speed_closed_loop
// //
// // This implements the torque control loop. As the stepper motors only support
// // torque using voltage mode.
// //
// // Read the current motor angle from the sensor, turn it into the electrical
// // angle and transforms the q-axis Uq voltage command motor.voltage_q
// //-----------------------------------------------------------------------------
// void backup StepperMotor::update_speed_closed_loop(
//                                                 float target_mechanical_rps,
//                                                 float delta_t)
// {
//     static float rad(0.0f);
//     rad += target_mechanical_rps*delta_t;

//     float electric_radians = normalize_radians(rad);



//     float angular_velocity_error(target_mechanical_rps - shaft_radians_per_second());

//     float current_sp = m_PID_velocity.update(angular_velocity_error);
//     float correction_term(0.0f);

//     float electrical_rps           = mechanical_to_electrical_radians(target_mechanical_rps);
//     float mag_flux_linkage_q       = PHASE_INDUCTANCE * m_amperage.q;
//     float back_emf_q_axis          = electrical_rps
//                                    * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

//      m_voltage.q =  current_sp * resistance() + back_emf_q_axis;
//      m_voltage.d = -current_sp * inductance() * electrical_rps;

//      m_voltage.q = symetric_clamp( m_voltage.q, m_voltage_limit);
//      m_voltage.d = symetric_clamp( m_voltage.d, m_voltage_limit);


//     setPhaseVoltage(m_voltage.q, m_voltage.d, get_electric_angle_radians());


//     m_amperage.d = m_amperage_prev.d
//                 + ( m_voltage.d -  m_voltage_prev.d ) * delta_t * PHASE_INDUCTANCE_INVERSE;

//     m_amperage.q = m_amperage_prev.q
//                 + ( m_voltage.q -  m_voltage_prev.q ) * delta_t * PHASE_INDUCTANCE_INVERSE;


//     float next_position = m_sensor.read_angle_radians();
//                         + target_mechanical_rps*delta_t
//                         + correction_term;


//     setPhaseVoltage(m_voltage.q, m_voltage.d, normalize_radians(next_position));

//     m_amperage_prev.d = m_amperage.d;
//     m_amperage_prev.q = m_amperage.q;
//     m_voltage_prev.d  = m_voltage.d;
//     m_voltage_prev.q  = m_voltage.q;

// }
// #endif


//-----------------------------------------------------------------------------
//                              sample_as5048_25us
//
// Caution: This is invoked from within an interrupt context
//-----------------------------------------------------------------------------
void StepperMotor::sample_as5048_25us()
{
      m_sensor.async_read_angle();
}


// #if 0
// uint32_t StepperMotor::compute_time_difference(uint32_t current_time, uint32_t previous_time)
// {
//     const uint32_t MAX_UINT32 (0xFFFFFFFFU);

//     if (current_time >= previous_time) {
//         // No rollover, normal subtraction
//         return current_time - previous_time;
//     } else {
//         // Rollover occurred, adjust the calculation
//         return (MAX_UINT32 - previous_time + current_time + 1);
//     }
// }
// #endif


//-----------------------------------------------------------------------------
//                              control_loop_25us
//
// Dependancies:
//
//
//-----------------------------------------------------------------------------
void StepperMotor::control_loop_25us()
{
    // For velocity control modes, call loopFOC at 10kHz (decimate 40kHz to 10kHz)
    if (m_motion_control == MOTION_CONTROL_TYPE::CL_VELOCITY ||
        m_motion_control == MOTION_CONTROL_TYPE::OL_VELOCITY)
    {
        static uint8_t decimation_counter = 0;
        decimation_counter++;

        if (decimation_counter >= 4)  // 40kHz / 4 = 10kHz
        {
            decimation_counter = 0;

            // Call FOC iteration from here for hardware-timed deterministic control
            winding_currents result = update_amperage();
            loopFOC(result.winding_amperage_a, result.winding_amperage_b);

            // Update velocity ramp
            extern void update_ramp(void);

            if(m_kickstart_finished)
            {
                update_ramp();
            }

        }
        return;
    }

    // Position control modes continue below
    static ElapsedTime elapsed_time; // microseconds

    float curr_mech_rad = read_angle_radians_from_buffer_with_offset();
    float curr_elec_rad = mechanical_to_electrical_radians(curr_mech_rad);


    // Ensure delta_seconds is consistent and clamp it to avoid extreme values
    // fairly consistent ~50 us or 0.00005 seconds
    //float delta_seconds = std::max(
    //                           0.00001f,
    //                           std::min(0.001f, elapsed_time.get() / 1000000.0f));

    float delta_mech_rad = m_commanded_speed_radians_per_sec * elapsed_time.get()* 0.000001f;
    float delta_elec_rad = mechanical_to_electrical_radians(delta_mech_rad);

    float cmd_elec_rad = curr_elec_rad + delta_elec_rad;

    g_voltage_q = m_voltage.q;
    g_voltage_d = m_voltage.d;
    g_target_elec_rad = cmd_elec_rad;



    setPhaseVoltage(m_voltage.q, m_voltage.d, cmd_elec_rad);

}







//-----------------------------------------------------------------------------
//                              calculate_velocity
//-----------------------------------------------------------------------------
float StepperMotor::calculate_velocity(float current_angle, float delta_seconds)
{
    static float previous_angle = 0.0f;

    //float delta_angle = current_angle - previous_angle;
    double delta_angle = static_cast<double>(current_angle) - static_cast<double>(previous_angle);

    if(fabs(delta_angle) > M_PI)
    {
        // Rollover
        if(current_angle < previous_angle)
        {
           delta_angle += 2.0f * M_PI;
        }
        else
        {
            delta_angle -= 2.0f * M_PI;
        }
    }

    //float velocity = delta_angle / delta_seconds;
    double velocity = delta_angle / static_cast<double>(delta_seconds);

    previous_angle = current_angle;

    g_delta_angle = delta_angle;

    return velocity;
}








//----------------------------------








//-----------------------------------------------------------------------------
//                            update_position_closed_loop
//-----------------------------------------------------------------------------
void StepperMotor::update_position_closed_loop(
                                                      float target_mech_angle_radians,
                                                      float delta_t)
{


    //float angular_error = target_mech_angle_radians - m_sensor.get_angle_radians();

    //float v_pid = m_PID_angle.update(angular_error);

    float amperage_d_dot(0.0f);
    //float amperage_q_dot(0.0f);

    if(delta_t > 0.0f)
    {
        amperage_d_dot = (m_amperage.d - m_amperage_prev.d)/delta_t;
       // amperage_q_dot = (m_amperage.q - m_amperage_prev.q)/delta_t;
    }

    // Flux linkages have units of volt seconds
    //float mag_flux_linkage_d            = inductance(0.0f) * m_amperage.d;
    float mag_flux_linkage_q            = inductance(0.0f) * m_amperage.q;


    //float resistive_voltage_drop_d_axis = resistance(0.0f) * m_amperage.d;
    float resistive_voltage_drop_q_axis = resistance(0.0f) * m_amperage.q;


    float inductive_voltage_drop_d_axis = inductance(0.0f) * amperage_d_dot;
    //float inductive_voltage_drop_q_axis = inductance(0.0f) * amperage_q_dot;

    float electrical_rps                = mechanical_to_electrical_radians(m_omega_mechanical_rps);
    //float back_emf_d_axis               = electrical_rps * mag_flux_linkage_d;
    float back_emf_q_axis               = electrical_rps
                                        * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);


#if 0
    // from basic principles
    float voltage_d                     = resistive_voltage_drop_d_axis
                                        + inductive_voltage_drop_d_axis
                                        - back_emf_d_axis;


     float voltage_q                    = resistive_voltage_drop_q_axis
                                        + inductive_voltage_drop_q_axis
                                        - back_emf_q_axis
                                        + v_pid;
#else
    // from original
    float voltage_d                     = -electrical_rps*inductive_voltage_drop_d_axis;


    float voltage_q                     = resistive_voltage_drop_q_axis
                                        + back_emf_q_axis;

#endif

     m_voltage.d = symetric_clamp( voltage_d, m_voltage_limit);
     m_voltage.q = symetric_clamp( voltage_q, m_voltage_limit);

//     m_amperage.d = m_amperage_prev.d
//                 + ( m_voltage.d -  m_voltage_prev.d ) * delta_t / inductance();

//     m_amperage.q = m_amperage_prev.q
//                 + ( m_voltage.q -  m_voltage_prev.q ) * delta_t / inductance();

     float target_angle_electric =  mechanical_to_electrical_radians(target_mech_angle_radians);
     setPhaseVoltage(m_voltage.q, m_voltage.d, target_angle_electric);

    // update state variables
    m_amperage_prev.d = m_amperage.d;
    m_amperage_prev.q = m_amperage.q;
    m_voltage_prev.d  = m_voltage.d;
    m_voltage_prev.q  = m_voltage.q;
}

//-----------------------------------------------------------------------------
//                         update_position_open_loop
//-----------------------------------------------------------------------------
void StepperMotor::update_position_open_loop(
                                                   float target_mechanical_radians,
                                                   float delta_t)
{
    const float MAX_RESISTIVE_VOLTAGE_DROP(m_current_limit*resistance(0.0f));

    // quick fix for strange cases (micros overflow + timestamp not defined)
    if(delta_t <= 0 || delta_t > 0.5f)
    {
      delta_t = 1e-3f;
    }

    // calculate the necessary angle to move from m_amperage position towards
    // m_target angle with maximal velocity (m_velocity_limit)

    // Note: m_velocity_limit may have been modified by absoluteZeroSearch()

    // float target_electrical_radians = mechanical_to_electrical_radians(target_mechanical_radians);

    float current_angle;
    if(!m_sensor.fetch_radians(current_angle))
    {
        return;
    }

    float target_angle_error  = target_mechanical_radians -  current_angle;
    float max_angular_change  = m_velocity_limit * delta_t;

    float shaft_angle_command;

    if(!m_sensor.fetch_radians(shaft_angle_command))
    {
        return;
    }

    if( fabs( target_angle_error ) > fabs( max_angular_change ))
    {
      shaft_angle_command += _sign(target_angle_error) * max_angular_change;
    }
    else
    {
      shaft_angle_command = target_mechanical_radians;
    }

    m_voltage.d = 0; // TODO d-component lag-compensation

    // Flux linkages have units of volt seconds
    float mag_flux_linkage_q = inductance(0.0f) * m_amperage.q;

    float electrical_rps  = mechanical_to_electrical_radians(m_omega_mechanical_rps);
    float back_emf_q_axis = electrical_rps
                          * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

    float voltage_q  = MAX_RESISTIVE_VOLTAGE_DROP + fabs(back_emf_q_axis);
    m_voltage.q = symetric_clamp(voltage_q, m_voltage_limit);

//    m_amperage.q = (m_voltage.q  - fabs(back_emf_q_axis)) / resistance(0.0f);

    float raw_elec_angle(mechanical_to_electrical_radians(shaft_angle_command));


//    m_amperage.d = m_amperage_prev.d
//                + ( m_voltage.d -  m_voltage_prev.d ) * delta_t / inductance(0.0f);

//    m_amperage.q = m_amperage_prev.q
 //               + ( m_voltage.q -  m_voltage_prev.q ) * delta_t / inductance(0.0f);

    raw_elec_angle -= m_radian_offset_to_electric_zero;
    setPhaseVoltage(m_voltage.q, m_voltage.d, normalize_radians(raw_elec_angle));

    m_amperage_prev.d = m_amperage.d;
    m_amperage_prev.q = m_amperage.q;
    m_voltage_prev.d  = m_voltage.d;
    m_voltage_prev.q  = m_voltage.q;

    return;
}

//-----------------------------------------------------------------------------
//                                 update_speed_open_loop
//-----------------------------------------------------------------------------
void StepperMotor::update_speed_open_loop(
                                               float target_mechanical_rps,
                                               float delta_t)
{
     if(FP_ZERO == fpclassify(target_mechanical_rps))
     {
       m_voltage.q  = 0.0f;
       m_voltage.d  = 0.0f;
       setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);  // <-- This causes angle==0!

       return;
     }

     float target_electrical_rps = mechanical_to_electrical_radians(target_mechanical_rps);

     static float mechanical_radians(0.0f);
     mechanical_radians += target_mechanical_rps*delta_t;

     mechanical_radians= normalize_radians(mechanical_radians);

    // Flux linkages have units of volt seconds
    // float mag_flux_linkage_d            = inductance(target_electrical_rps) * m_amperage.d;
    float mag_flux_linkage_q            = inductance(target_electrical_rps) * m_amperage.q;

    // float resistive_voltage_drop_d_axis = resistance(target_electrical_rps) * m_amperage.d;
    // float resistive_voltage_drop_q_axis = resistance(target_electrical_rps)  * m_amperage.q;

    // float inductive_voltage_drop_d_axis = inductance(target_electrical_rps) * amperage_d_dot;
    // float inductive_voltage_drop_q_axis = inductance(target_electrical_rps) * amperage_q_dot;

    float back_emf_q_axis  = m_omega_mechanical_rps
                           * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

    // float back_emf_d_axis  = m_omega_mechanical_rps
    //                        * (mag_flux_linkage_d + PERM_MAGNET_FLUX_LINKAGE);

    // Use DC resistance for steady-state I×R drop (DQ current is DC)
    float voltage_q = m_current_limit * PHASE_RESISTANCE
                    + fabs(back_emf_q_axis);

    m_voltage.q     = symetric_clamp(voltage_q, m_voltage_limit);

    m_voltage.d     = 0.0f;
    setPhaseVoltage(m_voltage.q, m_voltage.d, mechanical_to_electrical_radians(mechanical_radians));

    return;
}

//-------------------------------------------
// #if 0
// // refactored on 12/2/2025
// void StepperMotor::update_speed_closed_loop(float target_mechanical_rps,
//                                             float delta_seconds)
// {
//     // DEBUG: Track if this function is being called and with what target
//     g_update_speed_cl_call_count++;
//     g_target_rps_to_cl_controller = target_mechanical_rps;
//     g_delta_seconds_to_cl_controller = delta_seconds;

//     // DEBUG: Track ramped_speed vs epsilon to diagnose why angle becomes 0 so often

// #if 1  // DEBUG: Smooth ramp for open-loop startup testing only
// // Smooth ramp-up to target speed
// // CRITICAL: Initialize above epsilon threshold (1.0e-5) to avoid repeated angle==0 triggers during startup
// // The ramping increment is ACCEL_RATE * delta_seconds = 1.0 * 0.0001 = 0.0001 rad/s per iteration
// // Starting at 0.0 would cause ramped_speed to hover around epsilon for many iterations,
// // triggering setPhaseVoltage(0,0,0) repeatedly and causing spikes in g_park_sin/cos
// static float ramped_speed = 0.001f;  // Start at 0.001 rad/s (well above epsilon threshold)
// #if 0  // Ultra-gentle acceleration for startup synchronization
//     const float ACCEL_RATE = 0.25f; // rad/s² - extremely slow to allow rotor to follow
// #else  // Faster acceleration (after motor is spinning)
//     //const float ACCEL_RATE = 1.0f; // rad/s² - gentle acceleration
//     const float ACCEL_RATE = 1.0f; // rad/s² - gentle acceleration
// #endif

//     // Ramp toward target
//     if (ramped_speed < target_mechanical_rps)
//     {
//         ramped_speed += ACCEL_RATE * delta_seconds;
//         if (ramped_speed > target_mechanical_rps)
//             ramped_speed = target_mechanical_rps;
//     }
//     else if (ramped_speed > target_mechanical_rps)
//     {
//         ramped_speed -= ACCEL_RATE * delta_seconds;
//         if (ramped_speed < target_mechanical_rps)
//             ramped_speed = target_mechanical_rps;
//     }

//     // Use ramped speed instead of direct target
//     const float actual_target_rps = ramped_speed;
// #else
//     // Normal operation - use target directly
//     const float actual_target_rps = target_mechanical_rps;
// #endif

//     // Treat very small targets as "stop"
//     const float EPSILON_SPEED_RPS(1.0e-4f);

//     // DEBUG: Always capture these values to see the relationship
//     g_debug_ramped_speed = ramped_speed;
//     g_debug_actual_target_rps = actual_target_rps;

//     g_actual_target_rps_debug = actual_target_rps;  // Capture actual_target_rps for diagnostics

//     // EPSILON CHECK DISABLED - Motor should run continuously
//     // This was causing the motor to stop when target was small
//     /*
//     //if (fabsf(actual_target_rps) < EPSILON_SPEED_RPS)
//     if (fabsf(actual_target_rps) < EPSILON_SPEED_RPS && fabsf(measured_mechanical_rad_per_sec) < 0.01f)
//     {
//         // DEBUG: Track how often we set angle to 0 due to very small target speed
//         // This is another source of the many angle==0 conditions that cause spikes
//         // THE PROBLEM: ramped_speed starts at 0 and increments by ACCEL_RATE*delta_seconds
//         // With ACCEL_RATE=1.0 and delta_seconds=0.0001, increment = 0.0001 rad/s
//         // This is EXACTLY the epsilon threshold (1.0e-4), so ramped_speed hits this
//         // condition repeatedly during startup, causing many angle==0 spikes!
//         g_debug_epsilon_trigger_count++;

//         m_voltage.q = 0.0f;
//         m_voltage.d = 0.0f;

//         // Park the stator field at 0 electrical when stopped
//         setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);  // <-- This causes angle==0!

//         g_target_rad_per_sec = actual_target_rps;

//         return;
//     }
//     */

//     // Debug incoming parameters FIRST
//     g_ramped_target_rps = ramped_speed; // Show actual ramped speed being used
//     g_target_rad_per_sec = target_mechanical_rps; // Show commanded target speed

//     if (delta_seconds <= 0.0f)
//     {
//         return;
//     }

//     // ------------------------------------------------------------------------
//     // 1) Use ramped speed for smooth acceleration
//     // ------------------------------------------------------------------------
//     const float target_mechanical_rad_per_sec = actual_target_rps;

//     // Update debug variables
//     g_loop_counter++;
//     g_speed_update_counter++;

//     // ------------------------------------------------------------------------
//     // 2) Target and measured mechanical speed (radians/sec)
//     // ------------------------------------------------------------------------

//     const float current_mechanical_radians =
//         m_sensor.get_mechanical_phase_angle_radians();

//     // Debug: capture sensor angle being used and track if it's updating
//     g_sensor_mech_angle_for_cl = current_mechanical_radians;
//     g_sensor_mechanical_phase_angle = current_mechanical_radians;
//     g_sensor_raw_angle_radians = m_sensor.read_angle_radians();
//     g_sensor_read_count++;

// #if 1  // Closed-loop with Kalman filter velocity feedback - ENABLED!
//     // Use Kalman filter velocity estimate (much better than simple differentiation)
//     // UNITS: Kalman velocity is in MECHANICAL rad/s (not electrical)
//     float measured_mechanical_rad_per_sec = m_sensor.get_mechanical_velocity_rad_per_sec();

//     // Debug: capture raw velocity from Kalman filter BEFORE any processing
//     g_velocity_before_filter = measured_mechanical_rad_per_sec;
//     g_velocity_spike_reject_active = 0.0f;  // No spike rejection active

//     // NO FILTERING - Use Kalman velocity directly!
//     // The Kalman filter already provides optimal filtering
//     // Any additional filtering causes control loop to use stale/wrong velocity

//     // Only sanity check: clamp physically impossible velocities (>150 rad/s)
//     if (fabs(measured_mechanical_rad_per_sec) > 150.0f)
//     {
//         // Velocity over 150 rad/s is unrealistic for this motor - clamp it
//         measured_mechanical_rad_per_sec = fmaxf(fminf(measured_mechanical_rad_per_sec, 150.0f), -150.0f);
//         g_velocity_spike_reject_active = 1.0f;  // Indicate clamping occurred
//     }

//     g_velocity_after_clamp = measured_mechanical_rad_per_sec;

//     // Debug: capture the velocity we're using for control (should match Kalman!)
//     // UNITS: mechanical rad/s (same as g_as5048_velocity from sensor)
//     g_measured_velocity_for_control = measured_mechanical_rad_per_sec;

//     // ------------------------------------------------------------------------
//     // 3) Compute feedforward + feedback voltage
//     // ------------------------------------------------------------------------

//     // POST-KICKSTART BOOST: Use target velocity for feedforward immediately after kickstart
//     // Problem: After kickstart, measured velocity is ~0, so feedforward calculates ~0 voltage
//     // Solution: Use target velocity for a few seconds after startup to maintain motion
//     static bool post_kickstart_mode = true;
//     static uint32_t kickstart_complete_time = 0;

//     if (post_kickstart_mode)
//     {
//         // Check if motor is moving and some time has passed
//         if (fabs(measured_mechanical_rad_per_sec) > 2.0f ||
//             (kickstart_complete_time > 0 && (micros() - kickstart_complete_time) > 3000000))  // 3 seconds
//         {
//             post_kickstart_mode = false;  // Transition to normal mode
//         }
//         else if (kickstart_complete_time == 0)
//         {
//             kickstart_complete_time = micros();  // Mark kickstart complete time
//         }
//     }

//     // CRITICAL: Separate back-EMF (use MEASURED velocity) from feedforward (use TARGET velocity)
//     // Back-EMF calculation MUST use actual motor speed to be accurate
//     // If we use target velocity (80 rad/s = 4000 rad/s elec), back-EMF will be 100V and saturate immediately!

//     // Back-EMF uses MEASURED velocity (actual motor speed)
//     const float actual_electrical_rad_per_sec = mechanical_to_electrical_radians(measured_mechanical_rad_per_sec);
//     const float inductance_value = inductance(actual_electrical_rad_per_sec);
//     const float mag_flux_linkage_q = inductance_value * m_amperage.q;
//     const float back_emf_q_axis = actual_electrical_rad_per_sec
//                                 * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

//     // Feedforward uses TARGET velocity for aggressive acceleration
//     const float target_electrical_rad_per_sec = mechanical_to_electrical_radians(target_mechanical_rad_per_sec);

//     // DEBUG: Add variables to diagnose back-EMF calculation
//     extern volatile float g_debug_inductance_value;
//     extern volatile float g_debug_mag_flux_linkage_q;
//     extern volatile float g_debug_back_emf_q_axis;
//     extern volatile float g_debug_perm_magnet_flux;
//     extern volatile float g_debug_measured_elec_rad_per_sec;
//     extern volatile float g_debug_measured_mech_rad_per_sec;
//     extern volatile float g_debug_amperage_q_for_bemf;

//     g_debug_inductance_value = inductance_value;
//     g_debug_mag_flux_linkage_q = mag_flux_linkage_q;
//     g_debug_back_emf_q_axis = back_emf_q_axis;
//     g_debug_perm_magnet_flux = PERM_MAGNET_FLUX_LINKAGE;
//     g_debug_measured_elec_rad_per_sec = actual_electrical_rad_per_sec;
//     g_debug_measured_mech_rad_per_sec = measured_mechanical_rad_per_sec;
//     g_debug_amperage_q_for_bemf = m_amperage.q;

//     // CRITICAL: Use ACTUAL MEASURED CURRENT for feedforward (not fixed 2A assumption!)
//     // For unloaded motor: I_q ≈ 0.3A (friction/windage only)
//     // For loaded motor: I_q increases based on load torque
//     // Using measured I_q gives accurate feedforward for any load condition
//     float measured_current_q = fabs(m_amperage.q);

//     // Clamp to reasonable range (0.1A to current limit)
//     // Minimum 0.1A prevents division-by-zero and ensures minimum voltage for startup
//     const float MIN_FEEDFORWARD_CURRENT = 0.1f;
//     const float MAX_FEEDFORWARD_CURRENT = m_current_limit;
//     measured_current_q = fmaxf(fminf(measured_current_q, MAX_FEEDFORWARD_CURRENT), MIN_FEEDFORWARD_CURRENT);

//     // Feedforward voltage = resistive drop + back-EMF
//     // Use DC resistance for steady-state I×R drop (DQ current is DC)
//     float base_voltage_q = measured_current_q * PHASE_RESISTANCE
//                          + fabs(back_emf_q_axis);

//     // AGGRESSIVE VOLTAGE BOOST: Motor needs MORE voltage as it accelerates, not less
//     // Scale voltage with target speed to provide torque for acceleration
//     const float BASE_MIN_VOLTAGE = 2.0f;
//     const float VOLTAGE_PER_RAD_S = 0.3f;  // Add voltage as target speed increases

//     float required_voltage = BASE_MIN_VOLTAGE + (target_mechanical_rad_per_sec * VOLTAGE_PER_RAD_S);

//     if (base_voltage_q < required_voltage && target_mechanical_rad_per_sec > 0.05f)
//     {
//         base_voltage_q = required_voltage;
//     }

//     // Feedback correction (mechanical domain) - PI CONTROLLER
//     // TUNING GUIDE:
//     //   Kp controls response speed (higher = faster, but may overshoot)
//     //   Ki eliminates steady-state error (higher = faster convergence, but may oscillate)
//     //
//     //   Current tuning (for 20V motor, 25 rad/s max):
//     //     Kp = 1.5  (3× baseline, good transient response)
//     //     Ki = 0.5  (moderate integration, eliminates error in ~5s)
//     //
//     //   If motor oscillates: reduce Kp to 1.0, reduce Ki to 0.3
//     //   If response too slow: increase Kp to 2.0, keep Ki at 0.5
//     //   If steady-state error remains: increase Ki to 0.8
//     const float Kp_velocity_mech = 1.5f;   // Proportional gain (increased for faster response)
//     const float Ki_velocity_mech = 0.5f;   // Integral gain (eliminate steady-state error)
//     const float velocity_error_mech_rad_per_sec =
//         target_mechanical_rad_per_sec - measured_mechanical_rad_per_sec;

//     // Integral accumulator (static to maintain state between calls)
//     static float velocity_error_integral = 0.0f;

//     // Integration with anti-windup
//     // Note: velocity_error_integral is unitless accumulator (rad/s × seconds = radians)
//     // Ki converts it to voltage: V = Ki × integral
//     const float INTEGRAL_MAX = 30.0f;  // Limit integral accumulator to ±30 rad
//     velocity_error_integral += velocity_error_mech_rad_per_sec * 0.0001f;  // dt = 100us (10kHz)
//     velocity_error_integral = fmaxf(fminf(velocity_error_integral, INTEGRAL_MAX), -INTEGRAL_MAX);

//     // Calculate PI feedback correction voltage
//     float proportional_term = Kp_velocity_mech * velocity_error_mech_rad_per_sec;
//     float integral_term = Ki_velocity_mech * velocity_error_integral;
//     float feedback_correction = proportional_term + integral_term;

//     // ANTI-WINDUP: Prevent negative feedback from fighting startup
//     // During startup (low measured velocity), only allow positive feedback correction
//     // This prevents the controller from reducing voltage when motor is trying to start
//     if (measured_mechanical_rad_per_sec < 1.0f && feedback_correction < 0.0f)
//     {
//         feedback_correction = 0.0f;  // No negative correction during startup
//     }

//     // SATURATION PROTECTION: Limit feedback magnitude to prevent instability
//     const float MAX_FEEDBACK_CORRECTION = 18.0f;  // Increased limit for better tracking (was 10V)
//     float feedback_correction_clamped = fmaxf(fminf(feedback_correction, MAX_FEEDBACK_CORRECTION), -MAX_FEEDBACK_CORRECTION);

//     // Anti-windup: If feedback is saturated, stop integrating in that direction
//     if ((feedback_correction > MAX_FEEDBACK_CORRECTION && velocity_error_mech_rad_per_sec > 0.0f) ||
//         (feedback_correction < -MAX_FEEDBACK_CORRECTION && velocity_error_mech_rad_per_sec < 0.0f))
//     {
//         // Back off the integrator slightly to prevent windup
//         velocity_error_integral *= 0.99f;
//     }

//     // Reset integrator if motor is disabled or at target (prevent windup when stopped)
//     if (!m_enabled || (fabs(velocity_error_mech_rad_per_sec) < 0.05f && fabs(target_mechanical_rad_per_sec) < 0.01f))
//     {
//         velocity_error_integral = 0.0f;
//     }

//     // Use clamped feedback for control
//     feedback_correction = feedback_correction_clamped;

//     // Debug: capture all control components
//     g_velocity_error_for_debug = velocity_error_mech_rad_per_sec;
//     g_base_voltage_q_feedforward = base_voltage_q;
//     g_feedback_voltage_correction = feedback_correction;  // After clamping
//     g_velocity_error_integral = velocity_error_integral;  // Accumulator (rad)
//     g_velocity_correction_p = proportional_term;  // Before clamping (V)
//     g_velocity_correction_i = integral_term;  // Before clamping (V)
//     g_feedforward_current_q = measured_current_q;  // Measured I_q used for feedforward

//     float desired_voltage_q = base_voltage_q + feedback_correction;
//     g_desired_voltage_q_before_clamp = desired_voltage_q;

//     // Clamp to available DC bus / configured limit
//     m_voltage.q = symetric_clamp(desired_voltage_q, m_voltage_limit);
//     m_voltage.d = 0.0f; // no field weakening for now

//     // Debug: Capture m_voltage.q after calculation
//     g_m_voltage_q_in_controller = m_voltage.q;
// #else  // Pure open-loop (for when sensor is not working)

//     const float velocity_error_mech_rad_per_sec = 0.0;
//     // ------------------------------------------------------------------------
//     // 3) PURE OPEN-LOOP VOLTAGE (sensor broken, returns constant value)
//     // ------------------------------------------------------------------------

//     // Simple voltage calculation for open-loop operation
// #if 1  // Higher voltage for stronger torque
//     const float OPEN_LOOP_VOLTAGE = 10.0f; // Increased to overcome cogging torque
// #else  // Lower voltage for gentler operation
//     const float OPEN_LOOP_VOLTAGE = 6.0f; // Reduced voltage
// #endif

//     m_voltage.q = OPEN_LOOP_VOLTAGE;
//     m_voltage.d = 0.0f;
//     #endif

//         // ------------------------------------------------------------------------
//         // 4) ACCUMULATED ANGLE FOR STARTUP: Use ramped angle like kickstart
//         // ------------------------------------------------------------------------
//         // After kickstart, encoder-based may not work immediately
//         // Use accumulated angle based on target velocity until motor is spinning

//         // ------------------------------------------------------------------------
//         // 4) ANGLE SOURCE: Use encoder directly
//         // ------------------------------------------------------------------------

//         // CRITICAL: Use cached encoder angle from loopFOC()
//         extern volatile float g_cached_encoder_angle;
//         extern volatile float g_kickstart_end_angle_elec;

//         // Use encoder directly, which now reflects actual position after kickstart
//         float electrical_angle_radians = g_cached_encoder_angle;

//         // Debug: Show we're using pure closed-loop
//         g_open_loop_elec_before_blend = 0.0f;
//         g_closed_loop_elec_before_blend = electrical_angle_radians;
//         // Debug: capture blended angle before rate limiting
//         g_blended_angle_before_rate_limit = electrical_angle_radians;

//         // Rate limit angle changes to prevent voltage spikes
//         static float prev_electrical_angle = 0.0f;
//         static bool prev_angle_initialized = false;

//         // CRITICAL: Initialize prev_electrical_angle on first call to avoid huge jump
//         if (!prev_angle_initialized)
//         {
//             prev_electrical_angle = electrical_angle_radians;
//             prev_angle_initialized = true;
//         }

//         // CRITICAL: Unwrap current angle relative to previous angle BEFORE calculating change
//         // This prevents huge jumps when angle wraps from 2π to 0 or vice versa
//         float unwrapped_electrical_angle = electrical_angle_radians;
//         float raw_diff = electrical_angle_radians - prev_electrical_angle;

//         // If we wrapped forward (went from ~6.28 to ~0), add 2π to current angle
//         if (raw_diff < -MY_PI)
//         {
//             unwrapped_electrical_angle += TWO_PI;
//         }
//         // If we wrapped backward (went from ~0 to ~6.28), subtract 2π from current angle
//         else if (raw_diff > MY_PI)
//         {
//             unwrapped_electrical_angle -= TWO_PI;
//         }

//         // Now calculate change in unwrapped space (no discontinuities)
//         float angle_change = unwrapped_electrical_angle - prev_electrical_angle;

//         // Rate limiter disabled - use angle directly
//         g_rate_limiter_active = 0.0f;
//         prev_electrical_angle = unwrapped_electrical_angle;
//         electrical_angle_radians = normalize_radians(unwrapped_electrical_angle);

//         // Debug: capture angle after rate limiting
//         g_blended_angle_after_rate_limit = electrical_angle_radians;
//         g_angle_change_limited = angle_change;

//         // Debug outputs
//         g_hybrid_blend_factor = blend_factor;
//         g_hybrid_open_loop_angle = electrical_angle_radians;  // Pure closed-loop - no separate open-loop
//         g_hybrid_closed_loop_angle = electrical_angle_radians;
//         g_hybrid_measured_speed = fabs(measured_mechanical_rad_per_sec);

//         // this blockes float current_mech_angle = m_sensor.get_mechanical_phase_angle_radians();

//         // NON-BLOCKING: Calculate from cached electrical angle instead of blocking SPI call
//         // Convert electrical back to mechanical: add offset back then divide by pole pairs
//         float current_mech_angle = (electrical_angle_radians + m_radian_offset_to_electric_zero) / static_cast<float>(NUM_POLE_PAIRS);


//         g_mech_angle_for_foc = current_mech_angle;
//         g_accumulated_mech_rad = current_mech_angle;  // Pure closed-loop - show encoder angle

//         // Debug: capture electrical angle
//         g_sensor_elec_angle_for_cl = electrical_angle_radians;
//         g_elec_angle_for_foc = electrical_angle_radians;
//         g_angle_used_in_voltage_commutation = electrical_angle_radians;  // Debug: angle for Inverse Park

//         // CRITICAL: Apply the voltage we just calculated!
//         // This was missing - voltage was calculated but never applied
//         setPhaseVoltage(m_voltage.q, m_voltage.d, electrical_angle_radians);

// #if 0  // Old closed-loop only code - disabled
//     // CRITICAL: Use cached encoder angle from loopFOC()
//     // This ensures voltage commutation uses the EXACT SAME angle as current sensing
//     // even though they run at different rates (loopFOC=10kHz, this=1kHz)
//     extern volatile float g_cached_encoder_angle;
//     float electrical_angle_radians = g_cached_encoder_angle;

//     // Debug outputs
//     float current_mech_angle = m_sensor.get_mechanical_phase_angle_radians();
//     g_mech_angle_for_foc = current_mech_angle;
//     g_accumulated_mech_rad = current_mech_angle;  // For compatibility

//     // Debug: capture electrical angle from sensor
//     g_sensor_elec_angle_for_cl = electrical_angle_radians;
//     g_elec_angle_for_foc = electrical_angle_radians;
//     g_angle_used_in_voltage_commutation = electrical_angle_radians;  // Debug: angle for Inverse Park
// #endif

// #if 0  // Use accumulated angle for commutation (sensor stuck at 0!)
//     // Simply advance angle based on commanded speed (open-loop angle generation)
//     const float angle_increment = target_mechanical_rad_per_sec * delta_seconds;

//     // Capture state before accumulation
//     g_angle_before_add = m_accumulated_mechanical_radians;
//     g_angle_increment = angle_increment;

//     // Accumulate angle
//     m_accumulated_mechanical_radians += angle_increment;
//     m_accumulated_mechanical_radians = normalize_radians(m_accumulated_mechanical_radians);

//     // Capture state after accumulation
//     g_angle_after_add = m_accumulated_mechanical_radians;

//     // Debug outputs
//     g_accumulated_mech_rad = m_accumulated_mechanical_radians;
//     g_velocity_correction = angle_increment * 1000.0f; // Show increment scaled up
//     g_mech_angle_for_foc = m_accumulated_mechanical_radians;

//     // Convert accumulated angle directly to electrical (no offset subtraction needed)
//     float electrical_angle_radians =
//         mechanical_to_electrical_radians(m_accumulated_mechanical_radians);

//     electrical_angle_radians = normalize_radians(electrical_angle_radians);

//     g_elec_angle_for_foc = electrical_angle_radians;
//     g_angle_used_in_voltage_commutation = electrical_angle_radians;  // Debug: angle for Inverse Park
// #endif

//     // ------------------------------------------------------------------------
//     // 5) Apply voltage in dq frame at the correct electrical angle
//     // ------------------------------------------------------------------------
//     setPhaseVoltage(m_voltage.q,
//                     m_voltage.d,
//                     electrical_angle_radians);

//     // ------------------------------------------------------------------------
//     // 6) Debug / telemetry (optional globals)
//     // ------------------------------------------------------------------------
//     // CRITICAL: g_as5048_angle is now only updated by update_buffers() from SPI data
//     // Do not overwrite it here
//     g_electrical_rad_ref  = electrical_angle_radians;
//     g_velocity_correction = velocity_error_mech_rad_per_sec; // "error" here
//     g_amperage_q          = m_amperage.q;
//     g_voltage_q           = m_voltage.q;
//     g_voltage_d           = m_voltage.d;
// }
// #endif

//======================================
// 1/1/2025 - OLD IMPLEMENTATION (REMOVED)
// #if 0
// void StepperMotor::update_speed_closed_loop(float target_mechanical_rps,
//                                             float delta_seconds)
// {
//     // PI controller integral term - declared at function scope for persistence
//     static float velocity_error_integral = 0.0f;


//     // DEBUG: Track if this function is being called and with what target
//     g_update_speed_cl_call_count++;
//     g_target_rps_to_cl_controller = target_mechanical_rps;
//     g_delta_seconds_to_cl_controller = delta_seconds;

//     // DEBUG: Track function entry
//     extern volatile uint32_t g_cl_function_entry_count;
//     extern volatile uint32_t g_cl_function_exit_count;
//     g_cl_function_entry_count++;

//     // DEBUG: Track ramped_speed vs epsilon to diagnose why angle becomes 0 so often

// #if 1  // DEBUG: Smooth ramp for open-loop startup testing only
// // Smooth ramp-up to target speed
// // CRITICAL: Initialize above epsilon threshold (1.0e-5) to avoid repeated angle==0 triggers during startup
// // The ramping increment is ACCEL_RATE * delta_seconds = 1.0 * 0.0001 = 0.0001 rad/s per iteration
// // Starting at 0.0 would cause ramped_speed to hover around epsilon for many iterations,
// // triggering setPhaseVoltage(0,0,0) repeatedly and causing spikes in g_park_sin/cos
// // NOTE: After kickstart, motor is already moving at ~1.88 rad/s, so ramped_speed
// // should initialize to a value that allows smooth continuation (not deceleration)
// static float ramped_speed = 2.0;  // Start at 2.0 rad/s to match post-kickstart velocity
// static bool post_kickstart_initialized = false;

// // CRITICAL: After kickstart completes, initialize ramped_speed to match target
// // This prevents deceleration when transitioning from kickstart to CL control
// extern volatile float g_kickstart_just_completed;
// if (g_kickstart_just_completed && !post_kickstart_initialized)
// {
//     ramped_speed = target_mechanical_rps;  // Initialize to current target (set by initFOC)
//     post_kickstart_initialized = true;
//     g_kickstart_just_completed = false;
// }
// #if 0  // Ultra-gentle acceleration for startup synchronization
//     const float ACCEL_RATE = 0.25f; // rad/s² - extremely slow to allow rotor to follow
// #else  // Faster acceleration (after motor is spinning)
//     //const float ACCEL_RATE = 1.0f; // rad/s² - gentle acceleration
//     const float ACCEL_RATE = 1.0f; // rad/s² - gentle acceleration
// #endif

//     // Ramp toward target
//     if (ramped_speed < target_mechanical_rps)
//     {
//         ramped_speed += ACCEL_RATE * delta_seconds;
//         if (ramped_speed > target_mechanical_rps)
//             ramped_speed = target_mechanical_rps;
//     }
//     else if (ramped_speed > target_mechanical_rps)
//     {
//         ramped_speed -= ACCEL_RATE * delta_seconds;
//         if (ramped_speed < target_mechanical_rps)
//             ramped_speed = target_mechanical_rps;
//     }

//     // Use ramped speed instead of direct target
//     const float actual_target_rps = ramped_speed;
// #else
//     // Normal operation - use target directly
//     const float actual_target_rps = target_mechanical_rps;
// #endif

//     // Treat very small targets as "stop"
//     //const float EPSILON_SPEED_RPS(1.0e-4f);

//     // DEBUG: Always capture these values to see the relationship
//     g_debug_ramped_speed = ramped_speed;
//     g_debug_actual_target_rps = actual_target_rps;

//     // EPSILON CHECK DISABLED - Motor should run continuously
//     // The epsilon check was causing spurious stops
//     // If you need to stop the motor, set target to 0 explicitly via update_target_rad_per_sec()

//     extern volatile float g_motor_stop_reason;
//     extern volatile float g_m_target_when_checked;
//     extern volatile float g_cmd_rps_when_checked;
//     extern volatile float g_cmd_rps;

//     // Debug: Always capture what we're checking
//     g_m_target_when_checked = m_target;
//     g_cmd_rps_when_checked = g_cmd_rps;
//     g_motor_stop_reason = 0.0f;  // Motor running normally

//     // Debug incoming parameters FIRST
//     g_ramped_target_rps = ramped_speed; // Show actual ramped speed being used
//     g_target_rad_per_sec = target_mechanical_rps; // Show commanded target speed

//     if (delta_seconds <= 0.0f)
//     {
//         //return;
//     }

//     // ------------------------------------------------------------------------
//     // 1) Use ramped speed for smooth acceleration
//     // ------------------------------------------------------------------------
//     const float target_mechanical_rad_per_sec = actual_target_rps;

//     // Update debug variables
//     g_loop_counter++;
//     g_speed_update_counter++;

//     // ------------------------------------------------------------------------
//     // 2) Target and measured mechanical speed (radians/sec)
//     // ------------------------------------------------------------------------


// #if 1  // Closed-loop with Kalman filter velocity feedback - ENABLED!
//     // Use Kalman filter velocity estimate (much better than simple differentiation)
//     // UNITS: Kalman velocity is in MECHANICAL rad/s (not electrical)
//     float measured_mechanical_rad_per_sec = m_sensor.get_mechanical_velocity_rad_per_sec();

//     // Debug: capture raw velocity from Kalman filter BEFORE any processing
//     g_velocity_before_filter = measured_mechanical_rad_per_sec;
//     g_velocity_spike_reject_active = 0.0f;  // No spike rejection active

//     // NO FILTERING - Use Kalman velocity directly!
//     // The Kalman filter already provides optimal filtering
//     // Any additional filtering causes control loop to use stale/wrong velocity

//     // Only sanity check: clamp physically impossible velocities (>150 rad/s)
//     if (fabs(measured_mechanical_rad_per_sec) > 150.0f)
//     {
//         // Velocity over 150 rad/s is unrealistic for this motor - clamp it
//         measured_mechanical_rad_per_sec = fmaxf(fminf(measured_mechanical_rad_per_sec, 150.0f), -150.0f);
//         g_velocity_spike_reject_active = 1.0f;  // Indicate clamping occurred
//     }

//     g_velocity_after_clamp = measured_mechanical_rad_per_sec;

//     // Debug: capture the velocity we're using for control (should match Kalman!)
//     // UNITS: mechanical rad/s (same as g_as5048_velocity from sensor)
//     g_measured_velocity_for_control = measured_mechanical_rad_per_sec;

//     // ------------------------------------------------------------------------
//     // 3) Compute feedforward + feedback voltage
//     // ------------------------------------------------------------------------

//     // POST-KICKSTART BOOST: Use target velocity for feedforward immediately after kickstart
//     // Problem: After kickstart, measured velocity is ~0, so feedforward calculates ~0 voltage
//     // Solution: Use target velocity for a few seconds after startup to maintain motion
//     static bool post_kickstart_mode = true;
//     static uint32_t kickstart_complete_time = 0;

//     if (post_kickstart_mode)
//     {
//         // Check if motor is moving and some time has passed
//         if (fabs(measured_mechanical_rad_per_sec) > 2.0f ||
//             (kickstart_complete_time > 0 && (micros() - kickstart_complete_time) > 3000000))  // 3 seconds
//         {
//             post_kickstart_mode = false;  // Transition to normal mode
//         }
//         else if (kickstart_complete_time == 0)
//         {
//             kickstart_complete_time = micros();  // Mark kickstart complete time
//         }
//     }

//     // CRITICAL: Separate back-EMF (use MEASURED velocity) from feedforward (use TARGET velocity)
//     // Back-EMF calculation MUST use actual motor speed to be accurate
//     // If we use target velocity (80 rad/s = 4000 rad/s elec), back-EMF will be 100V and saturate immediately!

//     // Back-EMF uses MEASURED velocity (actual motor speed)
//     const float actual_electrical_rad_per_sec = mechanical_to_electrical_radians(measured_mechanical_rad_per_sec);
//     const float inductance_value = inductance(actual_electrical_rad_per_sec);
//     const float mag_flux_linkage_q = inductance_value * m_amperage.q;
//     const float back_emf_q_axis = actual_electrical_rad_per_sec
//                                 * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

//     // Feedforward uses TARGET velocity for aggressive acceleration
//     //const float target_electrical_rad_per_sec = mechanical_to_electrical_radians(target_mechanical_rad_per_sec);

//     // DEBUG: Add variables to diagnose back-EMF calculation
//     extern volatile float g_debug_inductance_value;
//     extern volatile float g_debug_mag_flux_linkage_q;
//     extern volatile float g_debug_back_emf_q_axis;
//     extern volatile float g_debug_perm_magnet_flux;
//     extern volatile float g_debug_measured_elec_rad_per_sec;
//     extern volatile float g_debug_measured_mech_rad_per_sec;
//     extern volatile float g_debug_amperage_q_for_bemf;

//     g_debug_inductance_value = inductance_value;
//     g_debug_mag_flux_linkage_q = mag_flux_linkage_q;
//     g_debug_back_emf_q_axis = back_emf_q_axis;
//     g_debug_perm_magnet_flux = PERM_MAGNET_FLUX_LINKAGE;
//     g_debug_measured_elec_rad_per_sec = actual_electrical_rad_per_sec;
//     g_debug_measured_mech_rad_per_sec = measured_mechanical_rad_per_sec;
//     g_debug_amperage_q_for_bemf = m_amperage.q;

//     // CRITICAL: Use ACTUAL MEASURED CURRENT for feedforward (not fixed 2A assumption!)
//     // For unloaded motor: I_q ≈ 0.3A (friction/windage only)
//     // For loaded motor: I_q increases based on load torque
//     // Using measured I_q gives accurate feedforward for any load condition
//     float measured_current_q = fabs(m_amperage.q);

//     // Clamp to reasonable range (0.1A to current limit)
//     // Minimum 0.1A prevents division-by-zero and ensures minimum voltage for startup
//     const float MIN_FEEDFORWARD_CURRENT = 0.1f;
//     const float MAX_FEEDFORWARD_CURRENT = m_current_limit;
//     measured_current_q = fmaxf(fminf(measured_current_q, MAX_FEEDFORWARD_CURRENT), MIN_FEEDFORWARD_CURRENT);

//     // Feedforward voltage = resistive drop + back-EMF
//     // Use DC resistance for steady-state I×R drop (DQ current is DC)
//     float base_voltage_q = measured_current_q * PHASE_RESISTANCE
//                          + fabs(back_emf_q_axis);

//     // AGGRESSIVE VOLTAGE BOOST: Motor needs MORE voltage as it accelerates, not less
//     // Scale voltage with target speed to provide torque for acceleration
//     const float BASE_MIN_VOLTAGE = 2.0f;
//     const float VOLTAGE_PER_RAD_S = 0.3f;  // Add voltage as target speed increases

//     float required_voltage = BASE_MIN_VOLTAGE + (target_mechanical_rad_per_sec * VOLTAGE_PER_RAD_S);

//     if (base_voltage_q < required_voltage && target_mechanical_rad_per_sec > 0.05f)
//     {
//         base_voltage_q = required_voltage;
//     }

//     // Feedback correction (mechanical domain) - PI CONTROLLER
//     // TUNING GUIDE:
//     //   Kp controls response speed (higher = faster, but may overshoot)
//     //   Ki eliminates steady-state error (higher = faster convergence, but may oscillate)
//     //
//     //   Current tuning (for 20V motor, 25 rad/s max):
//     //     Kp = 1.5  (3× baseline, good transient response)
//     //     Ki = 0.5  (moderate integration, eliminates error in ~5s)
//     //
//     //   If motor oscillates: reduce Kp to 1.0, reduce Ki to 0.3
//     //   If response too slow: increase Kp to 2.0, keep Ki at 0.5
//     //   If steady-state error remains: increase Ki to 0.8
//     const float Kp_velocity_mech = 1.5f;   // Proportional gain (increased for faster response)
//     const float Ki_velocity_mech = 0.5f;   // Integral gain (eliminate steady-state error)



//     const float velocity_error_mech_rad_per_sec =
//         target_mechanical_rad_per_sec - measured_mechanical_rad_per_sec;


//     // Integration with anti-windup
//     // Note: velocity_error_integral is unitless accumulator (rad/s × seconds = radians)
//     // Ki converts it to voltage: V = Ki × integral
//     const float INTEGRAL_MAX = 30.0f;  // Limit integral accumulator to ±30 rad
//     velocity_error_integral += velocity_error_mech_rad_per_sec * 0.0001f;  // dt = 100us (10kHz)
//     velocity_error_integral = fmaxf(fminf(velocity_error_integral, INTEGRAL_MAX), -INTEGRAL_MAX);

//     // Calculate PI feedback correction voltage
//     float proportional_term = Kp_velocity_mech * velocity_error_mech_rad_per_sec;
//     float integral_term = Ki_velocity_mech * velocity_error_integral;
//     float feedback_correction = proportional_term + integral_term;

//     // ANTI-WINDUP: Prevent negative feedback from fighting startup
//     // During startup (low measured velocity), only allow positive feedback correction
//     // This prevents the controller from reducing voltage when motor is trying to start
//     if (measured_mechanical_rad_per_sec < 1.0f && feedback_correction < 0.0f)
//     {
//         feedback_correction = 0.0f;  // No negative correction during startup
//     }

//     // SATURATION PROTECTION: Limit feedback magnitude to prevent instability
//     const float MAX_FEEDBACK_CORRECTION = 18.0f;  // Increased limit for better tracking (was 10V)
//     float feedback_correction_clamped = fmaxf(fminf(feedback_correction, MAX_FEEDBACK_CORRECTION), -MAX_FEEDBACK_CORRECTION);

//     // Anti-windup: If feedback is saturated, stop integrating in that direction
//     if ((feedback_correction > MAX_FEEDBACK_CORRECTION && velocity_error_mech_rad_per_sec > 0.0f) ||
//         (feedback_correction < -MAX_FEEDBACK_CORRECTION && velocity_error_mech_rad_per_sec < 0.0f))
//     {
//         // Back off the integrator slightly to prevent windup
//         velocity_error_integral *= 0.99f;
//     }

//     // Reset integrator if motor is disabled or at target (prevent windup when stopped)
//     if (!m_enabled || (fabs(velocity_error_mech_rad_per_sec) < 0.05f && fabs(target_mechanical_rad_per_sec) < 0.01f))
//     {
//         velocity_error_integral = 0.0f;
//     }

//     // Use clamped feedback for control
//     feedback_correction = feedback_correction_clamped;

//     // Debug: capture all control components
//     g_velocity_error_for_debug = velocity_error_mech_rad_per_sec;
//     g_base_voltage_q_feedforward = base_voltage_q;
//     g_feedback_voltage_correction = feedback_correction;  // After clamping
//     g_velocity_error_integral = velocity_error_integral;  // Accumulator (rad)
//     g_velocity_correction_p = proportional_term;  // Before clamping (V)
//     g_velocity_correction_i = integral_term;  // Before clamping (V)
//     g_feedforward_current_q = measured_current_q;  // Measured I_q used for feedforward

//     float desired_voltage_q = base_voltage_q + feedback_correction;
//     g_desired_voltage_q_before_clamp = desired_voltage_q;

//     // Clamp to available DC bus / configured limit
//     m_voltage.q = symetric_clamp(desired_voltage_q, m_voltage_limit);
//     m_voltage.d = 0.0f; // no field weakening for now

//     // Debug: Capture m_voltage.q after calculation
//     g_m_voltage_q_in_controller = m_voltage.q;
// #else
//     // ------------------------------------------------------------------------
//     // 3) PURE OPEN-LOOP VOLTAGE (sensor broken, returns constant value)
//     // ------------------------------------------------------------------------
//     const float velocity_error_mech_rad_per_sec = 0.0;
//     const float OPEN_LOOP_VOLTAGE = 10.0f; // Increased to overcome cogging torque
//     m_voltage.q = OPEN_LOOP_VOLTAGE;
//     m_voltage.d = 0.0f;
// #endif

//     // ------------------------------------------------------------------------
//     // 4) ACCUMULATED ANGLE FOR STARTUP: Use ramped angle like kickstart
//     // ------------------------------------------------------------------------
//     // After kickstart, encoder-based may not work immediately
//     // Use accumulated angle based on target velocity until motor is spinning


//     static float accumulated_angle = 0.0f;
//     static bool accumulated_angle_initialized = false;
//     //static uint32_t startup_begin_time = 0;
//     volatile extern  uint32_t startup_begin_time;
//     static bool encoder_mode_locked = false;  // Once in encoder mode, stay there

//     // Initialize accumulated angle from CACHED encoder on first call (non-blocking)



//     // Initialize accumulated angle from CACHED encoder on first call (non-blocking)
//     if (!accumulated_angle_initialized)
//     {
//         accumulated_angle = m_current_electrical_radians;  // Non-blocking: use cached angle
//         accumulated_angle_initialized = true;
//         startup_begin_time = micros();
//     }

//     // Force encoder mode if just exited kickstart
//     if (m_force_use_encoder)
//     {
//         // Allow accumulated mode for a few seconds after kickstart for smooth transition
//  //       startup_begin_time = micros();  // Reset timer to allow 5 more seconds
//         m_force_use_encoder = false;
//         encoder_mode_locked = false;  // Allow transition cycle
//     }

//     // STARTUP MODE: Use accumulated angle for first few seconds or until spinning
//     //float measured_speed = fabs(measured_mechanical_rad_per_sec);
//     //bool use_accumulated = ((micros() - startup_begin_time) < 5000000);  // 5 seconds only, no speed condition
//     //bool use_accumulated = ((micros() - startup_begin_time) < 5000000) && (startup_begin_time != 0);
//     // OVERFLOW-SAFE: Check if we're still in the 5-second startup window
//     // Handle micros() overflow: if current time is less than start time, overflow occurred
//     uint32_t current_time = micros();
//     uint32_t elapsed_time;
//     bool time_check_valid = false;

//     if (current_time >= startup_begin_time) {
//         // Normal case: no overflow
//         elapsed_time = current_time - startup_begin_time;
//         time_check_valid = true;
//     } else if (startup_begin_time != 0) {
//         // Overflow occurred: calculate wrapped time
//         // Time from startup_begin_time to overflow + time after overflow
//         elapsed_time = (UINT32_MAX - startup_begin_time) + current_time;
//         time_check_valid = true;
//     }

//     // Once we've transitioned to encoder mode, STAY THERE (don't cycle back)
//     bool use_accumulated = !encoder_mode_locked &&
//                          ((time_check_valid && elapsed_time < 5000000)
//                          || (m_radian_offset_to_electric_zero == NOT_SET)
//                          || (startup_begin_time == 0));


//     // CRITICAL: Reset integral on first iteration in encoder mode to prevent voltage spike
//     static bool was_in_accumulated_last_iteration = true;
//     if (was_in_accumulated_last_iteration && !use_accumulated) {
//         //velocity_error_integral = 0.0f;  // Just transitioned, reset integral
//     }
//     was_in_accumulated_last_iteration = use_accumulated;

//     float electrical_angle_radians;
//     //float blend_factor = 1.0f;

//     // Track mode transitions to synchronize angles
//     static bool was_using_accumulated = true;

//     if (use_accumulated && target_mechanical_rad_per_sec > 0.05f)
//     {

//         // Debug: count how many times this executes
//         extern volatile uint32_t g_accumulate_call_count;
//         g_accumulate_call_count++;

//         static uint32_t last_accumulate_time = 0;
//         uint32_t current_time = micros();

//         // Initialize on first call
//         float actual_delta_seconds;
//         if (last_accumulate_time == 0)
//         {
//             actual_delta_seconds = 0.0001f;  // Assume 10kHz for first iteration
//             last_accumulate_time = current_time;
//         }
//         else
//         {
//             // OVERFLOW-SAFE: Handle micros() overflow
//             uint32_t delta_time_us;
//             if (current_time >= last_accumulate_time) {
//                 // Normal case
//                 delta_time_us = current_time - last_accumulate_time;
//             } else {
//                 // Overflow occurred
//                 delta_time_us = (UINT32_MAX - last_accumulate_time) + current_time;
//             }

//             actual_delta_seconds = delta_time_us * 1.0e-6f;
//             last_accumulate_time = current_time;

//             // Sanity check - if too long or negative (shouldn't happen now), use default
//             if (actual_delta_seconds <= 0.0f || actual_delta_seconds > 0.001f)
//             {
//                 actual_delta_seconds = 0.0001f;
//             }
//         }

//         float angle_increment = target_mechanical_rad_per_sec * actual_delta_seconds;
//         accumulated_angle += mechanical_to_electrical_radians(angle_increment);

//         // Normalize to keep in [0, 2π) range like encoder
//         accumulated_angle = _normalizeAngle(accumulated_angle);

//         // Calculate shortest angular error between accumulated and STABLE encoder angle
//         // Use Kalman position (stable) instead of m_current_electrical_radians (oscillating)
//         extern volatile float g_kalman_position;
//         float stable_mech_angle = g_kalman_position - m_sensor_offset;
//         float stable_encoder_electrical = mechanical_to_electrical_radians(stable_mech_angle);
//         stable_encoder_electrical -= m_radian_offset_to_electric_zero;
//         stable_encoder_electrical = normalize_radians(stable_encoder_electrical);

//         float angle_error = stable_encoder_electrical - accumulated_angle;

//         // Unwrap to find shortest path
//         if (angle_error > MY_PI)
//             angle_error -= TWO_PI;
//         else if (angle_error < -MY_PI)
//             angle_error += TWO_PI;

//         // Apply small nudge toward STABLE encoder angle
//         const float SYNC_GAIN = 0.002f;
//         accumulated_angle += SYNC_GAIN * angle_error;

//         // Re-normalize after nudge
//         accumulated_angle = _normalizeAngle(accumulated_angle);

//         // Use for commutation
//         electrical_angle_radians = accumulated_angle;




//         // BLENDING: Reset blend timer when in accumulated mode
//         extern volatile uint32_t g_encoder_blend_start_time;
//         g_encoder_blend_start_time = 0;  // Reset - will restart on next transition

//         // Debug - expose values
//         extern volatile float g_accumulated_angle;
//         g_accumulated_angle = accumulated_angle;
//         g_angle_difference_accum_vs_encoder = angle_error;

//         extern volatile float g_accumulated_angle_source;
//         g_accumulated_angle_source = 2.0f;  // 2.0 = set by closed_loop



//         was_using_accumulated = true;
//         encoder_mode_locked = false;  // In accumulated mode, not locked

//         extern volatile float g_encoder_mode_locked;
//         g_encoder_mode_locked = 0.0f;
//     }
//     else
//     {
//         // In encoder mode - lock it so we don't cycle back to accumulated
//         encoder_mode_locked = true;

//         extern volatile float g_encoder_mode_locked;
//         g_encoder_mode_locked = 1.0f;
//         // =====================================================================
//         // GRADUAL TRANSITION FROM ACCUMULATED TO ENCODER (2-second blend)
//         // =====================================================================
//         // Problem: Hard switch to encoder can cause large angle jump if:
//         //   - Encoder has error bits set
//         //   - Accumulated angle drifted during 5-second startup
//         //   - Motor spinning fast (large velocity at transition)
//         //
//         // Solution: Blend over 2 seconds to smooth the transition
//         // =====================================================================

//         static uint32_t encoder_mode_start_time = 0;

//         // CRITICAL: Sync accumulated angle on first entry to encoder mode
//         // This prevents discontinuity at start of blend
//         // CRITICAL: Sync accumulated angle on first entry to encoder mode
//         // This prevents discontinuity at start of blend
//         if (was_using_accumulated)
//         {

//             g_transition_accumulated_angle_before = accumulated_angle;
//             g_transition_m_current_electrical_before = m_current_electrical_radians;
//             g_transition_voltage_q_before = m_voltage.q;

//             // Capture the phase mismatch at transition
//             extern volatile float g_debug_accumulated_at_transition;
//             extern volatile float g_debug_encoder_at_transition;
//             extern volatile float g_debug_phase_mismatch;

//             // FIX: Accumulated has drifted from actual rotor position
//             // Snap it to the encoder position to avoid phase discontinuity
//             accumulated_angle = m_current_electrical_radians;

//             g_transition_accumulated_angle_after = accumulated_angle;

//             g_debug_accumulated_at_transition = accumulated_angle;
//             g_debug_encoder_at_transition = m_current_electrical_radians;

//             float phase_error = m_current_electrical_radians - accumulated_angle;
//             if (phase_error > MY_PI) phase_error -= TWO_PI;
//             else if (phase_error < -MY_PI) phase_error += TWO_PI;
//             g_debug_phase_mismatch = phase_error;

//             // Calculate Kalman phase offset at transition (for correction during encoder mode)
//             //static float kalman_phase_offset = 0.0f;
//             extern volatile float g_kalman_position;
//             float kalman_mech_at_trans = g_kalman_position - m_sensor_offset;
//             float kalman_mech_norm = kalman_mech_at_trans - TWO_PI * floorf(kalman_mech_at_trans / TWO_PI);
//             float kalman_elec_at_trans = mechanical_to_electrical_radians(kalman_mech_norm) - m_radian_offset_to_electric_zero;
//             kalman_elec_at_trans = normalize_radians(kalman_elec_at_trans);

//             // Phase error = where encoder says rotor is - where Kalman thinks it is
// //            float kalman_phase_error = m_current_electrical_radians - kalman_elec_at_trans;
//             float kalman_phase_error = g_transition_accumulated_angle_before - kalman_elec_at_trans;

//             if (kalman_phase_error > MY_PI) kalman_phase_error -= TWO_PI;
//             else if (kalman_phase_error < -MY_PI) kalman_phase_error += TWO_PI;

//             kalman_phase_offset = kalman_phase_error;

//             // Debug: capture the correction being applied
//             extern volatile float g_debug_kalman_phase_offset;
//             g_debug_kalman_phase_offset = kalman_phase_offset;


//             g_debug_kalman_phase_offset = kalman_phase_offset;

//             // Additional debug - capture intermediate values
//             extern volatile float g_debug_kalman_elec_at_trans;
//             extern volatile float g_debug_kalman_phase_error;
//             g_debug_kalman_elec_at_trans = kalman_elec_at_trans;
//             g_debug_kalman_phase_error = kalman_phase_error;


//             // Capture state at transition
//             g_transition_accumulated_angle_before = accumulated_angle;
//             g_transition_m_current_electrical_before = m_current_electrical_radians;
//             g_transition_voltage_q_before = m_voltage.q;
// #if 0
//             // CRITICAL FIX: Sync to smooth Kalman angle while preserving phase continuity
//             extern volatile float g_kalman_position;

//             // DEBUG: Capture all intermediate values
//             extern volatile float g_debug_kalman_mech;
//             extern volatile float g_debug_kalman_elec_unbounded;
//             extern volatile float g_debug_angle_error_unwrapped;
//             extern volatile float g_debug_angle_error_wrapped;
//             extern volatile float g_debug_accumulated_before_add;
//             extern volatile float g_debug_accumulated_after_add;
//             extern volatile float g_debug_accumulated_after_normalize;

//             float stable_mech_angle = g_kalman_position - m_sensor_offset;
//             g_debug_kalman_mech = stable_mech_angle;

//             // Convert Kalman mechanical angle to electrical (UNBOUNDED - don't normalize yet!)
//             float kalman_electrical_unbounded = mechanical_to_electrical_radians(stable_mech_angle);
//             kalman_electrical_unbounded -= m_radian_offset_to_electric_zero;
//             g_debug_kalman_elec_unbounded = kalman_electrical_unbounded;

//             // Calculate angular difference in unbounded space
//             float angle_error = kalman_electrical_unbounded - accumulated_angle;
//             g_debug_angle_error_unwrapped = angle_error;

//             // Wrap the ERROR to [-π, π] to find shortest path
//             while (angle_error > MY_PI) angle_error -= TWO_PI;
//             while (angle_error < -MY_PI) angle_error += TWO_PI;
//             g_debug_angle_error_wrapped = angle_error;

//             // Update accumulated_angle by the wrapped error
//             g_debug_accumulated_before_add = accumulated_angle;
//             accumulated_angle += angle_error;
//             g_debug_accumulated_after_add = accumulated_angle;

//             // Normalize to [0, 2π)
//             accumulated_angle = normalize_radians(accumulated_angle);
//             g_debug_accumulated_after_normalize = accumulated_angle;

//             // Integral accumulator (static to maintain state between calls)
//             //static float velocity_error_integral = 0.0f;
// #endif

// // DON'T SYNC - let the 2-second blend handle the transition
// // Syncing to unbounded Kalman causes massive jumps
// // The blend at line ~3570 will smoothly correct any drift

// // Capture velocity error and integral at transition
// extern volatile float g_transition_velocity_error;
// extern volatile float g_transition_integral_term;
// g_transition_velocity_error = target_mechanical_rad_per_sec - measured_mechanical_rad_per_sec;
// g_transition_integral_term = velocity_error_integral;

// // DO NOT reset integral - maintain bumpless transfer
// // velocity_error_integral = 0.0f;  // REMOVED

// // Capture state (no sync, so before = after)
// g_transition_accumulated_angle_after = accumulated_angle;


//             // Capture velocity error and integral at transition
//             extern volatile float g_transition_velocity_error;
//             extern volatile float g_transition_integral_term;
//             g_transition_velocity_error = target_mechanical_rad_per_sec - measured_mechanical_rad_per_sec;
//             g_transition_integral_term = velocity_error_integral;

//             // CRITICAL: Reset integral to prevent voltage spike
//             //velocity_error_integral = 0.0f;

//             // Capture state after sync
//             g_transition_accumulated_angle_after = accumulated_angle;

//             // Calculate the angle jump (should be small - Kalman is stable)
//             g_transition_angle_jump = accumulated_angle - g_transition_accumulated_angle_before;


//             if (g_transition_angle_jump > MY_PI) g_transition_angle_jump -= TWO_PI;
//             else if (g_transition_angle_jump < -MY_PI) g_transition_angle_jump += TWO_PI;

//             was_using_accumulated = false;

//             // Capture raw sensor values
//             extern volatile float g_as5048_angle;
//             g_transition_g_as5048_angle = g_as5048_angle;
//             g_transition_g_cached_encoder_angle = g_cached_encoder_angle;
//             g_transition_mech_angle_from_sensor = m_sensor.read_angle_radians_from_buffer();
//             g_transition_timestamp = micros();

//             // CRITICAL: Start blend timer NOW
//             encoder_mode_start_time = micros();

//             // Transition manager will own blend start (do not reset here)
//         }


//         // Start blend timer on first call in encoder mode (after was_using_accumulated processed)
//         if (encoder_mode_start_time == 0) {
//             encoder_mode_start_time = micros();
//         }

//         // Calculate blend factor based on time since transition

//         // Calculate blend factor based on time since transition
//         // OVERFLOW-SAFE: Handle micros() overflow
//         uint32_t current_time_blend = micros();
//         uint32_t time_in_encoder_mode;

//         if (current_time_blend >= encoder_mode_start_time) {
//             // Normal case: no overflow
//             time_in_encoder_mode = current_time_blend - encoder_mode_start_time;
//         } else {
//             // Overflow occurred: calculate wrapped time
//             time_in_encoder_mode = (UINT32_MAX - encoder_mode_start_time) + current_time_blend;
//         }

//         const uint32_t BLEND_DURATION_US = 1000000;  // 2 seconds blend period

//         // Blend factor: 0.0 = pure accumulated, 1.0 = pure encoder
//         float blend_to_encoder = static_cast<float>(time_in_encoder_mode) / BLEND_DURATION_US;

//         // Clamp to [0.0, 1.0] range
//         if (blend_to_encoder > 1.0f) {
//             blend_to_encoder = 1.0f;
//         }

//         // Debug: Expose blend factor for monitoring
//         extern volatile float g_encoder_blend_factor;
//         extern volatile uint32_t g_encoder_blend_start_time;
//         g_encoder_blend_factor = blend_to_encoder;
//         g_encoder_blend_start_time = encoder_mode_start_time;



//         // ---------------------------------------------------------------------
//         // Calculate blended angle with proper wrapping
//         // ---------------------------------------------------------------------


//         // Use smooth Kalman angle - normalize the MECHANICAL angle first, then convert to electrical
//         extern volatile float g_kalman_position;
//         float kalman_mech_unbounded = g_kalman_position - m_sensor_offset;

//         // Normalize mechanical angle to [0, 2π) FIRST (before multiplying by 50)
//         // This avoids precision loss with huge numbers
//         float kalman_mech_normalized = kalman_mech_unbounded - TWO_PI * floorf(kalman_mech_unbounded / TWO_PI);

//         // Now convert to electrical (multiply by pole pairs)
//         float kalman_elec = mechanical_to_electrical_radians(kalman_mech_normalized);
//         kalman_elec -= m_radian_offset_to_electric_zero;

//         // Normalize electrical to [0, 2π)
//         kalman_elec = normalize_radians(kalman_elec);

//         // Use Kalman angle with phase correction (offset calculated at transition)
//         //static float kalman_phase_offset = 0.0f;
//         electrical_angle_radians = kalman_elec + kalman_phase_offset;
//         electrical_angle_radians = normalize_radians(electrical_angle_radians);

//         // CRITICAL: Keep accumulated angle tracking the blended result
//         // This ensures smooth continuation if we ever switch back to accumulated mode
//         //accumulated_angle = electrical_angle_radians;

//         // Debug: Expose blended result for monitoring
//         extern volatile float g_blended_result;
//         g_blended_result = electrical_angle_radians;
//     }

//     // Debug: Show difference between accumulated and encoder angles
//     //extern volatile float g_angle_difference_accum_vs_encoder;
//     //g_angle_difference_accum_vs_encoder = accumulated_angle - g_cached_encoder_angle;
//     // Debug: Show difference between accumulated and encoder angles (unwrapped)
//     extern volatile float g_angle_difference_accum_vs_encoder;
//     float raw_diff = accumulated_angle - m_current_electrical_radians;
//     // Unwrap the difference for proper visualization
//     if (raw_diff > MY_PI)
//         raw_diff -= TWO_PI;
//     else if (raw_diff < -MY_PI)
//         raw_diff += TWO_PI;
//     g_angle_difference_accum_vs_encoder = raw_diff;


//     // Rate limit angle changes to prevent voltage spikes
//     static float prev_electrical_angle = 0.0f;
//     static bool prev_angle_initialized = false;

//     // CRITICAL: Initialize prev_electrical_angle on first call to avoid huge jump
//     if (!prev_angle_initialized)
//     {
//         prev_electrical_angle = electrical_angle_radians;
//         prev_angle_initialized = true;
//     }

//     // CRITICAL: Unwrap current angle relative to previous angle BEFORE calculating change
//     // This prevents huge jumps when angle wraps from 2π to 0 or vice versa
//     float unwrapped_electrical_angle = electrical_angle_radians;
//     //float
// 	raw_diff = electrical_angle_radians - prev_electrical_angle;

//     // If we wrapped forward (went from ~6.28 to ~0), add 2π to current angle
//     if (raw_diff < -MY_PI)
//     {
//         unwrapped_electrical_angle += TWO_PI;
//     }
//     // If we wrapped backward (went from ~0 to ~6.28), subtract 2π from current angle
//     else if (raw_diff > MY_PI)
//     {
//         unwrapped_electrical_angle -= TWO_PI;
//     }

//     // Now calculate change in unwrapped space (no discontinuities)
//     //float angle_change = unwrapped_electrical_angle - prev_electrical_angle;

//     // TEMPORARY: Disable rate limiter for testing
//     // The rate limiter was causing vibration instead of rotation
//     // TODO: Re-enable with proper tuning once motor spins correctly
//     #if 0
//     // Limit maximum angle change per iteration (at 10kHz, this is ~10000 rad/s max)
//     const float MAX_ANGLE_CHANGE = 1.0f;  // radians per iteration
//     g_rate_limiter_active = 0.0f;  // Reset flag

//     float rate_limited_angle;
//     if (angle_change > MAX_ANGLE_CHANGE)
//     {
//         rate_limited_angle = prev_electrical_angle + MAX_ANGLE_CHANGE;
//         g_rate_limiter_active = 1.0f;  // Rate limiter clamping positive
//     }
//     else if (angle_change < -MAX_ANGLE_CHANGE)
//     {
//         rate_limited_angle = prev_electrical_angle - MAX_ANGLE_CHANGE;
//         g_rate_limiter_active = -1.0f;  // Rate limiter clamping negative
//     }
//     else
//     {
//         rate_limited_angle = unwrapped_electrical_angle;
//     }

//     // Store the rate-limited angle for next iteration (in unwrapped space)
//     prev_electrical_angle = rate_limited_angle;

//     // Now normalize back to [0, 2π) for output
//     electrical_angle_radians = normalize_radians(rate_limited_angle);
//     #else
//     // Rate limiter disabled - use angle directly
//     g_rate_limiter_active = 0.0f;
//     prev_electrical_angle = unwrapped_electrical_angle;
//     electrical_angle_radians = normalize_radians(unwrapped_electrical_angle);
//     #endif


//     // ------------------------------------------------------------------------
//     // 4.5) Apply phase advance compensation for processing delays
//     // ------------------------------------------------------------------------
//     // Total delay: encoder read + computation + PWM update ≈ 50-100μs
//     const float PROCESSING_DELAY_SECONDS = 75e-6f;  // 75 microseconds

//     // Get current velocity from Kalman filter (mechanical rad/s)
//     float mechanical_velocity = m_sensor.get_mechanical_velocity_rad_per_sec();

//     // Calculate phase advance in electrical radians
//     float mechanical_phase_advance = mechanical_velocity * PROCESSING_DELAY_SECONDS;
//     float electrical_phase_advance = mechanical_to_electrical_radians(mechanical_phase_advance);

//     // Apply phase advance to commutation angle
//     float angle_with_advance = electrical_angle_radians + electrical_phase_advance;
//     angle_with_advance = _normalizeAngle(angle_with_advance);

//     // Debug: expose phase advance being used
//     extern volatile float g_phase_advance_for_commutation;
//     g_phase_advance_for_commutation = electrical_phase_advance;

//     //float current_mech_angle = m_sensor.get_mechanical_phase_angle_radians();


//     //--------------------------------------------
//     // Capture the first angle/voltage sent after transition
//     static bool first_encoder_angle_captured = false;
//     if (encoder_mode_locked && !first_encoder_angle_captured)
//     {
//         g_transition_electrical_angle_sent = electrical_angle_radians;
//         g_transition_voltage_q_after = m_voltage.q;
//         first_encoder_angle_captured = true;
//     }

//     // ------------------------------------------------------------------------
//     // 5) Apply voltage in dq frame at the correct electrical angle
//     // ------------------------------------------------------------------------
//     g_electrical_angle_to_setPhaseVoltage = electrical_angle_radians;  // Capture angle for diagnostics
//     setPhaseVoltage(m_voltage.q,
//                     m_voltage.d,
//                     angle_with_advance); //electrical_angle_radians);

//     // ------------------------------------------------------------------------
//     // 6) Debug / telemetry (optional globals)
//     // ------------------------------------------------------------------------
//     // CRITICAL: g_as5048_angle is now only updated by update_buffers() from SPI data
//     // Do not overwrite it here
//     g_electrical_rad_ref  = electrical_angle_radians;
//     g_velocity_correction = velocity_error_mech_rad_per_sec; // "error" here
//     g_amperage_q          = m_amperage.q;
//     g_voltage_q           = m_voltage.q;
//     g_voltage_d           = m_voltage.d;

//     // DEBUG: Track function exit
//     extern volatile uint32_t g_cl_function_exit_count;
//     g_cl_function_exit_count++;
// }
// #endif

//=============================================================================
//                     update_speed_closed_loop (REFACTORED)
//=============================================================================
// 1/9/26 - Refactored to use AngleTransitionManager for clean transitions
void StepperMotor::update_speed_closed_loop(float target_mechanical_rps,
                                            float delta_seconds)
{
    //=========================================================================
    // SECTION 1: INITIALIZATION & DEBUG TRACKING
    //=========================================================================

    // PI controller integral term - persistent between calls
    static float velocity_error_integral = 0.0f;

    // Debug tracking
    g_update_speed_cl_call_count++;
    g_target_rps_to_cl_controller = target_mechanical_rps;
    g_delta_seconds_to_cl_controller = delta_seconds;
    g_cl_function_entry_count++;
    g_loop_counter++;
    g_speed_update_counter++;

    if (delta_seconds <= 0.0f) {
        return;  // Invalid timestep
    }

    //=========================================================================
    // SECTION 2: SPEED RAMPING (SMOOTH ACCELERATION)
    //=========================================================================

    static float ramped_speed = 2.0f;  // Start at 2.0 rad/s to match post-kickstart
    static bool post_kickstart_initialized = false;

    // Persistent state for angle accumulation (moved early for transition management)
    static float accumulated_rad_elec = 0.0f;
    static bool accumulated_angle_initialized = false;
    static uint32_t last_accumulate_time = 0;
    static bool transition_prepared = false;  // Guard to call prepare_transition only once
    static float prev_accumulated_rad_elec = 0.0f;  // Capture last valid accumulated angle before transition
    static bool was_in_accumulated_early = true;  // Track mode transitions

    // Initialize ramped speed after kickstart completes
    extern volatile float g_kickstart_just_completed;
    if (g_kickstart_just_completed && !post_kickstart_initialized) {
        ramped_speed = target_mechanical_rps;
        post_kickstart_initialized = true;
        g_kickstart_just_completed = false;
    }

    // Ramp toward target
    const float ACCEL_RATE = 10.0f;  // rad/s²
    if (ramped_speed < target_mechanical_rps) {
        ramped_speed += ACCEL_RATE * delta_seconds;
        if (ramped_speed > target_mechanical_rps)
            ramped_speed = target_mechanical_rps;
    } else if (ramped_speed > target_mechanical_rps) {
        ramped_speed -= ACCEL_RATE * delta_seconds;
        if (ramped_speed < target_mechanical_rps)
            ramped_speed = target_mechanical_rps;
    }

    const float actual_target_rps = ramped_speed;
    const float target_mechanical_rad_per_sec = actual_target_rps;

    // Initialize accumulated angle on first call
    extern volatile uint32_t startup_begin_time;
    if (!accumulated_angle_initialized) {
        accumulated_rad_elec = m_current_electrical_radians;
        prev_accumulated_rad_elec = m_current_electrical_radians;  // Initialize with same value
        accumulated_angle_initialized = true;
        startup_begin_time = micros();
        last_accumulate_time = micros();
    }

    // Force encoder mode if just exited kickstart
    if (m_force_use_encoder) {
 //       startup_begin_time = micros();  // Reset timer for smooth transition
        m_force_use_encoder = false;
    }

    // Debug
    g_debug_ramped_speed = ramped_speed;
    g_debug_actual_target_rps = actual_target_rps;
    g_ramped_target_rps = ramped_speed;
    g_target_rad_per_sec = target_mechanical_rps;
    g_motor_stop_reason = 0.0f;  // Motor running normally

    //=========================================================================
    // SECTION 3: VELOCITY MEASUREMENT (KALMAN FILTER)
    //=========================================================================

    // Use Kalman filter velocity estimate (mechanical rad/s)
    float measured_mechanical_rad_per_sec = m_sensor.get_mechanical_velocity_rad_per_sec();

    // Debug: capture raw velocity
    g_velocity_before_filter = measured_mechanical_rad_per_sec;
    g_velocity_spike_reject_active = 0.0f;

    // // Sanity check: clamp physically impossible velocities
    // if (fabs(measured_mechanical_rad_per_sec) > 150.0f) {
    //     measured_mechanical_rad_per_sec = fmaxf(fminf(measured_mechanical_rad_per_sec, 150.0f), -150.0f);
    //     g_velocity_spike_reject_active = 1.0f;
    // }

    g_velocity_after_clamp = measured_mechanical_rad_per_sec;
    g_measured_velocity_for_control = measured_mechanical_rad_per_sec;

    //=========================================================================
    // SECTION 4: FEEDFORWARD VOLTAGE CALCULATION
    //=========================================================================

    // Back-EMF calculation (uses MEASURED velocity)
    const float actual_electrical_rad_per_sec = mechanical_to_electrical_radians(measured_mechanical_rad_per_sec);
    const float inductance_value = inductance(actual_electrical_rad_per_sec);
    const float mag_flux_linkage_q = inductance_value * m_amperage.q;
    const float back_emf_q_axis = actual_electrical_rad_per_sec * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

    // Debug back-EMF calculation
    g_debug_inductance_value = inductance_value;
    g_debug_mag_flux_linkage_q = mag_flux_linkage_q;
    g_debug_back_emf_q_axis = back_emf_q_axis;
    g_debug_perm_magnet_flux = PERM_MAGNET_FLUX_LINKAGE;
    g_debug_measured_elec_rad_per_sec = actual_electrical_rad_per_sec;
    g_debug_measured_mech_rad_per_sec = measured_mechanical_rad_per_sec;
    g_debug_amperage_q_for_bemf = m_amperage.q;

    // Use measured current for feedforward (clamp to reasonable range)
    float measured_current_q = fabs(m_amperage.q);
    const float MIN_FEEDFORWARD_CURRENT = 0.1f;
    const float MAX_FEEDFORWARD_CURRENT = m_current_limit;
    measured_current_q = fmaxf(fminf(measured_current_q, MAX_FEEDFORWARD_CURRENT), MIN_FEEDFORWARD_CURRENT);

    // Base feedforward voltage = resistive drop + back-EMF
    float base_voltage_q = measured_current_q * PHASE_RESISTANCE + fabs(back_emf_q_axis);

    // Voltage boost during acceleration (REDUCED to prevent saturation)
    const float BASE_MIN_VOLTAGE = 2.0f;

    // Feed Forward
    const float VOLTAGE_PER_RAD_S = 0.4f; //0.05f;  // Reduced from 0.3f to prevent saturation
    float required_voltage = BASE_MIN_VOLTAGE + (target_mechanical_rad_per_sec * VOLTAGE_PER_RAD_S);

    if (base_voltage_q < required_voltage && target_mechanical_rad_per_sec > 0.05f) {
        base_voltage_q = required_voltage;
    }

    //=========================================================================
    // DETERMINE MODE EARLY (needed for PI controller)
    //=========================================================================
    extern volatile uint32_t startup_begin_time;
    uint32_t current_time_for_mode = micros();
//    bool use_accumulated = should_use_accumulated_mode(startup_begin_time, current_time_for_mode);
    bool use_accumulated = !m_encoder_mode_locked && should_use_accumulated_mode(startup_begin_time, current_time_for_mode);
    float blended_electrical_angle_radians(0.0f);


    g_debug_use_accumulated = use_accumulated ? 1.0f : 0.0f;
    g_debug_time_since_startup_sec = (current_time_for_mode - startup_begin_time) * 1.0e-6f;
    g_debug_startup_begin_time = startup_begin_time;


    // Detect mode transition and reset flags
    if (use_accumulated) {
        transition_prepared = false;
        was_in_accumulated_early = true;
    }
#if 1
else if (transition_prepared) {
    // We've successfully completed first transition - lock into encoder mode permanently
    m_encoder_mode_locked = true;
    g_encoder_mode_locked = 1.0f;
}
#endif
    //=========================================================================
    // // --- PRECOMPUTE TRANSITION BLEND (so PI uses current-cycle gain) ---
    if (!use_accumulated)
    {
        extern volatile float g_kalman_rad_mech;

        //float mechanical_radians = m_sensor.get_mechanical_phase_angle_radians();


        float kalman_rad_elec =
        		m_sensor.get_kalman_electrical_angle_radians(
        				m_sensor_offset,
        		        m_radian_offset_to_electric_zero,
        		        NUM_POLE_PAIRS);

        // Prepare transition before first get_blended_angle call (only on actual mode transition)
        if (!transition_prepared && was_in_accumulated_early) {
            // Debug: Capture angles at transition start
            extern volatile float g_debug_last_accumulated_at_trans;
            extern volatile float g_debug_kalman_elec_at_trans;
            extern volatile float g_debug_angle_error_at_trans;
            extern volatile uint32_t g_debug_transition_time;

            g_debug_last_accumulated_at_trans = prev_accumulated_rad_elec;
            g_debug_kalman_elec_at_trans = kalman_rad_elec;
            g_debug_angle_error_at_trans = kalman_rad_elec - prev_accumulated_rad_elec;
            g_debug_transition_time = current_time_for_mode;

            m_transition_manager.prepare_transition(
                prev_accumulated_rad_elec,  // Use captured value from end of last accumulated cycle
                kalman_rad_elec,
                m_sensor_offset,
                m_radian_offset_to_electric_zero,
                NUM_POLE_PAIRS,
                current_time_for_mode
            );
            transition_prepared = true;
            was_in_accumulated_early = false;  // We've handled the transition
        }

        // This call updates g_transition_blend_factor internally
        blended_electrical_angle_radians
        = m_transition_manager.get_blended_angle(
            kalman_rad_elec, // m_current_electrical_radians,
            NUM_POLE_PAIRS,
            target_mechanical_rad_per_sec,
            delta_seconds
        );

        // Now we can set integration gain for THIS cycle
        extern volatile float g_integration_gain;
        extern volatile float g_transition_blend_factor;
        extern volatile float g_integral_freeze_active;

        const float min_gain = 0.2f;  // keep I authority at transition start
        const float blend = m_transition_manager.is_transitioning() ? g_transition_blend_factor : 1.0f;



        const float kEnableBlend = 0.15f;

        // Hold gain constant until encoder has real authority, then ramp
        float gain = 1.0f;

        if (m_transition_manager.is_transitioning())
        {
            if (blend < kEnableBlend)
            {
                gain = min_gain;   // keep integrator mostly frozen early
            }
            else
            {
                const float t = (blend - kEnableBlend) / (1.0f - kEnableBlend);  // rescale to 0..1
                gain = min_gain + (1.0f - min_gain) * t;
            }
        }
        else
        {
            gain = 1.0f;
        }
        g_integration_gain = gain;
    }
    else
    {
        // Ensure accumulated mode never inherits a frozen gain from encoder mode
        extern volatile float g_integration_gain;
        extern volatile float g_integral_freeze_active;
        g_integration_gain = 1.0f;
        g_integral_freeze_active = 0.0f;
    }


    //=========================================================================
    // SECTION 5: PI FEEDBACK CONTROLLER
    //=========================================================================

    const float Kp_velocity_mech = 3.0f;  // 1.5 Proportional gain
    const float Ki_velocity_mech = 1.5f;  // .5Integral gain
    const float INTEGRAL_MAX = 12.0f;     // Anti-windup limit

    const float velocity_error_mech_rad_per_sec = target_mechanical_rad_per_sec - measured_mechanical_rad_per_sec;

    // Integration with anti-windup (will be frozen during transition if in encoder mode)
    static bool allow_integration = true;

    // Check if we should freeze integration (set by encoder mode section below)
    extern volatile float g_integral_freeze_active;

    // Get integration gain (will be 0.0-1.0 depending on gradual ramp state)
    extern volatile float g_integration_gain;
    float integration_gain_to_use = g_integration_gain;

    if (use_accumulated && !m_transition_manager.is_transitioning())
    {
        integration_gain_to_use = 0.0f;
    }


    //if (allow_integration && integration_gain_to_use > 0.0f  && !use_accumulated)
    // Allow integration in encoder mode OR during transition (even if technically still in startup window)
    const bool in_transition = m_transition_manager.is_transitioning();
    if (allow_integration && integration_gain_to_use > 0.0f && (!use_accumulated || in_transition))
    {
        //velocity_error_integral += velocity_error_mech_rad_per_sec * 0.0001f * integration_gain_to_use;
        float dt = delta_seconds;
        if (dt <= 0.0f || dt > 0.001f) { dt = 0.0001f; }
        velocity_error_integral += velocity_error_mech_rad_per_sec * dt  * integration_gain_to_use;
        velocity_error_integral = fmaxf(fminf(velocity_error_integral, INTEGRAL_MAX), -INTEGRAL_MAX);
    }

    // Calculate PI terms
    float proportional_term = Kp_velocity_mech * velocity_error_mech_rad_per_sec;
    float integral_term = Ki_velocity_mech * velocity_error_integral;
    float feedback_correction = proportional_term + integral_term;

    // Anti-windup: No negative correction during startup
    if (measured_mechanical_rad_per_sec < 1.0f && feedback_correction < 0.0f) {
        feedback_correction = 0.0f;
    }

    // Saturation protection
    const float MAX_FEEDBACK_CORRECTION = 18.0f;
    float feedback_correction_clamped = fmaxf(fminf(feedback_correction, MAX_FEEDBACK_CORRECTION), -MAX_FEEDBACK_CORRECTION);

    // Back off integrator if saturated
    if ((feedback_correction > MAX_FEEDBACK_CORRECTION && velocity_error_mech_rad_per_sec > 0.0f) ||
        (feedback_correction < -MAX_FEEDBACK_CORRECTION && velocity_error_mech_rad_per_sec < 0.0f)) {
        velocity_error_integral *= 0.99f;
    }

    // Reset integrator if motor disabled or at target
    if (!m_enabled || (fabs(velocity_error_mech_rad_per_sec) < 0.05f && fabs(target_mechanical_rad_per_sec) < 0.01f)) {
        velocity_error_integral = 0.0f;
    }

    feedback_correction = feedback_correction_clamped;

    // // Debug PI controller
    // g_velocity_error_for_debug = velocity_error_mech_rad_per_sec;
    // g_base_voltage_q_feedforward = base_voltage_q;
    // g_feedback_voltage_correction = feedback_correction;
    // g_velocity_error_integral = velocity_error_integral;
    // g_velocity_correction_p = proportional_term;
    // g_velocity_correction_i = integral_term;
    // g_feedforward_current_q = measured_current_q;

#if 0
    // CRITICAL: Scale feedback correction during accumulated mode and transition
    float feedback_scale = 1.0f;  // Default: full feedback in encoder mode

    if (use_accumulated) {
        feedback_scale = 0.0f;  // No feedback during open-loop accumulated mode
    }
    float desired_voltage_q = base_voltage_q + (feedback_correction * feedback_scale);
 #else
   float desired_voltage_q = base_voltage_q + feedback_correction;
#endif


    // Calculate final voltage command
    //float desired_voltage_q = base_voltage_q + (feedback_correction * feedback_scale);
    g_desired_voltage_q_before_clamp = desired_voltage_q;


    //m_voltage.q = symetric_clamp(desired_voltage_q, m_voltage_limit);
    m_voltage.d = 0.0f;  // No field weakening

    //g_m_voltage_q_in_controller = m_voltage.q;

    //=========================================================================
    // SECTION 6: ANGLE MODE MANAGEMENT (TRANSITION MANAGER)
    //=========================================================================

    // Use same time as mode determination
    uint32_t current_time = current_time_for_mode;

    // Determine if we should use accumulated mode
    //bool use_accumulated = should_use_accumulated_mode(startup_begin_time, current_time);
      // Mode already determined earlier (before PI controller)
      //
    float electrical_angle_radians;

    //-------------------------------------------------------------------------
    // ACCUMULATED MODE
    //-------------------------------------------------------------------------
    if (use_accumulated && target_mechanical_rad_per_sec > 0.05f) {

        g_accumulate_call_count++;

        // Calculate delta time (overflow-safe)
        uint32_t delta_time_us = compute_safe_elapsed_time(last_accumulate_time, current_time);
        float actual_delta_seconds = delta_time_us * 1.0e-6f;
        last_accumulate_time = current_time;

        // Sanity check timestep
        if (actual_delta_seconds <= 0.0f || actual_delta_seconds > 0.001f) {
            actual_delta_seconds = 0.0001f;
        }

        // Accumulate angle based on target velocity
        float mechanical_rad_increment = target_mechanical_rad_per_sec * actual_delta_seconds;
        accumulated_rad_elec += mechanical_to_electrical_radians(mechanical_rad_increment);
        accumulated_rad_elec = _normalizeAngle(accumulated_rad_elec);

        // Get stable Kalman-based encoder angle
        extern volatile float g_kalman_rad_mech;
        float stable_encoder_elec = calculate_stable_encoder_angle(g_kalman_rad_mech);

        // Sync accumulated toward encoder using transition manager
        accumulated_rad_elec = m_transition_manager.sync_accumulated_to_encoder(
            accumulated_rad_elec,
            stable_encoder_elec
        );

        // Calculate angle error for debug
        float angle_error = stable_encoder_elec - accumulated_rad_elec;
        if (angle_error > MY_PI) angle_error -= TWO_PI;
        else if (angle_error < -MY_PI) angle_error += TWO_PI;

        electrical_angle_radians = accumulated_rad_elec;

        // Debug
        extern volatile float g_accumulated_angle;
        g_accumulated_angle = accumulated_rad_elec;
        g_angle_difference_accum_vs_encoder = angle_error;
        g_accumulated_angle_source = 2.0f;
        g_encoder_mode_locked = 0.0f;

        // Capture this accumulated angle for use in next cycle's transition (if we switch modes)
        prev_accumulated_rad_elec = accumulated_rad_elec;

        // Set voltage for accumulated mode (no transition scaling needed)
        m_voltage.q = symetric_clamp(desired_voltage_q, m_voltage_limit);
        m_voltage.d = 0.0f;
        g_m_voltage_q_in_controller = m_voltage.q;

        // Debug PI controller for accumulated mode
        g_velocity_error_for_debug = velocity_error_mech_rad_per_sec;
        g_base_voltage_q_feedforward = base_voltage_q;
        g_feedback_voltage_correction = feedback_correction;
        g_velocity_error_integral = velocity_error_integral;
        g_velocity_correction_p = proportional_term;
        g_velocity_correction_i = integral_term;
        g_feedforward_current_q = measured_current_q;
    }
    //-------------------------------------------------------------------------
    // ENCODER MODE (with transition management)
    //-------------------------------------------------------------------------
    else {

        g_encoder_mode_locked = 1.0f;

        // Track mode transitions
        static bool was_in_accumulated = true;
        static uint32_t encoder_mode_start_iteration = 0;

        // TRANSITION EVENT: First iteration in encoder mode
        if (was_in_accumulated || m_transition_manager.is_transitioning())
        {
            handle_transition_to_encoder_mode(
                accumulated_rad_elec,
                velocity_error_integral,
                Kp_velocity_mech,
                Ki_velocity_mech,
                INTEGRAL_MAX,
                velocity_error_mech_rad_per_sec,
                current_time,
                target_mechanical_rad_per_sec,
                delta_seconds
            );

            was_in_accumulated = false;
            encoder_mode_start_iteration = 0;  // Reset counter
        }

        // CRITICAL: Control integration resumption to prevent windup
        encoder_mode_start_iteration++;

        extern volatile float g_integral_freeze_active;
        extern volatile float g_integration_gain;

        // Update accumulated angle to track blended result
        accumulated_rad_elec = blended_electrical_angle_radians; //electrical_angle_radians;

        // CRITICAL FIX: Set electrical_angle_radians for commutation in encoder mode
        electrical_angle_radians = blended_electrical_angle_radians;

#if 0
// CRITICAL: Smooth transition by removing initial discontinuity
static float last_feedback_correction = 0.0f;
static float feedback_offset = 0.0f;
static bool transition_initialized = false;

if (m_transition_manager.is_transitioning())
{
    extern volatile float g_transition_blend_factor;

    // On first transition iteration, calculate the feedback discontinuity
    if (!transition_initialized) {
        // Calculate the jump we want to prevent
        feedback_offset = feedback_correction - last_feedback_correction;
        transition_initialized = true;
    }

    // Gradually remove the offset over the blend period
    float current_offset = feedback_offset * (1.0f - g_transition_blend_factor);
    float applied_feedback = feedback_correction - current_offset;

    float transition_voltage_q = base_voltage_q + applied_feedback;
    m_voltage.q = symetric_clamp(transition_voltage_q, m_voltage_limit);
}
else
{
    transition_initialized = false;  // Reset for next transition
    last_feedback_correction = feedback_correction;
    m_voltage.q = symetric_clamp(base_voltage_q + feedback_correction, m_voltage_limit);
}

g_m_voltage_q_in_controller = m_voltage.q;

// Debug PI controller for encoder mode (shows scaled feedback during transition)
g_velocity_error_for_debug = velocity_error_mech_rad_per_sec;
g_base_voltage_q_feedforward = base_voltage_q;
g_feedback_voltage_correction = m_transition_manager.is_transitioning() ?
    (feedback_correction * g_transition_blend_factor) : feedback_correction;
g_velocity_error_integral = velocity_error_integral;
g_velocity_correction_p = proportional_term;
g_velocity_correction_i = integral_term;
g_feedforward_current_q = measured_current_q;

#else
// Apply full feedback during transition
m_voltage.q = symetric_clamp(base_voltage_q + feedback_correction, m_voltage_limit);
g_m_voltage_q_in_controller = m_voltage.q;


// Debug PI controller for encoder mode
g_velocity_error_for_debug = velocity_error_mech_rad_per_sec;
g_base_voltage_q_feedforward = base_voltage_q;
g_feedback_voltage_correction = feedback_correction;
g_velocity_error_integral = velocity_error_integral;
g_velocity_correction_p = proportional_term;
g_velocity_correction_i = integral_term;
g_feedforward_current_q = measured_current_q;
#endif

    }

    // Debug angle difference
    float raw_diff = accumulated_rad_elec - m_current_electrical_radians;
    if (raw_diff > MY_PI) raw_diff -= TWO_PI;
    else if (raw_diff < -MY_PI) raw_diff += TWO_PI;
    g_angle_difference_accum_vs_encoder = raw_diff;

    //=========================================================================
    // SECTION 7: PHASE ADVANCE COMPENSATION
    //=========================================================================

    const float PROCESSING_DELAY_SECONDS = 75e-6f;  // 75 microseconds
    float mechanical_velocity = m_sensor.get_mechanical_velocity_rad_per_sec();
    float mechanical_phase_advance = mechanical_velocity * PROCESSING_DELAY_SECONDS;
    float electrical_phase_advance = mechanical_to_electrical_radians(mechanical_phase_advance);

    float angle_with_advance = electrical_angle_radians + electrical_phase_advance;
    angle_with_advance = _normalizeAngle(angle_with_advance);

    // Debug
    g_phase_advance_for_commutation = electrical_phase_advance;
    g_electrical_angle_to_setPhaseVoltage = electrical_angle_radians;

    //=========================================================================
    // SECTION 8: APPLY VOLTAGE TO MOTOR
    //=========================================================================

    setPhaseVoltage(m_voltage.q, m_voltage.d, angle_with_advance);

    //=========================================================================
    // SECTION 9: DEBUG TELEMETRY
    //=========================================================================

    g_electrical_rad_ref = electrical_angle_radians;
    g_velocity_correction = velocity_error_mech_rad_per_sec;
    g_amperage_q = m_amperage.q;
    g_voltage_q = m_voltage.q;
    g_voltage_d = m_voltage.d;

    g_cl_function_exit_count++;
}

//=============================================================================
// HELPER FUNCTIONS (add these to StepperMotor class - private section)
//=============================================================================

//-----------------------------------------------------------------------------
// should_use_accumulated_mode
//-----------------------------------------------------------------------------
// Determines if accumulated angle mode should be used based on startup time
// and calibration state.
//-----------------------------------------------------------------------------
bool StepperMotor::should_use_accumulated_mode(uint32_t startup_begin_time,
                                                uint32_t current_time)
{
    // Check if we're in the 5-second startup window (overflow-safe)
    uint32_t elapsed_time;
    bool time_check_valid = false;

    if (current_time >= startup_begin_time) {
        elapsed_time = current_time - startup_begin_time;
        time_check_valid = true;
    } else if (startup_begin_time != 0) {
        // Overflow occurred
        elapsed_time = (UINT32_MAX - startup_begin_time) + current_time;
        time_check_valid = true;
    }

    // Use accumulated if:
    // - Still in 5-second startup window
    // - OR not yet calibrated
    // - OR startup time not initialized
    return (time_check_valid && elapsed_time < 5000000) ||
           (m_radian_offset_to_electric_zero == NOT_SET) ||
           (startup_begin_time == 0);
}

//-----------------------------------------------------------------------------
// compute_safe_elapsed_time
//-----------------------------------------------------------------------------
// Overflow-safe elapsed time calculation for micros() timestamps
//-----------------------------------------------------------------------------
uint32_t StepperMotor::compute_safe_elapsed_time(uint32_t start_time,
                                                  uint32_t current_time)
{
    if (current_time >= start_time) {
        return current_time - start_time;
    } else {
        // Overflow occurred
        return (UINT32_MAX - start_time) + current_time;
    }
}

//-----------------------------------------------------------------------------
// calculate_stable_encoder_angle
//-----------------------------------------------------------------------------
// Calculates electrical angle from stable Kalman mechanical position
//-----------------------------------------------------------------------------
float StepperMotor::calculate_stable_encoder_angle(float kalman_position)
{
    float stable_mech_angle = kalman_position - m_sensor_offset;

    // Normalize mechanical to [0, 2π)
    float stable_mech_norm = stable_mech_angle - TWO_PI * floorf(stable_mech_angle / TWO_PI);

    // Convert to electrical
    float stable_encoder_elec = mechanical_to_electrical_radians(stable_mech_norm);
    stable_encoder_elec -= m_radian_offset_to_electric_zero;

    return normalize_radians(stable_encoder_elec);
}

//-----------------------------------------------------------------------------
// handle_transition_to_encoder_mode
//-----------------------------------------------------------------------------
// Called once when transitioning from accumulated to encoder mode.
// Prepares transition manager and performs bumpless PI transfer.
//-----------------------------------------------------------------------------
void StepperMotor::handle_transition_to_encoder_mode(
    float accumulated_angle,
    float& velocity_error_integral,
    float Kp_velocity_mech,
    float Ki_velocity_mech,
    float INTEGRAL_MAX,
    float velocity_error_mech_rad_per_sec,
    uint32_t current_time,
    float target_mechanical_rad_per_sec,
    float delta_seconds)
{
    // Capture state before transition
    g_transition_accumulated_angle_before = accumulated_angle;
    g_transition_m_current_electrical_before = m_current_electrical_radians;
    g_transition_voltage_q_before = m_voltage.q;
    g_transition_velocity_error = velocity_error_mech_rad_per_sec;
    g_transition_integral_term = velocity_error_integral;

    // Capture raw sensor values
    extern volatile float g_as5048_angle;
    extern volatile float g_kalman_rad_mech;
    g_transition_g_as5048_angle = g_as5048_angle;
    g_transition_g_cached_encoder_angle = g_cached_encoder_angle;
    g_transition_mech_angle_from_sensor = m_sensor.read_angle_radians_from_buffer();
    g_transition_timestamp = micros();

    // Get Kalman-filtered electrical angle for smooth FOC operation
    float kalman_electrical_radians = m_sensor.get_kalman_electrical_angle_radians(
        m_sensor_offset,
        m_radian_offset_to_electric_zero,
        NUM_POLE_PAIRS);



    // // Prepare transition using transition manager
    // bool transition_ok = m_transition_manager.prepare_transition(
    //     accumulated_angle,
    //     m_current_electrical_radians,
    //     g_kalman_position,
    //     m_sensor_offset,
    //     m_radian_offset_to_electric_zero,
    //     NUM_POLE_PAIRS,
    //     current_time
    // );

    // if (transition_ok)
    // {
    //     // Perform bumpless PI transfer to prevent voltage jump


    //     // CRITICAL: Reset integral before bumpless transfer
    //       // The integral has accumulated to 30 rad during acceleration (clamped at limit)
    //       // Bumpless transfer will recalculate the correct starting value
    //       // to maintain current voltage output without the accumulated windup
    //       velocity_error_integral = 0.0f;

    //     m_transition_manager.bumpless_pi_transfer(
    //         Kp_velocity_mech,
    //         Ki_velocity_mech,
    //         velocity_error_mech_rad_per_sec,
    //         m_voltage.q,  // Maintain current voltage output
    //         INTEGRAL_MAX,
    //         velocity_error_integral  // This will be adjusted
    //     );
    // }

    // Prepare transition using transition manager (idempotent + delayed bumpless)
    extern volatile float g_transition_blend_factor;

    static bool s_transition_prepared = false;
    static bool s_bumpless_done = false;

    // When we are not transitioning anymore, reset internal latches
    if (!m_transition_manager.is_transitioning())
    {
        s_transition_prepared = false;
        s_bumpless_done = false;
    }

    bool transition_ok = true;

    // // Only prepare once (so repeated calls during the transition don't restart it)
    // if (!s_transition_prepared)
    // {
    //     transition_ok = m_transition_manager.prepare_transition(
    //         accumulated_angle,
    //         kalman_electrical_radians, //m_current_electrical_radians,
    //         g_kalman_position,
    //         m_sensor_offset,
    //         m_radian_offset_to_electric_zero,
    //         NUM_POLE_PAIRS,
    //         current_time
    //     );

    //     if (transition_ok)
    //     {
    //         s_transition_prepared = true;

#if 0
            // ------------------------------------------------------------
            // IMMEDIATE bumpless PI transfer at encoder-mode entry
            // This must happen BEFORE blending and before I gain ramps
            // ------------------------------------------------------------
            velocity_error_integral = 0.0f;

            m_transition_manager.bumpless_pi_transfer(
                Kp_velocity_mech,
                Ki_velocity_mech,
                velocity_error_mech_rad_per_sec,
                m_voltage.q,          // last open-loop voltage
                INTEGRAL_MAX,
                velocity_error_integral
            );

            s_bumpless_done = true;
#endif

            // FORCE first blend evaluation immediately
            // This eliminates the dead band before blend starts advancing
            // (void)m_transition_manager.get_blended_angle(
            //     /* encoder_angle        */ kalman_electrical_radians,
            //     /* kalman_mechanical    */ g_kalman_position,
            //     /* sensor_offset        */ m_sensor_offset,
            //     /* elec_zero_offset     */ m_radian_offset_to_electric_zero,
            //     /* num_pole_pairs       */ static_cast<float>(NUM_POLE_PAIRS),
            //     /* target_velocity      */ target_mechanical_rad_per_sec,
            //     /* delta_time_sec       */ delta_seconds
            // );
    //     }
    // }



    // Capture state after transition
    g_transition_accumulated_angle_after = accumulated_angle;
    g_transition_voltage_q_after = m_voltage.q;
    // NOTE: Do NOT write g_transition_blend_factor here.
    // It is owned/updated by AngleTransitionManager::get_blended_angle().



    // Calculate angle jump (normalize both angles first to handle wraparound)
    float normalized_encoder = normalize_radians(kalman_electrical_radians);
    float normalized_accumulated = normalize_radians(accumulated_angle);
    float angle_jump = normalized_encoder - normalized_accumulated;
    if (angle_jump > MY_PI) angle_jump -= TWO_PI;
    else if (angle_jump < -MY_PI) angle_jump += TWO_PI;
    g_transition_angle_jump = angle_jump;
}


//-----------------------------------------------------------------------------
//                              clarkeTransform
//-----------------------------------------------------------------------------
void StepperMotor::clarkeTransform( float Ia,
                            float  Ib,
                            float& Ialpha,
                            float& Ibeta)
{
    Ialpha = Ia;
    Ibeta = (Ia + 2.0f * Ib) * SQUARE_ROOT_OF_3_INVERSE;
}

//-----------------------------------------------------------------------------
//                              clarkeTransform
//
// Function to perform Park Transformation (Rotation)
//-----------------------------------------------------------------------------
void StepperMotor::parkTransform( float  Ialpha,
                          float  Ibeta,
                          float  theta,
                          float& Id,
                          float& Iq)
{
    Id =  Ialpha * cosf(theta) + Ibeta * sinf(theta);
    Iq = -Ialpha * sinf(theta) + Ibeta * cosf(theta);

    //Id =  Ialpha * arm_cos_f32(theta) + Ibeta * arm_sin_f32(theta);
    //Iq = -Ialpha * arm_sin_f32(theta) + Ibeta * arm_cos_f32(theta);
}

//-----------------------------------------------------------------------------
//                              transformCurrents
//-----------------------------------------------------------------------------
void StepperMotor::transformCurrents(
                               float  Ia,
                               float  Ib,
                               float  encoder_angle_radians,
                               float& Id,
                               float& Iq)
{
    float Ialpha, Ibeta;

    // Perform Clarke Transformation
    clarkeTransform(Ia, Ib, Ialpha, Ibeta);

    // Convert encoder angle to radians
    float theta = encoder_angle_radians; // * MY_PI / 180.0f;

    // Perform Park Transformation
    parkTransform(Ialpha, Ibeta, theta, Id, Iq);
}


//-----------------------------------------------------------------------------
//                        compute_inverse_park_transform
//-----------------------------------------------------------------------------
void StepperMotor::
compute_inverse_park_transform(
                                float Uq,
                                float Ud,
                                float electric_angle)
{
    g_park_transform_counter++;

    // Sinusoidal PWM modulation
    // Inverse Park transformation
    float _sa;
    float _ca;

    // CRITICAL: The angle passed in should already be in [0, 2π) range
    // Don't re-normalize unless absolutely necessary, as it might introduce errors
    // DEBUG: Capture BOTH the input angle and validated angle to see what's happening
    g_debug_electric_angle_input = electric_angle;  // Capture input angle

    // DEBUG: Capture input angle to verify it matches g_elec_angle_for_foc
    g_debug_electric_angle_input = electric_angle;  // Capture raw input

    // Debug: Also capture this in alignment-specific variable
    if(g_align_voltage_active > 0.5f)  // Only update during alignment
    {
        g_align_angle_to_inverse_park = electric_angle;
    }

    // CRITICAL: Normalize the angle first to catch cases where angle wraps to 0.0f (e.g., 2π → 0)
    // Then check if normalized angle is 0.0f - if so, skip ALL sin/cos calculations
    // When angle == 0.0f: cos(0) = 1.0 and sin(0) = 0.0, causing jumps from smooth sine wave
    // We must NOT calculate sin(0) or cos(0) to keep signals smooth
    float validated_angle = normalize_radians(electric_angle);

    // CRITICAL: Due to floating point precision, fmod(2π, 2π) might not return exactly 0.0f
    // It might return something like -0.0000001f or 0.0000001f, which normalizes to ~0 or ~2π
    // We need to check if the angle is effectively 0 by checking if it's exactly 0.0f OR exactly 2π
    // OR if it's very close to 0 (which would produce sin/cos values very close to sin(0)/cos(0))
    // When angle == 0.0f or angle == 2π: cos(0) = 1.0 and sin(0) = 0.0, causing jumps from smooth sine wave
    // We must NOT calculate sin(0) or cos(0) to keep signals smooth
    // Check for exactly 0.0f, exactly 2π, or if normalized result is exactly 2π (which means it wrapped from 0)
    // NOTE: In normal operation, angle should only be 0 once per revolution - if we see many 0s,
    // there's a bug elsewhere that's incorrectly setting angle to 0.0f

    // SPECIAL CASE: Handle angle = 0 without calling trig functions
    // For angle = 0: cos(0) = 1, sin(0) = 0
    // We can calculate directly without calling cosf(0) or sinf(0)
    if(validated_angle == 0.0f || validated_angle == TWO_PI)
    {
        g_angle_zero_skip_counter++;
        g_debug_angle_when_zero = electric_angle;

        // Direct calculation for angle = 0: cos(0)=1, sin(0)=0
        float _sa_zero = 0.0f;  // sin(0) = 0
        float _ca_zero = 1.0f;  // cos(0) = 1

        // Update debug variables
        g_park_sin = _sa_zero;
        g_park_cos = _ca_zero;

        // Inverse Park: U_alpha = cos(0)*Ud - sin(0)*Uq = Ud
        //               U_beta  = sin(0)*Ud + cos(0)*Uq = Uq
        m_U_alpha = _ca_zero * Ud - _sa_zero * Uq;  // = 1*Ud - 0*Uq = Ud
        m_U_beta  = _sa_zero * Ud + _ca_zero * Uq;  // = 0*Ud + 1*Uq = Uq

        // Debug: Capture that we used the angle=0 special case
        g_angle_zero_path_used = 1.0f;
        g_angle_zero_Uq_input = Uq;
        g_angle_zero_Ud_input = Ud;
        g_angle_zero_U_alpha_calc = m_U_alpha;
        g_angle_zero_U_beta_calc = m_U_beta;

        return;  // Done, no need for trig calculations
    }


    // Angle is non-zero - proceed with sin/cos calculations
    bool angle_is_nonzero = true;  // We already checked above

    g_debug_electric_angle_for_sincos = validated_angle;  // Capture angle used for sin/cos

    // TEST: Try standard library functions to see if lookup table is the issue
    // CRITICAL: Verify cosf is working correctly - it should return values in [-1, 1] range
    // If validated_angle goes from 0 to 2π, cosf should go from 1 → 0 → -1 → 0 → 1
    // DEBUG: Capture angle IMMEDIATELY before cosf() call to verify relationship
    g_debug_validated_angle_at_cosf = validated_angle;

    // Use sinf() for sine - it works correctly
    // CRITICAL: We only reach here if angle != 0.0f, so we never calculate sin(0) or cos(0)
    _sa = sinf(validated_angle);

    // DEBUG: Capture angle before _cos call
    // CRITICAL: Only update debug variables if angle is non-zero to avoid
    // polluting debug data with zero values from disable/init calls
    extern volatile float g_debug_angle_before_cos;
    if(angle_is_nonzero)
    {
        g_debug_angle_before_cos = validated_angle;
    }

    // For cosine, let's compute it manually using sin(θ + π/2) to see what's happening
    // We'll trace through the exact same logic as _cos but with debug variables
    float cos_input_angle = validated_angle;

    // Normalize if needed (same as _cos does)
    if(cos_input_angle < 0.0f || cos_input_angle >= TWO_PI)
    {
        float normalized = fmodf(cos_input_angle, TWO_PI);
        cos_input_angle = normalized >= 0.0f ? normalized : (normalized + TWO_PI);
    }

    // Compute a_sin = angle + π/2 (same as _cos does)
    float a_sin = cos_input_angle + HALF_PI;

    // Wrap to [0, 2π) (same as _cos does)
    if(a_sin >= TWO_PI)
    {
        a_sin = a_sin - TWO_PI;
    }
    if(a_sin < 0.0f)
    {
        a_sin = a_sin + TWO_PI;
    }

    // Compute cosine using sinf() with the wrapped angle
    float cos_via_sinf = sinf(a_sin);

    // Also try _cos for comparison
    _ca = _cos(validated_angle);

    // DEBUG: Only update debug variables if angle is non-zero to avoid
    // polluting debug data with zero values from disable/init calls
    extern volatile float g_debug_a_sin_before_wrap;
    extern volatile float g_debug_a_sin_after_wrap;
    extern volatile float g_debug_cos_via_sinf;
    if(angle_is_nonzero)
    {
        g_debug_a_sin_before_wrap = cos_input_angle + HALF_PI;  // Before wrapping
        g_debug_a_sin_after_wrap = a_sin;
        g_debug_cos_via_sinf = cos_via_sinf;
    }

    // DEBUG: Capture intermediate values for cosine calculation
    extern volatile float g_debug_cos_angle_before_wrap;
    extern volatile float g_debug_cos_angle_before_wrap_check;
    extern volatile bool g_debug_cos_angle_needs_wrap;
    extern volatile float g_debug_cos_angle_after_wrap;
    extern volatile float g_debug_cos_angle_before_sinf;
    float cos_angle_unwrapped = validated_angle + HALF_PI;
    float cos_angle = cos_angle_unwrapped;
    while(cos_angle >= TWO_PI) cos_angle = cos_angle - TWO_PI;
    while(cos_angle < 0.0f) cos_angle = cos_angle + TWO_PI;
    g_debug_cos_angle_before_wrap = cos_angle_unwrapped;
    g_debug_cos_angle_before_wrap_check = cos_angle_unwrapped;
    g_debug_cos_angle_needs_wrap = (cos_angle_unwrapped >= TWO_PI);
    g_debug_cos_angle_after_wrap = cos_angle;
    g_debug_cos_angle_before_sinf = cos_angle;

    // DEBUG: Verify the phase relationship is correct
    extern volatile float g_debug_sinf_result;
    extern volatile float g_debug_cosf_result;
    g_debug_sinf_result = _sa;
    g_debug_cosf_result = _ca;  // This should now show -1 to 1 range and oscillate around 0
    //arm_sin_cos_f32(electric_angle, &_sa, &_ca);
    //sincos(electric_angle, &_sa, &_ca);

    // Debug: capture sin/cos values
    // CRITICAL: We only reach here if angle != 0.0f (early return above handles angle == 0.0f)
    // So we can always update since we never calculate sin(0) or cos(0)
    g_park_sin = _sa;
    g_park_cos = _ca;

    // Inverse park transform
    // DEBUG: Capture inputs to inverse park transform
    extern volatile float g_debug_Uq_input;
    extern volatile float g_debug_Ud_input;
    extern volatile float g_debug_U_beta_calc_sa;
    extern volatile float g_debug_U_beta_calc_ca;
    extern volatile float g_debug_U_beta_term1;  // _sa * Ud
    extern volatile float g_debug_U_beta_term2;  // _ca * Uq
    if(angle_is_nonzero)
    {
        g_debug_Uq_input = Uq;
        g_debug_Ud_input = Ud;
        g_debug_U_beta_calc_sa = _sa;
        g_debug_U_beta_calc_ca = _ca;
        g_debug_U_beta_term1 = _sa * Ud;
        g_debug_U_beta_term2 = _ca * Uq;
    }

    // Calculate m_U_beta step by step for debugging
    // CRITICAL: Capture the exact values AT THE MOMENT OF CALCULATION
    // Only capture when angle is non-zero to avoid polluting with cos(0)=1 values
    extern volatile float g_debug_sa_for_term1;
    extern volatile float g_debug_Ud_for_term1;
    extern volatile float g_debug_ca_for_term2;
    extern volatile float g_debug_Uq_for_term2;
    extern volatile float g_debug_ca_at_calc;
    extern volatile float g_debug_sa_at_calc;
    extern volatile float g_debug_Ud_at_calc;
    extern volatile float g_debug_Uq_at_calc;

    // Capture ALL values at the exact same moment, right before calculation
    // CRITICAL: Guard debug variables to prevent zero-sample pollution (cos(0)=1)
    // Only update when angle is non-zero, same as g_park_cos
    if(angle_is_nonzero)
    {
        g_debug_sa_for_term1 = _sa;
        g_debug_Ud_for_term1 = Ud;
        g_debug_ca_for_term2 = _ca;
        g_debug_Uq_for_term2 = Uq;
        g_debug_ca_at_calc = _ca;  // Should match g_debug_ca_for_term2 and g_park_cos
        g_debug_sa_at_calc = _sa;  // Should match g_debug_sa_for_term1 and g_park_sin
        g_debug_Ud_at_calc = Ud;   // Should match g_debug_Ud_for_term1
        g_debug_Uq_at_calc = Uq;   // Should match g_debug_Uq_for_term2
    }

    float term1_calc = _sa * Ud;
    float term2_calc = _ca * Uq;
    extern volatile float g_debug_term1_direct;
    extern volatile float g_debug_term2_direct;
    extern volatile float g_debug_term1_verify;
    extern volatile float g_debug_term2_verify;
    extern volatile float g_debug_m_U_beta_calc_direct;
    extern volatile float g_debug_m_U_beta_calc_verify;

    // Calculate m_U_beta using the same method as g_debug_m_U_beta_calc_direct
    // This ensures m_U_beta matches the smooth sine wave calculation
    float m_U_beta_calc = term1_calc + term2_calc;

    // CRITICAL: Store clean U_beta value for debug plots (only when angle is non-zero)
    // This prevents zero-sample pollution (cos(0)=1) from making the plots look wrong
    extern volatile float g_clean_U_beta_for_plot;
    if(angle_is_nonzero)
    {
        g_clean_U_beta_for_plot = m_U_beta_calc;
    }

    // Only update debug variables when angle is non-zero (same guard as above)
    // This prevents zero-sample pollution (cos(0)=1) from making the plots look wrong
    if(angle_is_nonzero)
    {
        g_debug_term1_direct = term1_calc;
        g_debug_term2_direct = term2_calc;

        // DEBUG: Verify the calculation step by step
        g_debug_term1_verify = g_debug_sa_for_term1 * g_debug_Ud_for_term1;
        g_debug_term2_verify = g_debug_ca_for_term2 * g_debug_Uq_for_term2;

        // Calculate the sum directly for comparison (inside guard for debug)
        float m_U_beta_calc_debug = term1_calc + term2_calc;
        g_debug_m_U_beta_calc_direct = m_U_beta_calc_debug;

        // DEBUG: Also calculate using the verified terms
        g_debug_m_U_beta_calc_verify = g_debug_term1_verify + g_debug_term2_verify;
    }

    // CRITICAL: We only reach here if angle != 0.0f (early return above handles angle == 0.0f)
    // So we can always update since we never calculate sin(0) or cos(0)
    // Motor controller needs smooth values, not discontinuities
    m_U_alpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
    m_U_beta = term1_calc + term2_calc;  // Identical calculation to g_debug_m_U_beta_calc_direct

    // DEBUG: Capture m_U_beta immediately after calculation for comparison
    // Always capture (no guard) to see all values including zeros
    extern volatile float g_debug_m_U_beta_after_calc;
    g_debug_m_U_beta_after_calc = m_U_beta;

    // DEBUG: Compare the direct calculation with the member variable
    extern volatile float g_debug_m_U_beta_diff;
    g_debug_m_U_beta_diff = m_U_beta - m_U_beta_calc;

    //m_U_alpha = -sinf(electric_angle) * Uq;
    //m_U_beta  =  cosf(electric_angle) * Uq;

    return;
}


//-----------------------------------------------------------------------------
//                         setPhaseVoltage
//
// Method using FOC to set Uq and Ud to the motor at the optimal angle
//
// Function implementing Sine PWM algorithms
// - space vector not implemented yet
//
// Function uses sine approximation
//    regular  sin +  cos ~300us    (no memory usaage)
//    approx  _sin + _cos ~110us    (400Byte ~ 20% of memory)
//
//! @param Uq
//! @param Ud
//! @param angle_el
//-----------------------------------------------------------------------------
void StepperMotor::
setPhaseVoltage(float Uq, float Ud, float electric_angle)
{
  // Debug: Count calls to this function
  g_setPhaseVoltage_call_count++;

  g_motor_enabled = m_enabled;

  // Debug: Track motor enabled status (should be true for PWM to work)
  g_motor_enabled_status = m_enabled ? 1 : 0;

  // Debug: Capture inputs to setPhaseVoltage
  g_Uq_to_setPhaseVoltage = Uq;
  g_Ud_to_setPhaseVoltage = Ud;
  g_angle_to_setPhaseVoltage = electric_angle;

  // CRITICAL: Always compute the inverse park transform for debugging,
  // even if motor is disabled, so we can see the sin/cos values
  // CRITICAL: electric_angle is already normalized in update_speed_open_loop
  // Pass it through directly without re-normalization to avoid introducing errors
  // The multiple normalization checks were causing the angle to be incorrectly modified
  compute_inverse_park_transform( Uq, Ud, electric_angle);

  // TEMPORARY FIX: Force motor enabled for closed-loop testing
  // This bypasses the m_enabled check to diagnose PWM issue
  // Debug: Capture U_alpha and U_beta before PWM conversion
  g_U_alpha_debug = m_U_alpha;
  g_U_beta_debug = m_U_beta;

  if(true)  // Always execute PWM (was: if(m_enabled))
  {
      // set the voltages in hardware
      //m_driver.set_pwm_duty_cycle(m_U_alpha, m_U_beta);

      m_driver.set_pwm_duty_cycle( m_U_alpha,
                                   m_U_beta,
                                   m_hifactor_a,
                                   m_lofactor_a,
                                   m_hifactor_b,
                                   m_lofactor_b);
  }


  // Debug: monitor alpha/beta outputs
  // CRITICAL: Always capture m_U_beta right before assignment to g_U_beta
  // This will show if m_U_beta is correct when assigned
  g_debug_m_U_beta_before_assign = m_U_beta;  // Always capture for debugging

  // Guard debug variables to prevent zero-sample pollution in plots
  // Only update debug variables when angle != 0.0f, same as g_park_cos and g_debug_ca_at_calc
  bool angle_is_nonzero_setphase = (electric_angle != 0.0f);
  if(angle_is_nonzero_setphase)
  {
      g_debug_m_U_beta_at_assign = m_U_beta;
  }

  // CRITICAL: Only update g_U_beta when angle != 0 to match g_debug_m_U_beta_calc_direct behavior
  // Updating when angle=0 causes discontinuity because m_U_beta = Uq at angle=0, which jumps from smooth sine wave
  // g_debug_m_U_beta_calc_direct only updates when angle != 0, so g_U_beta must do the same
  // CRITICAL: Use same condition as m_U_beta and g_park_cos for consistency
  // Use the same validated_angle check that compute_inverse_park_transform uses
  g_U_alpha = m_U_alpha;

  // CRITICAL: Always update g_U_beta from m_U_beta to ensure it reflects the calculated value
  // The conditional update in compute_inverse_park_transform already handles the angle=0 case
  // We need g_U_beta to always show what m_U_beta contains (which is only updated when angle != 0)
  g_U_beta = m_U_beta;
}

//-----------------------------------------------------------------------------
//                              angleOpenloop
//
// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and m_velocity_limit variables
//
// TODO: Remove magic numbers
//-----------------------------------------------------------------------------
float StepperMotor::angleOpenloop(float target_angle)
{
    // get m_amperage timestamp
    unsigned long now_us = micros();

    // calculate the sample time from last call
    float timestamp = (now_us - m_prev_open_loop_timestamp_us) * 1e-6f;
    // quick fix for strange cases (micros overflow + timestamp not defined)
    if(timestamp <= 0 || timestamp > 0.5f)
    {
      timestamp = 1e-3f;
    }

    // calculate the necessary angle to move from m_amperage position towards
    // m_target angle with maximal velocity (m_velocity_limit)

    // Note: m_velocity_limit may have been modified by absoluteZeroSearch()


    float shaft_radians;
    if(!m_sensor.fetch_radians(shaft_radians))
    {
        return 0.0f;
    }

    float target_angle_error(   target_angle - shaft_radians );
    float delta_angle_measured(     m_velocity_limit * timestamp    );

    if( abs( target_angle_error ) > abs( delta_angle_measured ))
    {
      m_shaft_angle += _sign(target_angle_error) * (abs( m_velocity_limit ) * timestamp);

      m_omega_mechanical_rps = m_velocity_limit;
    }
    else
    {
      m_shaft_angle          = target_angle;
      m_omega_mechanical_rps = 0;
    }

    // If PHASE_RESISTANCE is set use m_voltage_limit, otherwise use m_current_limit
    float Uq = m_voltage_limit;
    if(NOT_SET != resistance(0.0f))
    {
      // in voltage mode
      // the voltage limit is equal to the current limit * the phase resistance
      Uq = m_current_limit*resistance(0.0f) + fabs(m_voltage_bemf);
      Uq = symetric_clamp(Uq, m_voltage_limit);

 //     m_amperage.q = (Uq - fabs(m_voltage_bemf)) / resistance(0.0f);
    }

    // set the maximal allowed m_voltage (voltage_limit) with the necessary angle
    setPhaseVoltage(Uq,  0, mechanical_to_electrical_radians(m_shaft_angle));

    m_prev_open_loop_timestamp_us = now_us;  // save timestamp for next call

    return Uq;
}

//-----------------------------------------------------------------------------
//                         get_filtered_shaft_angle
//-----------------------------------------------------------------------------
float StepperMotor::get_filtered_shaft_angle()
{
    // float radians = m_LPF_angle( m_sensor.get_angle_radians());
    float radians;
    if(m_sensor.fetch_radians(radians))
    {
        if(Direction::CCW == m_sensor_direction)
        {
           radians *= -1.0f;
        }

        // m_sensor_offset is currently 0
        return  radians - m_sensor_offset;
    }
    else
    {
       return 0.0f;
    }
}

//-----------------------------------------------------------------------------
//                              convert_count_to_shaft_angle
//-----------------------------------------------------------------------------
float StepperMotor::convert_count_to_shaft_angle(uint16_t count)
{
    const uint16_t COUNTS_PER_REVOLUTION(0x4000);

    float radians = TWO_PI
                  * static_cast<float>(count)
                  / static_cast<float>(COUNTS_PER_REVOLUTION);

    //float result = (m_sensor.is_direction_invert())
    //             ? -radians
    //             :  radians;

 //   if(Direction::CCW == m_sensor_direction)
 //   {
 //      radians *= -1.0f;
 //   }

    // m_sensor_offset is currently 0
    return  /*normalize_radians*/(radians - m_sensor_offset);
}

#if 0
//-----------------------------------------------------------------------------
//                         shaft_radians_per_second
//-----------------------------------------------------------------------------
float StepperMotor::shaft_radians_per_second()
{
  float rad_per_sec = m_LPF_velocity(m_sensor.get_radians_per_second());

  return (Direction::CCW ==m_sensor_direction) ? -rad_per_sec : rad_per_sec;
  //return rad_per_sec;
}
#endif

//-----------------------------------------------------------------------------
//                        get_phase_angle_radians (electrical)
//-----------------------------------------------------------------------------
float StepperMotor::get_electric_angle_radians()
{
    //float direction       = static_cast<float>(m_sensor_direction);

    //float electic_radians = mechanical_to_electrical_radians(
    //                           m_sensor.get_mechanical_phase_angle_radians()) ;

    float mech_angle = m_sensor.get_mechanical_phase_angle_radians();
    float electic_radians = mechanical_to_electrical_radians(mech_angle);

    // Debug: Expose raw angles and calibration offset for monitoring
    extern volatile float g_encoder_mech_angle_raw;
    extern volatile float g_encoder_elec_angle_raw;
    extern volatile float g_calibration_offset_elec;
    extern volatile float g_encoder_elec_after_offset;
    extern volatile uint32_t g_get_electric_angle_call_count;

    g_get_electric_angle_call_count++;  // Track how often this is called
    g_encoder_mech_angle_raw = mech_angle;
    g_encoder_elec_angle_raw = electic_radians;
    g_calibration_offset_elec = m_radian_offset_to_electric_zero;

    // m_radian_offset_to_electric_zero is already in electrical radians
    // Direction inversion now happens in get_mechanical_phase_angle_radians()
    // so calibration offset is calculated with corrected direction
    float raw_angle = electic_radians - m_radian_offset_to_electric_zero;
    float result = normalize_radians( raw_angle );

    g_encoder_elec_after_offset = result;

    return result;
}

//-----------------------------------------------------------------------------
//                          rotorFieldAlignment
//-----------------------------------------------------------------------------
void StepperMotor::rotorFieldAlignment(double targetPosition)
{
    // Initialize motor and position sensor
    float currentPosition = 0.0f; // Current rotor position (radians)
    float voltage         = 0.0f; // Motor voltage (V)

    // Specify the alignment speed and acceleration
    float alignmentSpeed        = 2 * MY_PI; // radians per second
 //   float alignmentAcceleration = MY_PI;     // radians per second squared


const int ITERATION_LIMIT(100);
 int iteration_cont(0);

    // Loop until the target position is reached
    while (( std::abs(currentPosition - targetPosition) > 0.01)
            && (iteration_cont++ < ITERATION_LIMIT)
            )
    {
        // Calculate error between current and target position
        float positionError = targetPosition - currentPosition;

        // Calculate the required torque (proportional control)
        float torque = positionError * m_current_limit;

        // Apply voltage to align the rotor field
        voltage = torque / (alignmentSpeed * NUM_POLE_PAIRS);

        // Ensure voltage doesn't exceed the maximum
        voltage = symetric_clamp(voltage, m_voltage_limit);

        // Simulate motor response (in a real system, this would control the motor)
        // Update the current position (this is simplified)
        //currentPosition += alignmentSpeed * 0.001; // Assuming a 1 ms time step
       // m_sensor.update();
        currentPosition = m_sensor.read_angle_radians();

        // Print current position for demonstration purposes
        //char  buff[128];
        //sprintf(buff, "Current Position: %ld radians\r\n", static_cast<uint32_t>(1000.0f *currentPosition));
       // HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(buff), strlen(buff), HAL_MAX_DELAY);

        HAL_Delay(100);


        setPhaseVoltage(voltage, 0.0f, mechanical_to_electrical_radians(currentPosition));


    }

    // Stop motor (set voltage to zero)
    voltage = 0.0f;

    // Print message when alignment is complete
    //char  buff[128];
    //sprintf(buff, "Rotor Field Alignment Complete\r\n");
    //HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(buff), strlen(buff), HAL_MAX_DELAY);

}
