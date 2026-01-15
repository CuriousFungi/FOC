//=============================================================================
//                    Velocity Estimation Test & Diagnostics
//
// This file provides diagnostic functions to evaluate Kalman filter
// velocity estimation performance across different speed ranges.
//
// Usage:
//   1. Call velocity_diagnostics_init() once at startup
//   2. Call velocity_diagnostics_update() in your main control loop
//   3. Monitor g_velocity_diag_* variables in STM32CubeMonitor
//=============================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>

//-----------------------------------------------------------------------------
//                          Diagnostic Variables
//-----------------------------------------------------------------------------

// Test state tracking
volatile float g_velocity_diag_target_speed = 0.0f;
volatile float g_velocity_diag_actual_speed = 0.0f;
volatile float g_velocity_diag_speed_error = 0.0f;
volatile float g_velocity_diag_noise_level = 0.0f;
volatile float g_velocity_diag_settling_time = 0.0f;
volatile uint32_t g_velocity_diag_sample_count = 0;
volatile uint8_t g_velocity_diag_test_phase = 0;

// Statistics over measurement window
volatile float g_velocity_diag_mean = 0.0f;
volatile float g_velocity_diag_std_dev = 0.0f;
volatile float g_velocity_diag_min = 0.0f;
volatile float g_velocity_diag_max = 0.0f;
volatile float g_velocity_diag_peak_to_peak = 0.0f;

// Performance metrics
volatile float g_velocity_diag_snr_db = 0.0f;  // Signal-to-noise ratio
volatile float g_velocity_diag_response_time_ms = 0.0f;
volatile float g_velocity_diag_overshoot_percent = 0.0f;

//-----------------------------------------------------------------------------
//                          Internal State
//-----------------------------------------------------------------------------

namespace {
    constexpr uint32_t WINDOW_SIZE = 1000;  // Number of samples for statistics
    float velocity_history[WINDOW_SIZE];
    uint32_t history_index = 0;
    bool window_filled = false;
    
    uint32_t test_start_time = 0;
    float test_start_velocity = 0.0f;
    bool settling_detected = false;
    
    // Test sequence speeds (rad/s)
    constexpr float TEST_SPEEDS[] = {
        0.5f,   // Very low speed
        1.0f,   // Low speed
        2.0f,   // Low-medium speed
        4.0f,   // Medium speed (where noise was observed)
        6.0f,   // High speed
        10.0f,  // Very high speed
        15.0f,  // Maximum test speed
        0.0f    // Return to zero
    };
    constexpr uint32_t NUM_TEST_SPEEDS = sizeof(TEST_SPEEDS) / sizeof(TEST_SPEEDS[0]);
    
    uint32_t current_test_index = 0;
    uint32_t test_hold_time = 0;
    constexpr uint32_t HOLD_DURATION_MS = 5000;  // Hold each speed for 5 seconds
}

//-----------------------------------------------------------------------------
//                          Helper Functions
//-----------------------------------------------------------------------------

// Calculate mean of velocity history
static float calculate_mean() {
    float sum = 0.0f;
    uint32_t count = window_filled ? WINDOW_SIZE : history_index;
    
    for (uint32_t i = 0; i < count; i++) {
        sum += velocity_history[i];
    }
    
    return (count > 0) ? (sum / static_cast<float>(count)) : 0.0f;
}

// Calculate standard deviation
static float calculate_std_dev(float mean) {
    float sum_squared_diff = 0.0f;
    uint32_t count = window_filled ? WINDOW_SIZE : history_index;
    
    for (uint32_t i = 0; i < count; i++) {
        float diff = velocity_history[i] - mean;
        sum_squared_diff += diff * diff;
    }
    
    float variance = (count > 1) ? (sum_squared_diff / static_cast<float>(count - 1)) : 0.0f;
    return sqrtf(variance);
}

// Find min and max in history
static void calculate_min_max(volatile float& min_val, volatile float& max_val) {
    uint32_t count = window_filled ? WINDOW_SIZE : history_index;
    
    if (count == 0) {
        min_val = max_val = 0.0f;
        return;
    }
    
    min_val = max_val = velocity_history[0];
    
    for (uint32_t i = 1; i < count; i++) {
        if (velocity_history[i] < min_val) min_val = velocity_history[i];
        if (velocity_history[i] > max_val) max_val = velocity_history[i];
    }
}

//-----------------------------------------------------------------------------
//                          Public Functions
//-----------------------------------------------------------------------------

/**
 * Initialize velocity diagnostics system
 * Call once at startup
 */
extern "C" void velocity_diagnostics_init(void) {
    // Clear history buffer
    for (uint32_t i = 0; i < WINDOW_SIZE; i++) {
        velocity_history[i] = 0.0f;
    }
    
    history_index = 0;
    window_filled = false;
    current_test_index = 0;
    test_hold_time = 0;
    settling_detected = false;
    
    // Reset diagnostic variables
    g_velocity_diag_target_speed = 0.0f;
    g_velocity_diag_actual_speed = 0.0f;
    g_velocity_diag_sample_count = 0;
    g_velocity_diag_test_phase = 0;
}

/**
 * Update velocity diagnostics with new measurement
 * 
 * @param measured_velocity - Current velocity measurement from Kalman filter (rad/s)
 * @param target_velocity - Current velocity setpoint (rad/s)
 * @param timestamp_ms - Current timestamp in milliseconds
 */
extern "C" void velocity_diagnostics_update(float measured_velocity, 
                                           float target_velocity,
                                           uint32_t timestamp_ms) {
    // Store velocity in circular buffer
    velocity_history[history_index] = measured_velocity;
    history_index++;
    
    if (history_index >= WINDOW_SIZE) {
        history_index = 0;
        window_filled = true;
    }
    
    g_velocity_diag_sample_count++;
    g_velocity_diag_actual_speed = measured_velocity;
    g_velocity_diag_target_speed = target_velocity;
    
    // Calculate error
    g_velocity_diag_speed_error = target_velocity - measured_velocity;
    
    // Update statistics every 100 samples to reduce CPU load
    if (g_velocity_diag_sample_count % 100 == 0) {
        // Calculate mean
        g_velocity_diag_mean = calculate_mean();
        
        // Calculate standard deviation (noise level)
        g_velocity_diag_std_dev = calculate_std_dev(g_velocity_diag_mean);
        g_velocity_diag_noise_level = g_velocity_diag_std_dev;
        
        // Find min/max and peak-to-peak
        calculate_min_max(g_velocity_diag_min, g_velocity_diag_max);
        g_velocity_diag_peak_to_peak = g_velocity_diag_max - g_velocity_diag_min;
        
        // Calculate SNR (signal-to-noise ratio) in dB
        if (g_velocity_diag_std_dev > 0.001f && fabsf(g_velocity_diag_mean) > 0.01f) {
            float snr_linear = fabsf(g_velocity_diag_mean) / g_velocity_diag_std_dev;
            g_velocity_diag_snr_db = 20.0f * log10f(snr_linear);
        } else {
            g_velocity_diag_snr_db = 0.0f;
        }
    }
    
    // Detect settling time (time to reach within 5% of target)
    if (!settling_detected && fabsf(g_velocity_diag_speed_error) < 0.05f * fabsf(target_velocity)) {
        if (target_velocity != test_start_velocity) {
            g_velocity_diag_response_time_ms = static_cast<float>(timestamp_ms - test_start_time);
            settling_detected = true;
        }
    }
    
    // Calculate overshoot during transients
    if (target_velocity > test_start_velocity) {
        // Accelerating - check for overshoot
        if (measured_velocity > target_velocity) {
            float overshoot = measured_velocity - target_velocity;
            float percent = (overshoot / target_velocity) * 100.0f;
            if (percent > g_velocity_diag_overshoot_percent) {
                g_velocity_diag_overshoot_percent = percent;
            }
        }
    } else if (target_velocity < test_start_velocity) {
        // Decelerating - check for undershoot
        if (measured_velocity < target_velocity) {
            float undershoot = target_velocity - measured_velocity;
            float percent = (undershoot / fabsf(target_velocity)) * 100.0f;
            if (percent > g_velocity_diag_overshoot_percent) {
                g_velocity_diag_overshoot_percent = percent;
            }
        }
    }
}

/**
 * Run automatic test sequence
 * Cycles through predefined speeds to evaluate filter performance
 * 
 * @param timestamp_ms - Current timestamp in milliseconds
 * @return Next target velocity for the motor controller
 */
extern "C" float velocity_diagnostics_auto_test(uint32_t timestamp_ms) {
    // Check if it's time to move to next test speed
    uint32_t elapsed = timestamp_ms - test_hold_time;
    
    if (elapsed >= HOLD_DURATION_MS) {
        // Move to next test speed
        current_test_index++;
        
        if (current_test_index >= NUM_TEST_SPEEDS) {
            current_test_index = 0;  // Loop back to start
        }
        
        // Update test phase
        g_velocity_diag_test_phase = static_cast<uint8_t>(current_test_index);
        
        // Record transition time
        test_start_time = timestamp_ms;
        test_start_velocity = g_velocity_diag_actual_speed;
        test_hold_time = timestamp_ms;
        settling_detected = false;
        g_velocity_diag_overshoot_percent = 0.0f;
        
        // Clear statistics for new speed
        history_index = 0;
        window_filled = false;
    }
    
    return TEST_SPEEDS[current_test_index];
}

/**
 * Manual test - set specific speed and evaluate performance
 * 
 * @param target_speed - Desired test speed (rad/s)
 * @param timestamp_ms - Current timestamp in milliseconds
 */
extern "C" void velocity_diagnostics_manual_test(float target_speed, uint32_t timestamp_ms) {
    // Check if speed changed
    if (fabsf(target_speed - g_velocity_diag_target_speed) > 0.1f) {
        // Speed changed - reset metrics
        test_start_time = timestamp_ms;
        test_start_velocity = g_velocity_diag_actual_speed;
        settling_detected = false;
        g_velocity_diag_overshoot_percent = 0.0f;
        
        // Clear statistics
        history_index = 0;
        window_filled = false;
    }
}

/**
 * Get diagnostic summary report
 * Returns formatted string with key metrics (for debugging)
 */
extern "C" void velocity_diagnostics_print_report(char* buffer, uint32_t buffer_size) {
    if (buffer == nullptr || buffer_size < 200) return;
    
    snprintf(buffer, buffer_size,
             "Velocity Diagnostics Report:\n"
             "  Target: %.2f rad/s\n"
             "  Actual: %.2f rad/s (mean: %.2f)\n"
             "  Error: %.3f rad/s\n"
             "  Noise (std): %.4f rad/s\n"
             "  Peak-to-Peak: %.4f rad/s\n"
             "  SNR: %.1f dB\n"
             "  Response Time: %.1f ms\n"
             "  Overshoot: %.1f %%\n"
             "  Samples: %lu\n",
             g_velocity_diag_target_speed,
             g_velocity_diag_actual_speed,
             g_velocity_diag_mean,
             g_velocity_diag_speed_error,
             g_velocity_diag_noise_level,
             g_velocity_diag_peak_to_peak,
             g_velocity_diag_snr_db,
             g_velocity_diag_response_time_ms,
             g_velocity_diag_overshoot_percent,
             (unsigned long)g_velocity_diag_sample_count);
}

/**
 * Get pass/fail assessment based on performance criteria
 * 
 * @return 1 if performance is acceptable, 0 if failed
 */
extern "C" int velocity_diagnostics_check_performance(void) {
    // Define acceptance criteria
    const float MAX_NOISE_AT_LOW_SPEED = 0.05f;    // 50 mrad/s at < 2 rad/s
    const float MAX_NOISE_AT_HIGH_SPEED = 0.15f;   // 150 mrad/s at > 4 rad/s
    const float MIN_SNR_DB = 30.0f;                // Minimum 30dB SNR
    const float MAX_SETTLING_TIME_MS = 500.0f;     // Must settle within 500ms
    const float MAX_OVERSHOOT_PERCENT = 10.0f;     // Max 10% overshoot
    
    int pass = 1;
    
    // Check noise level based on speed
    float speed_abs = fabsf(g_velocity_diag_actual_speed);
    float max_allowed_noise = (speed_abs < 2.0f) ? MAX_NOISE_AT_LOW_SPEED : MAX_NOISE_AT_HIGH_SPEED;
    
    if (g_velocity_diag_noise_level > max_allowed_noise) {
        pass = 0;  // Fail: excessive noise
    }
    
    // Check SNR
    if (g_velocity_diag_snr_db < MIN_SNR_DB && speed_abs > 0.5f) {
        pass = 0;  // Fail: poor SNR
    }
    
    // Check settling time
    if (settling_detected && g_velocity_diag_response_time_ms > MAX_SETTLING_TIME_MS) {
        pass = 0;  // Fail: too slow
    }
    
    // Check overshoot
    if (g_velocity_diag_overshoot_percent > MAX_OVERSHOOT_PERCENT) {
        pass = 0;  // Fail: excessive overshoot
    }
    
    return pass;
}

//=============================================================================
//                          Integration Instructions
//=============================================================================
/*

To integrate this diagnostic system into your motor control code:

1. Add to your initialization function (e.g., in main.cpp):
   
   velocity_diagnostics_init();


2. Add to your control loop (e.g., in StepperMotor::update_speed_closed_loop):
   
   extern void velocity_diagnostics_update(float, float, uint32_t);
   
   // After calculating measured_mechanical_rad_per_sec:
   velocity_diagnostics_update(
       measured_mechanical_rad_per_sec,    // Kalman velocity
       m_target_rps,                       // Target velocity
       HAL_GetTick()                       // Current time (ms)
   );


3. For automatic testing, replace your manual speed commands with:
   
   extern float velocity_diagnostics_auto_test(uint32_t);
   
   float target_speed = velocity_diagnostics_auto_test(HAL_GetTick());
   // Use target_speed as your velocity command


4. Monitor these variables in STM32CubeMonitor:
   - g_velocity_diag_noise_level (should be low, < 0.1 rad/s)
   - g_velocity_diag_snr_db (should be > 30 dB)
   - g_velocity_diag_response_time_ms (should be < 500 ms)
   - g_velocity_diag_overshoot_percent (should be < 10%)


5. Performance acceptance test:
   
   extern int velocity_diagnostics_check_performance(void);
   
   if (velocity_diagnostics_check_performance()) {
       // Kalman filter is performing well
   } else {
       // Need to retune Kalman filter parameters
   }

*/