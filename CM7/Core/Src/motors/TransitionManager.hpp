#ifndef ANGLE_TRANSITION_MANAGER_HPP
#define ANGLE_TRANSITION_MANAGER_HPP

#include <stdint.h>
#include <cmath>

// Forward declare micros() function
extern "C" uint32_t micros(void);

//=============================================================================
// Angle Transition Manager
//=============================================================================
// Manages the transition from accumulated angle mode to encoder mode for
// FOC motor control. Provides smooth angle synchronization and bumpless
// voltage transfer to prevent motor stalls.
//
// Usage:
//   1. Initialize: AngleTransitionManager transition;
//   2. Call sync_accumulated_to_encoder() periodically during accumulated mode
//   3. Call prepare_transition() just before switching modes
//   4. Call get_blended_angle() during transition period
//   5. Call bumpless_pi_transfer() to synchronize PI controller state
//=============================================================================

class AngleTransitionManager {
public:
    //-------------------------------------------------------------------------
    // Configuration Constants
    //-------------------------------------------------------------------------
    static constexpr float BLEND_DURATION_SEC = 2.0f;      // 2 second smooth transition
    static constexpr float SYNC_GAIN = 0.01f; //.002f;             // Gentle nudge toward encoder
    static constexpr float TRANSITION_THRESHOLD_RAD = 0.1f; // Max angle error to allow transition
    static constexpr float MY_PI = 3.14159265359f;
    static constexpr float TWO_PI = 6.28318530718f;

    //-------------------------------------------------------------------------
    // Transition State
    //-------------------------------------------------------------------------
    enum class TransitionState {
        ACCUMULATED_MODE,    // Using accumulated angle only
        TRANSITIONING,       // Blending from accumulated to encoder
        ENCODER_MODE        // Using encoder angle only
    };

    //-------------------------------------------------------------------------
    // Constructor
    //-------------------------------------------------------------------------
    AngleTransitionManager()
        : m_accumulated_rad_elec(0.0f)
        , m_angle_rad_elec(0.0f)
        , m_transition_start_us(0)
        , m_transition_state(TransitionState::ACCUMULATED_MODE)
        , m_kalman_phase_err_rad_elec(0.0f)
    {}

    //-------------------------------------------------------------------------
    // sync_accumulated_to_encoder
    //-------------------------------------------------------------------------
    // Continuously nudges accumulated angle toward encoder during accumulation
    // mode. Prevents large angle discontinuities at transition.
    //
    // Call this every control loop iteration while in accumulated mode.
    //
    // @param accumulated      Current accumulated angle (electrical radians)
    // @param encoder_stable   Stable encoder angle (e.g., from Kalman filter)
    // @return                 Synchronized accumulated angle
    //-------------------------------------------------------------------------
    float sync_accumulated_to_encoder(float accumulated, float encoder_stable) {
        // Calculate shortest angular error
        float angle_error = encoder_stable - accumulated;

        // Unwrap to find shortest path [-π, π]
        if (angle_error > MY_PI)
            angle_error -= TWO_PI;
        else if (angle_error < -MY_PI)
            angle_error += TWO_PI;

        // Apply small nudge toward encoder
        accumulated += SYNC_GAIN * angle_error;

        // Normalize to [0, 2π)
        accumulated = normalize_angle(accumulated);

        // Store for transition
        m_accumulated_rad_elec = accumulated;

        return accumulated;
    }

    //-------------------------------------------------------------------------
    // prepare_transition
    //-------------------------------------------------------------------------
    // Called once when transitioning from accumulated to encoder mode.
    // Calculates phase offset and initializes blend timer.
    //
    // @param accumulated_angle      Final accumulated angle before transition
    // @param encoder_angle          Current encoder electrical angle
    // @param sensor_offset          Sensor mechanical offset
    // @param elec_zero_offset       Electrical zero offset
    // @param num_pole_pairs         Motor pole pairs
    // @param current_time_us        Current timestamp in microseconds
    // @return                       true if transition should proceed
    //-------------------------------------------------------------------------
    bool prepare_transition(float accumulated_rad_elec,
                            float kalman_rad_elec,
                            float sensor_offset,
                            float elec_zero_offset,
                            float num_pole_pairs,
                            uint32_t current_time_us)
    {
        // Check if angles are close enough to safely transition
        float err_rad_elec = kalman_rad_elec - accumulated_rad_elec;
        if (err_rad_elec > MY_PI)
        {
            err_rad_elec -= TWO_PI;
        }
        else if (err_rad_elec < -MY_PI)
        {
            err_rad_elec += TWO_PI;
        }

        // store error
        m_kalman_phase_err_rad_elec = err_rad_elec;

        // Store angles
        m_accumulated_rad_elec = accumulated_rad_elec;
        m_angle_rad_elec = kalman_rad_elec;


        // Start transition timer
        m_transition_start_us = current_time_us;
        m_transition_state    = TransitionState::TRANSITIONING;

        // Log transition event
        extern volatile float g_transition_angle_jump;
        extern volatile float g_debug_kalman_phase_offset;
        g_transition_angle_jump = err_rad_elec;
        g_debug_kalman_phase_offset = m_kalman_phase_err_rad_elec;

        return true;
    }

    //-------------------------------------------------------------------------
    // get_blended_angle
    //-------------------------------------------------------------------------
    // Returns smoothly blended angle during transition period.
    // After blend completes, returns encoder angle with phase correction.
    //
    // @param encoder_angle       Current encoder electrical angle
    // @param kalman_mechanical   Kalman mechanical angle
    // @param sensor_offset       Sensor mechanical offset
    // @param elec_zero_offset    Electrical zero offset
    // @param num_pole_pairs      Motor pole pairs
    // @param current_time_us     Current timestamp in microseconds
    // @param target_velocity     Target mechanical velocity (rad/s) to advance angle
    // @param delta_time_sec      Time since last call (seconds)
    // @return                    Angle for commutation (electrical radians)
    //-------------------------------------------------------------------------
    float get_blended_angle(float kalman_elec,
                           float num_pole_pairs,
                           float target_velocity,
                           float delta_time_sec)
    {

        if (m_transition_state == TransitionState::ACCUMULATED_MODE)
        {
            return m_accumulated_rad_elec;
        }

        // Get current time directly to ensure accuracy
        uint32_t current_time_us = micros();

        // Calculate blend factor [0.0 = accumulated, 1.0 = encoder]
        float blend_factor = 0.0f;

        if (m_transition_state == TransitionState::TRANSITIONING)
        {
            uint32_t elapsed_us        = compute_elapsed_time(m_transition_start_us, current_time_us);
            uint32_t blend_duration_us = static_cast<uint32_t>(BLEND_DURATION_SEC * 1e6f);

            if (elapsed_us >= blend_duration_us)
            {
                blend_factor = 1.0f;
                m_transition_state = TransitionState::ENCODER_MODE;
            } else {
                blend_factor = static_cast<float>(elapsed_us) / static_cast<float>(blend_duration_us);
            }

            // Debug: Export elapsed time for monitoring
            extern volatile uint32_t g_transition_elapsed_us;
            g_transition_elapsed_us = elapsed_us;

        }
        else {
            blend_factor = 1.0f;  // Already in encoder mode
        }

        // Gradually reduce the phase error over the transition period
        float corrected_error = m_kalman_phase_err_rad_elec * (1.0f - blend_factor);
        float target_angle = kalman_elec + corrected_error;
        target_angle = normalize_angle(target_angle);


        // CRITICAL: Advance accumulated_angle_ based on target velocity
        // to keep motor spinning during transition
        if (/*blend_factor < 0.999f && */ delta_time_sec > 0.0f)
        {
            float angle_increment = target_velocity * delta_time_sec * num_pole_pairs;
            m_accumulated_rad_elec += angle_increment;
            m_accumulated_rad_elec = normalize_angle(m_accumulated_rad_elec);
        }

        // if (blend_factor < 0.999f) {
        //     // Calculate our own delta time for reliable angle advancement
        //     static uint32_t last_update_time_us = 0;
        //     if (last_update_time_us == 0) last_update_time_us = current_time_us;

        //     uint32_t dt_us = compute_elapsed_time(last_update_time_us, current_time_us);
        //     float dt_sec = dt_us * 1.0e-6f;
        //     last_update_time_us = current_time_us;

        //     if (dt_sec > 0.0f && dt_sec < 0.001f) {  // Sanity check: 0-1ms
        //         float angle_increment = target_velocity * dt_sec * num_pole_pairs;
        //         accumulated_angle_ += angle_increment;
        //         accumulated_angle_ = normalize_angle(accumulated_angle_);
        //     }
        // }

        // CRITICAL: Blend from accumulated_angle_ to target_angle
        // blend_factor: 0.0 = use accumulated, 1.0 = use encoder
        float blended_angle;

        if (blend_factor < 0.001f)
        {
            // Pure accumulated mode
            blended_angle = m_accumulated_rad_elec;
        }
        else if (blend_factor > 0.999f)
        {
            // // Pure encoder mode - but advance by predicted motion during computation
            // float angle_increment = target_velocity * delta_time_sec * num_pole_pairs;
            // blended_angle = target_angle + angle_increment;
            // blended_angle = normalize_angle(blended_angle);

            // Use accumulated angle which has been tracking encoder via sync
            blended_angle = m_accumulated_rad_elec;
        }
        else
        {
            // Blending - handle angle wraparound properly
            float angle_diff = m_accumulated_rad_elec - target_angle;

            // Unwrap to find shortest path
            if (angle_diff > MY_PI) angle_diff -= TWO_PI;
            else if (angle_diff < -MY_PI) angle_diff += TWO_PI;

            // Linear interpolation
            blended_angle = m_accumulated_rad_elec + blend_factor * angle_diff;
            blended_angle = normalize_angle(blended_angle);
        }

        // Expose blend factor for debugging
        extern volatile float g_encoder_blend_factor;
        extern volatile float g_transition_blend_factor;
        g_encoder_blend_factor = blend_factor;
        g_transition_blend_factor = blend_factor;

        return blended_angle;
    }

    //-------------------------------------------------------------------------
    // bumpless_pi_transfer
    //-------------------------------------------------------------------------
    // Adjusts PI controller integral term to maintain current output during
    // mode transition. Prevents voltage step that causes motor stall.
    //
    // Call this once when transitioning modes.
    //
    // @param Kp                    Proportional gain
    // @param Ki                    Integral gain
    // @param current_error         Current velocity error
    // @param current_output        Desired voltage output to maintain
    // @param integral_max          Anti-windup limit
    // @param[in/out] integral      PI integral term (will be adjusted)
    //-------------------------------------------------------------------------
    void bumpless_pi_transfer(float Kp,
                             float Ki,
                             float current_error,
                             float current_output,
                             float integral_max,
                             float& integral) {

        // Back-calculate integral term needed to maintain current output
        // output = Kp * error + Ki * integral
        // => integral = (output - Kp * error) / Ki

        if (Ki > 1e-6f) {  // Avoid division by zero
            float required_integral = (current_output - Kp * current_error) / Ki;

            // Clamp to anti-windup limits
            required_integral = fmaxf(fminf(required_integral, integral_max), -integral_max);

            integral = required_integral;
        }
    }

    //-------------------------------------------------------------------------
    // Getters
    //-------------------------------------------------------------------------
    TransitionState get_state() const { return m_transition_state; }
    float get_kalman_phase_offset() const { return m_kalman_phase_err_rad_elec; }
    bool is_in_accumulated_mode() const { return m_transition_state == TransitionState::ACCUMULATED_MODE; }
    bool is_transitioning() const { return m_transition_state == TransitionState::TRANSITIONING; }
    bool is_in_encoder_mode() const { return m_transition_state == TransitionState::ENCODER_MODE; }

private:
    //-------------------------------------------------------------------------
    // Helper: normalize_angle
    //-------------------------------------------------------------------------
    float normalize_angle(float angle) const {
        angle = fmodf(angle, TWO_PI);
        if (angle < 0.0f) angle += TWO_PI;
        return angle;
    }

    //-------------------------------------------------------------------------
    // Helper: compute_elapsed_time (overflow-safe)
    //-------------------------------------------------------------------------
    uint32_t compute_elapsed_time(uint32_t start_time, uint32_t current_time) const {
        if (current_time >= start_time) {
            return current_time - start_time;
        } else {
            // Overflow occurred
            return (UINT32_MAX - start_time) + current_time;
        }
    }

    //-------------------------------------------------------------------------
    // Member Variables
    //-------------------------------------------------------------------------
    float           m_accumulated_rad_elec; // Last accumulated angle
    float           m_angle_rad_elec;       // Last encoder angle
    uint32_t        m_transition_start_us;  // When transition began
    TransitionState m_transition_state;     // Current transition state
    float           m_kalman_phase_err_rad_elec; // Phase correction from transition
};

#endif // ANGLE_TRANSITION_MANAGER_HPP
