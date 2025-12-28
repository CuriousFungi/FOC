//=============================================================================
//                              KalmanFilter.hpp
//
// 2-State Kalman Filter for Encoder Position and Velocity Estimation
//
// State vector: [position, velocity]
// Measurement: position only
//
// This filter optimally estimates velocity from noisy position measurements
// while minimizing lag and rejecting quantization noise.
//=============================================================================

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <cstdint>
#include <cmath>

//-----------------------------------------------------------------------------
//                            KalmanFilter2D
//
// A simple 2-state Kalman filter for position and velocity estimation
// Optimized for encoder applications with quantization noise
//-----------------------------------------------------------------------------
class KalmanFilter2D
{
public:
    //-------------------------------------------------------------------------
    //                           Constructor
    //
    // @param process_noise_pos - Process noise for position (rad²)
    // @param process_noise_vel - Process noise for velocity (rad²/s²)
    // @param measurement_noise - Measurement noise for position (rad²)
    //-------------------------------------------------------------------------
    KalmanFilter2D(float process_noise_pos = 1e-6f,
                   float process_noise_vel = 1e-4f,
                   float measurement_noise = 1e-3f)
        : m_q_position(process_noise_pos)
        , m_q_velocity(process_noise_vel)
        , m_r_measurement(measurement_noise)
        , m_position(0.0f)
        , m_velocity(0.0f)
        , m_p11(1.0f)  // Position variance
        , m_p12(0.0f)  // Position-velocity covariance
        , m_p22(1.0f)  // Velocity variance
        , m_initialized(false)
    {
    }

    //-------------------------------------------------------------------------
    //                           initialize
    //
    // Initialize the filter with an initial position measurement
    //
    // @param initial_position - Initial position in radians
    //-------------------------------------------------------------------------
    void initialize(float initial_position)
    {
        m_position = initial_position;
        m_velocity = 0.0f;
        
        // Initial uncertainty
        m_p11 = 1.0f;   // Position variance
        m_p12 = 0.0f;   // Position-velocity covariance
        m_p22 = 10.0f;  // Velocity variance (high initial uncertainty)
        
        m_initialized = true;
    }

    //-------------------------------------------------------------------------
    //                           predict
    //
    // Prediction step: propagate state forward in time
    //
    // State model:
    //   position(k+1) = position(k) + velocity(k) * dt
    //   velocity(k+1) = velocity(k)  [constant velocity model]
    //
    // @param dt - Time step in seconds
    //-------------------------------------------------------------------------
    void predict(float dt)
    {
        if (!m_initialized) return;
        
        // State prediction
        m_position = m_position + m_velocity * dt;
        // m_velocity remains the same (constant velocity model)
        
        // Covariance prediction: P = F*P*F' + Q
        // State transition matrix F:
        // [ 1  dt ]
        // [ 0   1 ]
        
        float p11_pred = m_p11 + 2.0f * dt * m_p12 + dt * dt * m_p22 + m_q_position;
        float p12_pred = m_p12 + dt * m_p22;
        float p22_pred = m_p22 + m_q_velocity;
        
        m_p11 = p11_pred;
        m_p12 = p12_pred;
        m_p22 = p22_pred;
    }

    //-------------------------------------------------------------------------
    //                           update
    //
    // Update step: correct state estimate with new position measurement
    //
    // @param measured_position - Position measurement in radians
    //-------------------------------------------------------------------------
    void update(float measured_position)
    {
        if (!m_initialized)
        {
            initialize(measured_position);
            return;
        }
        
        // Innovation (measurement residual)
        float innovation = measured_position - m_position;
        
        // Handle angle wrap-around: keep innovation in [-π, π]
        while (innovation > M_PI) innovation -= 2.0f * M_PI;
        while (innovation < -M_PI) innovation += 2.0f * M_PI;
        
        // Innovation covariance: S = H*P*H' + R
        // Measurement matrix H = [1  0] (we only measure position)
        float innovation_covariance = m_p11 + m_r_measurement;
        
        // Kalman gain: K = P*H' / S
        float k1 = m_p11 / innovation_covariance;  // Gain for position
        float k2 = m_p12 / innovation_covariance;  // Gain for velocity
        
        // State update: x = x + K * innovation
        m_position = m_position + k1 * innovation;
        m_velocity = m_velocity + k2 * innovation;
        
        // Covariance update: P = (I - K*H) * P
        float p11_new = m_p11 - k1 * m_p11;
        float p12_new = m_p12 - k1 * m_p12;
        float p22_new = m_p22 - k2 * m_p12;
        
        m_p11 = p11_new;
        m_p12 = p12_new;
        m_p22 = p22_new;
        
        // Keep position in [0, 2π] range
        while (m_position >= 2.0f * M_PI) m_position -= 2.0f * M_PI;
        while (m_position < 0.0f) m_position += 2.0f * M_PI;
    }

    //-------------------------------------------------------------------------
    //                          Getters
    //-------------------------------------------------------------------------
    float get_position() const { return m_position; }
    float get_velocity() const { return m_velocity; }
    float get_position_variance() const { return m_p11; }
    float get_velocity_variance() const { return m_p22; }
    bool is_initialized() const { return m_initialized; }
    
    //-------------------------------------------------------------------------
    //                          Setters for Tuning
    //-------------------------------------------------------------------------
    void set_process_noise_position(float q) { m_q_position = q; }
    void set_process_noise_velocity(float q) { m_q_velocity = q; }
    void set_measurement_noise(float r) { m_r_measurement = r; }
    
    //-------------------------------------------------------------------------
    //                          reset
    //-------------------------------------------------------------------------
    void reset()
    {
        m_position = 0.0f;
        m_velocity = 0.0f;
        m_p11 = 1.0f;
        m_p12 = 0.0f;
        m_p22 = 1.0f;
        m_initialized = false;
    }

private:
    // Process noise covariances (Q matrix)
    float m_q_position;      // Position process noise (rad²)
    float m_q_velocity;      // Velocity process noise (rad²/s²)
    
    // Measurement noise covariance (R matrix)
    float m_r_measurement;   // Position measurement noise (rad²)
    
    // State estimates
    float m_position;        // Estimated position (rad)
    float m_velocity;        // Estimated velocity (rad/s)
    
    // State covariance matrix P (2x2, symmetric)
    float m_p11;             // Position variance
    float m_p12;             // Position-velocity covariance
    float m_p22;             // Velocity variance
    
    // Initialization flag
    bool m_initialized;
};

#endif // KALMAN_FILTER_HPP