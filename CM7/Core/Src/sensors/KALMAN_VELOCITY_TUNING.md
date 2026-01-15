# Kalman Filter Velocity Estimation Tuning Guide
# For High-Speed Motor Control (0 - 100 rad/sec)

## Problem Description

The motor control system experiences velocity estimation noise at speeds around 4 rad/sec and above, with a target operating range up to **100 rad/sec**. This manifests as:

- Erratic velocity readings (g_debug_measured_mech_rad_per_sec)
- Motor hesitation at higher speeds
- Control loop instability due to noisy velocity feedback
- Severe quantization effects at high speeds

## Physics of High-Speed Encoder Measurements

### Encoder Specifications
- **Resolution**: 14-bit (16384 counts per revolution)
- **Angular resolution**: 2π / 16384 = 0.000383 rad per count
- **Maximum speed target**: 100 rad/sec

### Quantization Effects at Different Speeds

| Speed (rad/s) | Counts/sec | Time/count | Samples @ 10kHz | Δθ per sample |
|---------------|------------|------------|-----------------|---------------|
| 1             | 2,610      | 383 μs     | 3.8 counts      | 0.0015 rad    |
| 4             | 10,440     | 96 μs      | 1 count         | 0.0004 rad    |
| 10            | 26,100     | 38 μs      | 0.4 counts      | 0.0010 rad    |
| 50            | 130,500    | 7.7 μs     | 0.08 counts     | 0.0050 rad    |
| 100           | 261,000    | 3.8 μs     | 0.04 counts     | 0.0100 rad    |

**Critical Observation**: At 100 rad/s with 10 kHz sampling, the motor rotates ~0.01 radians between samples (26 encoder counts). This creates severe quantization noise that requires aggressive filtering.

## Root Cause Analysis

### Previous Parameters (Not Suitable for High Speed)
```cpp
m_kalman_filter(1e-5f, 1e-2f, 5e-4f)
//              Q_pos  Q_vel  R_meas
```

**Problems for 100 rad/sec operation:**
1. **Q_velocity too moderate**: Cannot track rapid velocity changes and accelerations
2. **R_measurement too low**: Trusts noisy measurements too much at high speeds
3. **No adaptation**: Single parameter set cannot work across 0-100 rad/s range

## Solution: Adaptive Kalman Filter for Wide Speed Range

### New Base Parameters
```cpp
m_kalman_filter(1e-5f, 5e-3f, 2e-3f)
//              Q_pos  Q_vel  R_meas
```

**Design rationale:**
1. **Q_position (1e-5)**: Moderate - position tracking still important
2. **Q_velocity (5e-3)**: High - allows tracking of rapid velocity changes
3. **R_measurement (2e-3)**: Moderate base - will adapt with speed

### Adaptive Measurement Noise (Speed-Dependent)

```cpp
float estimated_speed_abs = fabsf(m_kalman_filter.get_velocity());

// Scale measurement noise with speed to handle quantization
float adaptive_R = 2e-3f + (estimated_speed_abs * 3e-4f);
if (adaptive_R > 5e-2f) {
    adaptive_R = 5e-2f;  // Cap at maximum
}
m_kalman_filter.set_measurement_noise(adaptive_R);
```

**Behavior:**
- **0-5 rad/s**: R = 2e-3 (trust measurements, track tightly)
- **5-20 rad/s**: R = 3.5e-3 to 8e-3 (moderate smoothing)
- **20-50 rad/s**: R = 8e-3 to 1.7e-2 (heavy smoothing)
- **50-100 rad/s**: R = 1.7e-2 to 3.2e-2 (very heavy smoothing)
- **>100 rad/s**: R = 5e-2 (maximum smoothing)

### Adaptive Process Noise Velocity (Acceleration Tracking)

```cpp
float adaptive_Q_vel = 5e-3f;
if (estimated_speed_abs > 10.0f) {
    // Increase Q_vel at high speeds to track acceleration
    adaptive_Q_vel = 5e-3f + ((estimated_speed_abs - 10.0f) * 1e-4f);
    if (adaptive_Q_vel > 2e-2f) {
        adaptive_Q_vel = 2e-2f;
    }
}
m_kalman_filter.set_process_noise_velocity(adaptive_Q_vel);
```

**Behavior:**
- **0-10 rad/s**: Q_vel = 5e-3 (base responsiveness)
- **10-100 rad/s**: Q_vel = 5e-3 to 1.4e-2 (allows faster velocity tracking)
- **>100 rad/s**: Q_vel = 2e-2 (maximum responsiveness)

## Tuning Strategy for Your Application

### Step 1: Verify Sampling Rate

Check `g_kalman_dt` in your debug output:
- **Ideal**: 50-200 μs (5-20 kHz)
- **Minimum for 100 rad/s**: 50 μs (20 kHz)
- **If slower**: Consider increasing SPI/encoder read rate

### Step 2: Test at Incremental Speeds

Test and tune at these speed ranges:

#### Low Speed (0.5 - 5 rad/s)
**Expected behavior:**
- Low noise: < 0.05 rad/s standard deviation
- Tight tracking: < 50 ms settling time
- Minimal lag: < 5% steady-state error

**If problems:**
- Noisy → Increase R_meas base (e.g., 3e-3f)
- Sluggish → Decrease R_meas base (e.g., 1e-3f)

#### Medium Speed (5 - 20 rad/s)
**Expected behavior:**
- Moderate noise: < 0.2 rad/s standard deviation
- Good tracking: < 100 ms settling time
- Acceptable lag: < 3% steady-state error

**If problems:**
- Noisy → Increase R_meas scaling (e.g., 4e-4f per rad/s)
- Lags behind → Increase Q_vel base (e.g., 8e-3f)

#### High Speed (20 - 50 rad/s)
**Expected behavior:**
- Controlled noise: < 0.5 rad/s standard deviation
- Stable tracking: < 200 ms settling time
- Some lag acceptable: < 5% steady-state error

**If problems:**
- Very noisy → Increase R_meas cap (e.g., 7e-2f)
- Cannot track acceleration → Increase Q_vel scaling

#### Ultra-High Speed (50 - 100 rad/s)
**Expected behavior:**
- Heavy smoothing: 1-2 rad/s standard deviation acceptable
- Slower tracking: < 500 ms settling time
- Smoothness priority over lag

**If problems:**
- Oscillations → Increase R_meas significantly
- Completely fails to track → Check sampling rate, may need hardware upgrade

### Step 3: Acceleration Testing

Test rapid speed changes:
```
0 → 50 rad/s in 100 ms (500 rad/s² acceleration)
50 → 100 rad/s in 100 ms (500 rad/s² acceleration)
100 → 0 rad/s in 200 ms (-500 rad/s² deceleration)
```

**Monitor:**
- `g_kalman_velocity_variance` - should remain bounded (< 1.0)
- `g_velocity_diag_overshoot_percent` - should be < 20% at high speeds
- `g_velocity_diag_response_time_ms` - will increase with speed

## Advanced Tuning Parameters

### Alternative Parameter Sets

#### Conservative (Stability Priority)
```cpp
m_kalman_filter(1e-5f, 2e-3f, 5e-3f)
// Higher base R = more smoothing
// Lower Q_vel = less responsive but stabler
```
Use when: Stability is critical, can tolerate lag

#### Aggressive (Tracking Priority)
```cpp
m_kalman_filter(1e-5f, 1e-2f, 1e-3f)
// Lower base R = less smoothing
// Higher Q_vel = more responsive
```
Use when: Need tight tracking, motor can handle some noise

#### Maximum Speed Optimized (100+ rad/s)
```cpp
m_kalman_filter(1e-4f, 2e-2f, 1e-2f)
// All parameters higher for extreme speeds
```
Use when: Operating primarily above 50 rad/s

### Sampling Rate Considerations

If you can increase your encoder sampling rate:

| Sample Rate | Max Recommended Speed | Q_vel | Base R_meas |
|-------------|----------------------|-------|-------------|
| 5 kHz       | 20 rad/s             | 1e-3  | 5e-3        |
| 10 kHz      | 50 rad/s             | 5e-3  | 2e-3        |
| 20 kHz      | 100 rad/s            | 1e-2  | 1e-3        |
| 40 kHz      | 150+ rad/s           | 2e-2  | 5e-4        |

**Rule of thumb**: Sample at least 50x faster than your maximum speed (in Hz).
- 100 rad/s = 15.9 Hz → need 800 Hz minimum, but 10+ kHz recommended

## Diagnostic Checklist

### Before Tuning
- [ ] Verify encoder is working correctly (check raw counts)
- [ ] Verify SPI communication has no errors
- [ ] Check `g_kalman_dt` is consistent and reasonable
- [ ] Verify motor can physically achieve target speeds

### During Tuning
- [ ] Monitor `g_kalman_velocity_variance` (should converge and stay bounded)
- [ ] Watch `g_debug_measured_mech_rad_per_sec` vs `g_target_rps_to_cl_controller`
- [ ] Check `g_velocity_diag_noise_level` at each speed range
- [ ] Verify no velocity spikes during direction changes

### After Tuning
- [ ] Test full speed range: 0 → 100 → 0 rad/s
- [ ] Verify no hesitation or oscillation
- [ ] Check steady-state error < 5% at all speeds
- [ ] Confirm stable operation for extended periods (5+ minutes)

## Common Issues and Solutions

### Issue: Works at 4 rad/s, fails at 10+ rad/s
**Cause:** Adaptive scaling not aggressive enough
**Solution:** Increase R_meas scaling coefficient from 3e-4f to 5e-4f or higher

### Issue: Extremely noisy at all speeds
**Cause:** Base parameters too aggressive, sampling rate too low
**Solution:** 
1. Increase base R_meas from 2e-3f to 5e-3f
2. Check encoder sampling rate (aim for 10+ kHz)
3. Verify no SPI communication errors

### Issue: Velocity lags behind during acceleration
**Cause:** Q_velocity too low or R_measurement too high
**Solution:**
1. Increase Q_vel base from 5e-3f to 1e-2f
2. Enable adaptive Q_vel scaling for speeds > 10 rad/s
3. Reduce R_meas slightly if stable at target speed

### Issue: Oscillations at constant high speed
**Cause:** Q_velocity too high, filter tracking noise
**Solution:**
1. Decrease Q_vel from 5e-3f to 2e-3f
2. Increase R_meas cap from 5e-2f to 1e-1f for heavy smoothing
3. Add velocity deadband in control loop

### Issue: Cannot reach 100 rad/s
**Cause:** May not be Kalman filter - check motor physics
**Solution:**
1. Verify BEMF voltage allows 100 rad/s at your supply voltage
2. Check current limit is not saturating
3. Verify mechanical load is not limiting speed
4. If Kalman-related: Increase Q_vel to 5e-2f, decrease R_meas scaling

## Performance Targets by Speed Range

### 0-5 rad/s (Low Speed)
- **Noise**: < 0.05 rad/s (1% of 5 rad/s)
- **SNR**: > 40 dB
- **Settling time**: < 50 ms
- **Overshoot**: < 5%

### 5-20 rad/s (Medium Speed)
- **Noise**: < 0.2 rad/s (1-2% of speed)
- **SNR**: > 35 dB
- **Settling time**: < 100 ms
- **Overshoot**: < 10%

### 20-50 rad/s (High Speed)
- **Noise**: < 1.0 rad/s (2-5% of speed)
- **SNR**: > 30 dB
- **Settling time**: < 200 ms
- **Overshoot**: < 15%

### 50-100 rad/s (Ultra-High Speed)
- **Noise**: < 3.0 rad/s (3-6% of speed)
- **SNR**: > 25 dB
- **Settling time**: < 500 ms
- **Overshoot**: < 20%

## Critical Code Locations

1. **Kalman initialization**: `AS5048A.cpp:210`
   ```cpp
   m_kalman_filter(1e-5f, 5e-3f, 2e-3f)
   ```

2. **Adaptive measurement noise**: `AS5048A.cpp:630`
   ```cpp
   float adaptive_R = 2e-3f + (estimated_speed_abs * 3e-4f);
   ```

3. **Adaptive process noise**: `AS5048A.cpp:615`
   ```cpp
   float adaptive_Q_vel = 5e-3f + ...;
   ```

4. **Velocity output**: `AS5048A.cpp:640`
   ```cpp
   g_as5048_velocity = g_kalman_velocity;
   ```

## Quick Reference Card

```
SPEED RANGE QUICK TUNE TABLE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Speed        Q_vel      R_meas     Notes
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
0-5 rad/s    5e-3       2e-3       Low noise priority
5-20 rad/s   5e-3       3e-3       Balanced
20-50 rad/s  8e-3       1e-2       More smoothing
50-100 rad/s 1.5e-2     2.5e-2     Heavy smoothing
100+ rad/s   2e-2       5e-2       Maximum smoothing
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

## Next Steps

1. **Flash updated firmware** with new Kalman parameters
2. **Start testing at 4 rad/s** where issues were observed
3. **Gradually increase** to 10, 20, 50, 75, 100 rad/s
4. **Monitor debug variables** at each speed
5. **Fine-tune** adaptive coefficients based on results
6. **Document** final parameters for your specific motor/load

## References

- Implementation: `CM7/Core/Src/sensors/AS5048A.cpp`
- Kalman filter class: `CM7/Core/Inc/KalmanFilter.hpp`
- Control loop: `CM7/Core/Src/motors/StepperMotor.cpp`
- Diagnostic tools: `CM7/Core/Src/sensors/velocity_test_diagnostics.cpp`

---
**Last Updated**: 2024-01 - Parameters optimized for 0-100 rad/s operation