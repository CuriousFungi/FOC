# High-Speed Velocity Estimation Fix
## Quick Start Guide for 100 rad/sec Operation

## Problem
Motor hesitates and velocity measurements become noisy at speeds above 4 rad/sec. Target operating range is 0-100 rad/sec.

## Root Cause
Kalman filter parameters were not tuned for high-speed operation where quantization noise dominates.

### Physics at 100 rad/sec
- **Encoder**: 14-bit (16384 counts/rev) = 0.000383 rad/count
- **At 100 rad/s**: 261,000 counts/second (3.8 μs per count!)
- **At 10 kHz sampling**: Motor rotates ~0.01 rad (26 counts) between samples
- **Result**: Severe quantization noise requires adaptive filtering

## Solution: Adaptive Kalman Filter

### Changes Made to `AS5048A.cpp`

#### 1. Base Parameters Updated (Line ~210)
```cpp
// OLD (not suitable for high speed):
m_kalman_filter(1e-5f, 1e-2f, 5e-4f)

// NEW (optimized for 0-100 rad/s):
m_kalman_filter(1e-5f, 5e-3f, 2e-3f)
//              Q_pos  Q_vel  R_meas
```

**Changes:**
- `Q_velocity`: 1e-2 → 5e-3 (moderate for responsiveness)
- `R_measurement`: 5e-4 → 2e-3 (higher base for quantization noise)

#### 2. Adaptive Process Noise Velocity (Line ~615)
```cpp
// NEW: Allows faster tracking at high speeds
float adaptive_Q_vel = 5e-3f;
if (estimated_speed_abs > 10.0f) {
    adaptive_Q_vel = 5e-3f + ((estimated_speed_abs - 10.0f) * 1e-4f);
    if (adaptive_Q_vel > 2e-2f) {
        adaptive_Q_vel = 2e-2f;  // Cap
    }
}
m_kalman_filter.set_process_noise_velocity(adaptive_Q_vel);
```

**Behavior:**
- 0-10 rad/s: Q_vel = 5e-3 (base)
- 10-100 rad/s: Q_vel scales up to 1.4e-2
- >100 rad/s: Q_vel = 2e-2 (maximum)

#### 3. Adaptive Measurement Noise (Line ~630)
```cpp
// NEW: Heavy smoothing at high speeds
float adaptive_R = 2e-3f + (estimated_speed_abs * 3e-4f);
if (adaptive_R > 5e-2f) {
    adaptive_R = 5e-2f;  // Cap at 50 mrad²
}
m_kalman_filter.set_measurement_noise(adaptive_R);
```

**Behavior:**
- 0-5 rad/s: R ≈ 2e-3 (trust measurements)
- 20 rad/s: R ≈ 8e-3 (moderate smoothing)
- 50 rad/s: R ≈ 1.7e-2 (heavy smoothing)
- 100 rad/s: R ≈ 3.2e-2 (very heavy smoothing)
- >167 rad/s: R = 5e-2 (maximum cap)

## Files Modified
1. `CM7/Core/Src/sensors/AS5048A.cpp` - Kalman filter parameters and adaptive logic

## Files Created
1. `CM7/Core/Src/sensors/KALMAN_VELOCITY_TUNING.md` - Comprehensive tuning guide
2. `CM7/Core/Src/sensors/velocity_test_diagnostics.cpp` - Diagnostic tools (optional)

## Testing Procedure

### Step 1: Flash New Firmware
Compile and flash the updated `AS5048A.cpp` with new parameters.

### Step 2: Test at Problem Speed (4 rad/sec)
1. Command motor to 4 rad/sec
2. Monitor `g_debug_measured_mech_rad_per_sec` in STM32CubeMonitor
3. **Expected**: Smooth velocity readings, no erratic spikes
4. **Target noise**: < 0.2 rad/s standard deviation

### Step 3: Test Increasing Speeds
Test at these speeds in sequence:
```
0 → 5 → 10 → 20 → 30 → 50 → 75 → 100 rad/s
```

At each speed, verify:
- [ ] `g_debug_measured_mech_rad_per_sec` is smooth
- [ ] No motor hesitation or stuttering
- [ ] `g_kalman_velocity_variance` remains bounded (< 1.0)
- [ ] Position tracking still accurate

### Step 4: Acceleration Tests
Test rapid speed changes:
```
0 → 50 rad/s in 100ms
50 → 100 rad/s in 100ms  
100 → 0 rad/s in 200ms
```

**Expected**: Smooth tracking, no oscillations, overshoot < 20%

## Key Debug Variables to Monitor

| Variable | Description | Expected at 100 rad/s |
|----------|-------------|----------------------|
| `g_debug_measured_mech_rad_per_sec` | Final velocity | Smooth, ~100 rad/s |
| `g_kalman_velocity` | Kalman output | Same as above |
| `g_kalman_velocity_variance` | Uncertainty | < 1.0, stable |
| `g_kalman_dt` | Sample time | 50-200 μs |
| `g_velocity_raw` | Unfiltered | Very noisy |

## Expected Performance by Speed Range

| Speed Range | Noise Level | SNR | Settling Time |
|-------------|-------------|-----|---------------|
| 0-5 rad/s | < 0.05 rad/s | > 40 dB | < 50 ms |
| 5-20 rad/s | < 0.2 rad/s | > 35 dB | < 100 ms |
| 20-50 rad/s | < 1.0 rad/s | > 30 dB | < 200 ms |
| 50-100 rad/s | < 3.0 rad/s | > 25 dB | < 500 ms |

## Fine-Tuning (If Needed)

### If Still Noisy at High Speeds
Increase measurement noise scaling:
```cpp
// Line ~630: Change scaling coefficient
float adaptive_R = 2e-3f + (estimated_speed_abs * 5e-4f);  // Was 3e-4f
```

### If Lags During Acceleration
Increase process noise velocity:
```cpp
// Line ~210: Change base Q_velocity
m_kalman_filter(1e-5f, 1e-2f, 2e-3f)  // Was 5e-3f
```

### If Cannot Reach 100 rad/s
This may NOT be a Kalman issue. Check:
1. **BEMF limit**: V_supply must exceed back-EMF at 100 rad/s
2. **Current limit**: May be saturating
3. **Mechanical limit**: Load may prevent high speeds
4. **Sampling rate**: Verify encoder updates at 10+ kHz

## Alternative Parameter Sets

### Conservative (Maximum Stability)
```cpp
m_kalman_filter(1e-5f, 2e-3f, 5e-3f)
// Use if: Stability critical, can tolerate more lag
```

### Aggressive (Maximum Tracking)
```cpp
m_kalman_filter(1e-5f, 1e-2f, 1e-3f)
// Use if: Need tight tracking, motor can handle some noise
```

### Ultra-High Speed (100+ rad/s primary)
```cpp
m_kalman_filter(1e-4f, 2e-2f, 1e-2f)
// Use if: Operating primarily above 50 rad/s
```

## Troubleshooting

### Problem: Motor still hesitates at 4 rad/s
**Possible causes:**
1. Kalman parameters still too aggressive for your sampling rate
2. Control loop issue (not velocity estimation)
3. Insufficient back-EMF compensation

**Debug:**
- Check `g_kalman_dt` - should be < 200 μs
- Compare `g_velocity_raw` vs `g_kalman_velocity` - filter should smooth
- Check `g_debug_amperage_q_for_bemf` - may need tuning

### Problem: Works up to 50 rad/s, fails higher
**Possible causes:**
1. Hardware limitation (sampling rate, BEMF, current)
2. Need more aggressive R_measurement scaling
3. Motor physically cannot reach that speed

**Debug:**
- Monitor `g_kalman_dt` - should stay consistent
- Check supply voltage during high speed
- Verify mechanical load is not excessive

### Problem: Velocity readings are smooth but motor still hesitates
**Not a Kalman filter issue!** Check:
1. Current controller tuning (P-I gains)
2. Back-EMF compensation (permanent magnet flux)
3. PWM frequency and dead-time settings
4. Phase resistance and inductance values

## Success Criteria

✅ **Fix is working if:**
- Motor runs smoothly from 0 to 100 rad/s
- `g_debug_measured_mech_rad_per_sec` is smooth at all speeds
- No hesitation or stuttering
- Velocity noise < 3% of commanded speed

❌ **Further tuning needed if:**
- Velocity still very noisy at high speeds (> 5% noise)
- Motor cannot reach 100 rad/s
- Large overshoot (> 30%) during speed changes
- Velocity estimate drifts from actual speed

## Additional Resources

- **Detailed tuning guide**: `CM7/Core/Src/sensors/KALMAN_VELOCITY_TUNING.md`
- **Diagnostic tools**: `CM7/Core/Src/sensors/velocity_test_diagnostics.cpp`
- **Kalman filter implementation**: `CM7/Core/Inc/KalmanFilter.hpp`

## Quick Reference

```
ADAPTIVE PARAMETER RANGES
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Parameter    Base      Max       Speed-dependent
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Q_position   1e-5      (fixed)   No
Q_velocity   5e-3      2e-2      Yes, > 10 rad/s
R_measure    2e-3      5e-2      Yes, all speeds
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

**Implementation Date**: 2024-01
**Target Speed Range**: 0 - 100 rad/sec
**Key Improvement**: Adaptive filtering eliminates velocity noise at high speeds