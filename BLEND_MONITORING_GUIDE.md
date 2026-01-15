# Blend Ratio Monitoring Guide

## Overview

This guide explains how to monitor the transition between open-loop and closed-loop motor control using the blend ratio signal and related debug variables.

## Key Variable: Blend Ratio

**Variable Name:** `g_hybrid_blend_factor`

**Range:** 0.0 to 1.0
- **0.0** = Pure open-loop control (using accumulated angle based on target velocity)
- **1.0** = Pure closed-loop control (using encoder angle directly)
- **0.0 - 1.0** = Blended operation (weighted average of open-loop and closed-loop angles)

## Blend Transition Logic

The blend factor is calculated based on motor speed (measured via Kalman filter):

```
Speed < 10 rad/s        → blend_factor = 0.0 (pure open-loop)
Speed > 20 rad/s        → blend_factor = 1.0 (pure closed-loop)
Speed between 10-20     → Linear interpolation
```

**Formula:**
```
blend_factor = (measured_speed - 10.0) / (20.0 - 10.0)
```

## Recommended Monitoring Set

To understand the blending behavior, monitor these variables together:

### Primary Blend Variables
1. **`g_hybrid_blend_factor`** - The blend ratio (0.0 to 1.0)
2. **`g_hybrid_measured_speed`** - Current speed estimate used for blend calculation
3. **`g_open_loop_elec_before_blend`** - Open-loop electrical angle before blending
4. **`g_closed_loop_elec_before_blend`** - Closed-loop (encoder) electrical angle before blending
5. **`g_blended_angle_before_rate_limit`** - Result of blending operation
6. **`g_blended_angle_after_rate_limit`** - Final angle after rate limiting
7. **`g_angle_change_limited`** - Amount of angle change applied by rate limiter

### Supporting Variables
8. **`g_cached_encoder_angle`** - Raw encoder electrical angle
9. **`g_as5048_angle`** - Encoder mechanical angle (0 to 2π per rotation)
10. **`g_kalman_velocity`** - Kalman-filtered velocity estimate
11. **`g_target_rps_to_cl_controller`** - Commanded target speed

## What to Look For

### During Startup (Open-Loop Phase)
- `g_hybrid_blend_factor` should be **0.0**
- `g_blended_angle_before_rate_limit` should match `g_open_loop_elec_before_blend`
- Motor should accelerate using accumulated angle

### During Transition (Blending Phase)
- `g_hybrid_blend_factor` should smoothly increase from 0.0 to 1.0
- `g_blended_angle_before_rate_limit` should smoothly transition between open and closed loop angles
- Watch for discontinuities or jumps in the blended angle

### During Closed-Loop Operation
- `g_hybrid_blend_factor` should be **1.0**
- `g_blended_angle_before_rate_limit` should match `g_closed_loop_elec_before_blend`
- `g_closed_loop_elec_before_blend` should match `g_cached_encoder_angle`

### Rate Limiter Behavior
- Compare `g_blended_angle_before_rate_limit` and `g_blended_angle_after_rate_limit`
- If they differ, the rate limiter is active (protecting against voltage spikes)
- `g_angle_change_limited` shows the actual angle change per iteration

## Common Issues

### Issue 1: Blend Factor Stuck at 0.0
**Symptom:** Motor never transitions to closed-loop
**Possible Causes:**
- Speed not reaching 10 rad/s threshold
- Kalman velocity estimate incorrect
- Check `g_kalman_velocity` and `g_hybrid_measured_speed`

### Issue 2: Angle Jump During Transition
**Symptom:** Motor stutters or vibrates when blend_factor changes
**Possible Causes:**
- Open-loop and closed-loop angles are misaligned
- Check angle difference: `g_closed_loop_elec_before_blend - g_open_loop_elec_before_blend`
- Should be < π/4 (about 0.785 rad) for smooth blending
- Wraparound handling may be failing

### Issue 3: Rate Limiter Always Active
**Symptom:** `g_blended_angle_before_rate_limit != g_blended_angle_after_rate_limit`
**Possible Causes:**
- Angle changes too fast (exceeding rate limit)
- Rate limit may be too conservative
- Check `g_angle_change_limited` magnitude

### Issue 4: Oscillating Blend Factor
**Symptom:** `g_hybrid_blend_factor` oscillates instead of smooth transition
**Possible Causes:**
- Speed oscillating around thresholds (10 or 20 rad/s)
- Kalman filter instability
- Consider adding hysteresis to blend thresholds

## Tuning Parameters

The blend thresholds are defined in `StepperMotor.cpp` around line 2745:

```cpp
const float SPEED_THRESHOLD_LOW = 10.0f;   // rad/s - fully open-loop below this
const float SPEED_THRESHOLD_HIGH = 20.0f;  // rad/s - fully closed-loop above this
```

### Adjustment Guidelines

**Wider transition range (e.g., 5-30 rad/s):**
- Smoother transition
- Takes longer to reach full closed-loop
- Better for reducing torque ripple

**Narrower transition range (e.g., 15-20 rad/s):**
- Faster transition
- May cause more noticeable transition effects
- Better for responsive control

**Higher thresholds (e.g., 20-40 rad/s):**
- Stay in open-loop longer
- Better for high-speed applications
- Encoder quality becomes more critical

**Lower thresholds (e.g., 5-10 rad/s):**
- Transition to closed-loop earlier
- Better for precise position control
- Requires good low-speed encoder performance

## STM32CubeMonitor Setup

1. Add these variables to your monitoring session
2. Create two separate views:
   - **View 1:** Blend overview (blend_factor, speeds, angles)
   - **View 2:** Angle details (all angle-related variables zoomed in)
3. Use different colors for open-loop vs closed-loop angles
4. Set blend_factor on a separate Y-axis (0-1 range) for clarity

## Example Interpretation

Looking at your graphs:

**First Graph (22.6s - 24.4s):**
- Orange line (`g_as5048_angle`) climbing steadily → motor rotating in closed-loop
- Purple spikes (`g_cached_encoder_angle`) → electrical angle wrapping every 2π/pole_pairs
- Should see `g_hybrid_blend_factor = 1.0` during this period

**Second Graph (22.47s - 22.64s):**
- Duty cycles showing smooth sinusoidal pattern
- This is the electrical commutation in action
- All three phases properly synchronized

**What's Missing:**
Add `g_hybrid_blend_factor` to see exactly when transition occurs!

## Code Reference

The blending logic is implemented in:
- **File:** `project3/CM7/Core/Src/motors/StepperMotor.cpp`
- **Function:** `StepperMotor::update_speed_closed_loop()`
- **Lines:** Approximately 2740-2890

Key sections:
1. **Blend factor calculation** (~line 2749-2764)
2. **Open-loop angle accumulation** (~line 2766-2782)
3. **Angle wraparound handling** (~line 2795-2809)
4. **Blending operation** (~line 2812-2815)
5. **Rate limiting** (~line 2820-2880)
6. **Debug variable assignment** (~line 2887-2891)

## Next Steps

1. **Add `g_hybrid_blend_factor` to your STM32CubeMonitor session**
2. **Run the motor through a complete startup cycle**
3. **Observe the blend_factor transition from 0.0 → 1.0**
4. **Verify smooth angle blending without discontinuities**
5. **Correlate blend_factor with motor speed (should follow thresholds)**

This will give you complete visibility into the open-loop to closed-loop transition!