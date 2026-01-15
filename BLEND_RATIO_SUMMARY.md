# Blend Ratio Implementation - Master Summary

## Overview

Your FOC motor control system already has a **blend ratio signal** implemented! It's called `g_hybrid_blend_factor` and it smoothly transitions the motor control from open-loop startup to closed-loop operation based on motor speed.

## The Variable You Need

```c
volatile float g_hybrid_blend_factor;  // Range: 0.0 to 1.0
```

**Location:** Declared in `project3/CM7/Core/Src/motors/StepperMotor.cpp` at line 177
**Updated in:** `StepperMotor::update_speed_closed_loop()` at line 2887

### What It Means

| Value | Control Mode | Description |
|-------|--------------|-------------|
| 0.0   | Pure Open-Loop | Motor uses accumulated angle based on target velocity |
| 0.0-1.0 | Blended | Weighted average of open-loop and closed-loop angles |
| 1.0   | Pure Closed-Loop | Motor uses encoder angle directly |

## How It Works

### Speed-Based Transition

The blend factor is calculated based on motor speed (from Kalman filter):

```
Speed < 10 rad/s:        blend_factor = 0.0  (pure open-loop)
Speed = 10-20 rad/s:     blend_factor = linear interpolation
Speed > 20 rad/s:        blend_factor = 1.0  (pure closed-loop)
```

**Formula:**
```c
if (speed < 10.0f)
    blend_factor = 0.0f;
else if (speed > 20.0f)
    blend_factor = 1.0f;
else
    blend_factor = (speed - 10.0f) / (20.0f - 10.0f);
```

### Angle Blending Process

```
1. Calculate blend_factor from speed
2. Get open-loop angle (accumulated from target velocity)
3. Get closed-loop angle (from encoder)
4. Handle 2π wraparound (critical for smooth blending!)
5. Blend: final = (1-blend) × open + blend × closed
6. Apply rate limiting (prevent voltage spikes)
7. Use final angle for commutation
```

## Complete Variable Set

### Primary Blend Variables

| Variable Name | Type | Range | Purpose |
|---------------|------|-------|---------|
| `g_hybrid_blend_factor` | float | 0.0-1.0 | **Main blend ratio** |
| `g_hybrid_measured_speed` | float | rad/s | Speed used for blend calculation |
| `g_open_loop_elec_before_blend` | float | 0-2π rad | Open-loop electrical angle input |
| `g_closed_loop_elec_before_blend` | float | 0-2π rad | Closed-loop (encoder) angle input |
| `g_blended_angle_before_rate_limit` | float | 0-2π rad | Result of blending operation |
| `g_blended_angle_after_rate_limit` | float | 0-2π rad | Final angle after rate limiting |
| `g_angle_change_limited` | float | rad | Delta angle applied by rate limiter |

### Supporting Variables

| Variable Name | Type | Range | Purpose |
|---------------|------|-------|---------|
| `g_cached_encoder_angle` | float | 0-2π rad | Raw encoder electrical angle |
| `g_as5048_angle` | float | 0-2π rad | Encoder mechanical angle |
| `g_kalman_velocity` | float | rad/s | Kalman-filtered velocity estimate |
| `g_target_rps_to_cl_controller` | float | rad/s | Target speed command |

## STM32CubeMonitor Setup

### Quick Start: Add These Variables

**Minimum set to visualize blending:**
1. `g_hybrid_blend_factor` (ESSENTIAL - the blend ratio itself!)
2. `g_hybrid_measured_speed`
3. `g_kalman_velocity`

**Recommended set for detailed analysis:**
4. `g_open_loop_elec_before_blend`
5. `g_closed_loop_elec_before_blend`
6. `g_blended_angle_before_rate_limit`
7. `g_blended_angle_after_rate_limit`
8. `g_target_rps_to_cl_controller`

### Chart Configuration

**Chart 1: Blend Ratio and Speed**
- Y-axis 1 (0-1): `g_hybrid_blend_factor` (thick line, bright color)
- Y-axis 2 (0-50 rad/s): 
  - `g_hybrid_measured_speed`
  - `g_kalman_velocity`
  - `g_target_rps_to_cl_controller`

**Chart 2: Angle Blending**
- Y-axis (0-2π rad):
  - `g_open_loop_elec_before_blend` (orange)
  - `g_closed_loop_elec_before_blend` (blue)
  - `g_blended_angle_before_rate_limit` (green)
  - `g_blended_angle_after_rate_limit` (red)

## Expected Behavior

### Startup Sequence

**Phase 1: Alignment (0-1s)**
```
blend_factor = 0.0
speed = 0 rad/s
Status: Motor held at fixed angle
```

**Phase 2: Open-Loop Acceleration (1-3s)**
```
blend_factor = 0.0
speed = 0 → 10 rad/s
Status: Pure open-loop, angle accumulation
```

**Phase 3: Transition (3-5s)**
```
blend_factor = 0.0 → 1.0 (smooth ramp)
speed = 10 → 20 rad/s
Status: Blending from open-loop to closed-loop
⚠️ CRITICAL: Watch for smooth transition!
```

**Phase 4: Closed-Loop (5s+)**
```
blend_factor = 1.0
speed > 20 rad/s
Status: Pure encoder-based control
```

## Troubleshooting Guide

### Problem: Blend Factor Stuck at 0.0

**Symptoms:**
- Motor never transitions to closed-loop
- `g_hybrid_blend_factor` remains 0.0

**Diagnosis:**
- Check `g_hybrid_measured_speed` - is it reaching 10 rad/s?
- Check `g_kalman_velocity` - is velocity estimate updating?
- Check `g_target_rps_to_cl_controller` - is target speed high enough?

**Solutions:**
- Increase target speed
- Check Kalman filter tuning
- Lower `SPEED_THRESHOLD_LOW` (currently 10 rad/s)

### Problem: Angle Jump During Transition

**Symptoms:**
- Motor stutters or vibrates when blend_factor changes
- Discontinuity in `g_blended_angle_before_rate_limit`

**Diagnosis:**
- Calculate: `angle_diff = g_closed_loop_elec_before_blend - g_open_loop_elec_before_blend`
- If |angle_diff| > π/4 (0.785 rad), angles are misaligned

**Solutions:**
- Check encoder direction (may be inverted)
- Verify calibration offset is correct
- Ensure open-loop angle initialized to match encoder
- Check for 2π wraparound handling bug

### Problem: Blend Factor Oscillates

**Symptoms:**
- `g_hybrid_blend_factor` rapidly switches between values
- Not a smooth ramp

**Diagnosis:**
- Check if `g_hybrid_measured_speed` is oscillating around 10 or 20 rad/s
- Indicates speed estimate instability

**Solutions:**
- Add hysteresis to blend thresholds
- Increase Kalman filter damping
- Widen transition range (e.g., 5-30 rad/s)

### Problem: Rate Limiter Always Active

**Symptoms:**
- `g_blended_angle_before_rate_limit ≠ g_blended_angle_after_rate_limit`
- Continuously during steady state

**Diagnosis:**
- Check `g_angle_change_limited` magnitude
- If consistently at max (0.5 rad), rate limit is too restrictive

**Solutions:**
- Increase rate limit threshold
- Check if angle changes are actually too fast
- Verify delta_time calculation is correct

## Tuning Parameters

### Location in Code

**File:** `project3/CM7/Core/Src/motors/StepperMotor.cpp`
**Function:** `StepperMotor::update_speed_closed_loop()`
**Lines:** ~2744-2746

```cpp
const float SPEED_THRESHOLD_LOW = 10.0f;   // rad/s - fully open-loop below this
const float SPEED_THRESHOLD_HIGH = 20.0f;  // rad/s - fully closed-loop above this
```

### Tuning Guidelines

**For smoother transition (wider range):**
```cpp
const float SPEED_THRESHOLD_LOW = 5.0f;
const float SPEED_THRESHOLD_HIGH = 30.0f;
```
- Pros: Very smooth, less noticeable transition
- Cons: Takes longer to reach full closed-loop

**For faster transition (narrower range):**
```cpp
const float SPEED_THRESHOLD_LOW = 15.0f;
const float SPEED_THRESHOLD_HIGH = 20.0f;
```
- Pros: Quick transition to closed-loop
- Cons: May be more noticeable, requires good alignment

**For high-speed applications:**
```cpp
const float SPEED_THRESHOLD_LOW = 20.0f;
const float SPEED_THRESHOLD_HIGH = 40.0f;
```
- Pros: Stay in open-loop longer, better for high speeds
- Cons: Encoder quality becomes critical

**For precision/low-speed applications:**
```cpp
const float SPEED_THRESHOLD_LOW = 5.0f;
const float SPEED_THRESHOLD_HIGH = 10.0f;
```
- Pros: Transition to closed-loop earlier, better position control
- Cons: Requires excellent low-speed encoder performance

## Code Implementation Details

### Key Sections

**1. Blend Factor Calculation (line ~2744):**
```cpp
const float measured_speed = fabs(g_kalman_velocity);
float blend_factor = 0.0f;
if (measured_speed < SPEED_THRESHOLD_LOW)
    blend_factor = 0.0f;
else if (measured_speed > SPEED_THRESHOLD_HIGH)
    blend_factor = 1.0f;
else
    blend_factor = (measured_speed - SPEED_THRESHOLD_LOW) / 
                   (SPEED_THRESHOLD_HIGH - SPEED_THRESHOLD_LOW);
```

**2. Open-Loop Angle Accumulation (line ~2766):**
```cpp
static float open_loop_angle = 0.0f;
float angle_increment = target_mechanical_rad_per_sec * delta_seconds;
open_loop_angle += angle_increment;
open_loop_angle = normalize_radians(open_loop_angle);
float open_loop_electrical = mechanical_to_electrical_radians(open_loop_angle);
open_loop_electrical = normalize_radians(open_loop_electrical);
```

**3. Wraparound Handling (line ~2795):**
```cpp
float angle_diff = closed_loop_electrical - open_loop_electrical;
if (angle_diff > MY_PI)
    closed_loop_electrical -= TWO_PI;
else if (angle_diff < -MY_PI)
    closed_loop_electrical += TWO_PI;
```

**4. Blending (line ~2812):**
```cpp
float electrical_angle_radians = (1.0f - blend_factor) * open_loop_electrical 
                               + blend_factor * closed_loop_electrical;
```

**5. Rate Limiting (line ~2820):**
```cpp
static float prev_electrical_angle = 0.0f;
float angle_change = electrical_angle_radians - prev_electrical_angle;
const float MAX_ANGLE_CHANGE = 0.5f;  // rad per iteration
if (fabs(angle_change) > MAX_ANGLE_CHANGE)
    angle_change = copysign(MAX_ANGLE_CHANGE, angle_change);
electrical_angle_radians = prev_electrical_angle + angle_change;
prev_electrical_angle = electrical_angle_radians;
```

**6. Debug Variable Assignment (line ~2887):**
```cpp
g_hybrid_blend_factor = blend_factor;
g_hybrid_open_loop_angle = open_loop_electrical;
g_hybrid_closed_loop_angle = closed_loop_electrical;
g_hybrid_measured_speed = measured_speed;
```

## Testing Procedure

### Step 1: Add Variables to Monitor

Open STM32CubeMonitor and add:
- `g_hybrid_blend_factor` (ESSENTIAL!)
- `g_hybrid_measured_speed`
- `g_kalman_velocity`
- `g_target_rps_to_cl_controller`

### Step 2: Run Complete Startup

1. Power on motor
2. Let alignment complete
3. Ramp speed from 0 to >20 rad/s
4. Observe blend_factor: 0.0 → 0.0~1.0 → 1.0
5. Record at least 10 seconds of data

### Step 3: Verify Smooth Transition

Check for:
- [ ] Blend factor smoothly increases from 0.0 to 1.0
- [ ] No oscillations or rapid switching
- [ ] Transition occurs around expected speeds (10-20 rad/s)
- [ ] Motor runs smoothly throughout transition (no stutter)
- [ ] No angle discontinuities in encoder reading

### Step 4: Analyze Results

**Good transition looks like:**
```
t=0-2s:   blend = 0.0 (flat line)
t=2-4s:   blend = 0.0 → 1.0 (smooth ramp)
t=4s+:    blend = 1.0 (flat line)
```

**Problem transition looks like:**
```
t=2-4s:   blend = 0.0, 0.3, 0.1, 0.6, 0.2... (oscillating)
OR
t=2-4s:   blend = 0.0, 1.0, 0.0, 1.0... (toggling)
```

## Mathematical Background

### Blend Function

The blend operation is a **linear interpolation (lerp)**:

```
f(t) = (1-t) × a + t × b

where:
  t = blend_factor (0 to 1)
  a = open_loop_angle
  b = closed_loop_angle
  f(t) = blended_angle
```

**Properties:**
- When t=0: f(0) = a (pure open-loop)
- When t=1: f(1) = b (pure closed-loop)
- When t=0.5: f(0.5) = average of a and b
- Linear interpolation guarantees smooth transition

### Wraparound Problem

When angles cross the 2π boundary:
```
Example:
  open_loop = 0.1 rad
  closed_loop = 6.2 rad (near 2π)
  
Without correction:
  blended = 0.5 × 0.1 + 0.5 × 6.2 = 3.15 rad
  ❌ WRONG! Takes long path around circle
  
With correction:
  closed_loop -= 2π = -0.08 rad (unwrap)
  blended = 0.5 × 0.1 + 0.5 × (-0.08) = 0.01 rad
  ✅ CORRECT! Takes short path
```

The code handles this using:
```cpp
if (angle_diff > π) closed_loop -= 2π;
if (angle_diff < -π) closed_loop += 2π;
```

## Benefits of This Approach

### Why Blend Instead of Hard Switch?

**Hard Switch (BAD):**
```
if (speed < threshold)
    use_open_loop();
else
    use_closed_loop();
```
Problems:
- Sudden angle jump at transition
- Torque disturbance
- Potential motor stall
- Audible noise

**Soft Blend (GOOD):**
```
blend_factor = smooth_function(speed);
angle = (1-blend) × open + blend × closed;
```
Benefits:
- Smooth angle transition
- Continuous torque
- Inaudible transition
- Robust to speed fluctuations

### Comparison with Other Methods

| Method | Startup | Precision | Smoothness | Complexity |
|--------|---------|-----------|------------|------------|
| Pure Open-Loop | Good | Poor | Good | Low |
| Pure Closed-Loop | Poor | Excellent | Poor at low speed | Low |
| Hard Switch | OK | OK | Poor | Low |
| **Soft Blend** | **Excellent** | **Excellent** | **Excellent** | **Medium** |

## Related Documentation

- **BLEND_MONITORING_GUIDE.md** - Detailed monitoring and troubleshooting
- **BLEND_VARS_QUICK_REF.txt** - Quick variable reference card
- **BLEND_FLOW_DIAGRAM.txt** - ASCII flow diagrams
- **ANALYZING_YOUR_GRAPHS.md** - How to interpret your specific graphs

## Summary

✅ **Your system already has blend ratio implemented!**
✅ **Variable is: `g_hybrid_blend_factor`**
✅ **It's working in your code right now**
❌ **You just need to add it to STM32CubeMonitor to visualize it!**

**Action Item:** Add `g_hybrid_blend_factor` to your monitoring session and run the motor through a complete startup to see the beautiful 0.0 → 1.0 transition! 🎯