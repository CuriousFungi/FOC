# Open-Loop to Closed-Loop Transition Issue - Diagnostic Guide

## Problem Description

Motor exhibits "flakiness" (hesitation, noise, instability) around **4-5 rad/s**, exactly at the transition point between open-loop and closed-loop control modes.

**Symptoms:**
- Smooth operation below 4 rad/s
- Erratic behavior at 4-5 rad/s (transition zone)
- Smooth operation above 5 rad/s
- Velocity measurement appears noisy during transition
- Motor may stutter or hesitate

## Root Cause

The issue is **NOT the Kalman filter** - it's the **angle blending between control modes**.

### Control Mode Architecture

Your system uses two control strategies:

```
0 - 4 rad/s:     OPEN-LOOP (accumulated angle based on target velocity)
4 - 5 rad/s:     TRANSITION (blended angle - THIS IS THE PROBLEM ZONE)
5+ rad/s:        CLOSED-LOOP (encoder feedback angle)
```

**Why Open-Loop at Low Speed?**
At startup, motor is stationary. If you use the encoder angle directly for FOC, the magnetic field aligns with the rotor (0° phase) → **no torque**. Open-loop generates a rotating field to get the motor moving.

**Why Transition is Problematic?**
During the blend (4-5 rad/s), the code linearly interpolates between two angles:
- Open-loop angle (accumulated from target velocity)
- Closed-loop angle (from encoder)

If these angles don't match perfectly, the interpolation creates:
- Discontinuous jumps in commutation angle
- Rapid phase corrections
- Torque ripple and vibration
- Apparent "noise" in velocity measurements

## Code Location

**File:** `CM7/Core/Src/motors/StepperMotor.cpp`
**Function:** `update_speed_closed_loop()`
**Lines:** ~2788-2860

**Key Parameters:**
```cpp
const float SPEED_THRESHOLD_LOW = 4.0f;   // Fully open-loop below
const float SPEED_THRESHOLD_HIGH = 5.0f;  // Fully closed-loop above
```

## Solution 1: Lower Transition Speed (RECOMMENDED)

With the new low-lag Kalman filter, closed-loop can engage much earlier!

**Change:**
```cpp
const float SPEED_THRESHOLD_LOW = 1.0f;   // Was 4.0f
const float SPEED_THRESHOLD_HIGH = 2.0f;  // Was 5.0f
```

**Rationale:**
- Old Kalman had 10-20ms lag → needed open-loop until 4 rad/s
- New Kalman has 0.5-1ms lag → can use closed-loop at 2 rad/s
- Motor spends less time in problematic transition zone
- Transition happens at lower speed where torque ripple is less noticeable

**Expected Result:**
- Transition completes before reaching 4 rad/s
- Motor runs smoothly through entire speed range
- Velocity measurements stay clean

## Solution 2: Smoother Blend Function

Instead of linear blending, use a smoother transition:

```cpp
// Replace linear blend with cubic easing
float t = (measured_speed - SPEED_THRESHOLD_LOW) / (SPEED_THRESHOLD_HIGH - SPEED_THRESHOLD_LOW);
t = fmaxf(0.0f, fminf(1.0f, t));  // Clamp to [0, 1]

// Smooth cubic easing (ease-in-out)
float blend_factor = t * t * (3.0f - 2.0f * t);
```

This creates a smoother transition with less abrupt angle changes.

## Solution 3: Angle Synchronization

Ensure open-loop and closed-loop angles are synchronized before blending:

```cpp
// Before entering transition zone, sync open-loop to encoder
if (measured_speed > (SPEED_THRESHOLD_LOW - 0.5f) && !in_transition)
{
    // Entering transition - synchronize angles
    open_loop_angle = m_sensor.get_mechanical_phase_angle_radians();
    in_transition = true;
}

if (measured_speed > SPEED_THRESHOLD_HIGH)
{
    in_transition = false;  // Exited transition
}
```

## Solution 4: Skip Open-Loop Entirely (Advanced)

If your motor can start reliably with encoder feedback:

```cpp
// Disable open-loop - pure closed-loop from 0 rad/s
float blend_factor = 1.0f;  // Always use encoder
float electrical_angle_radians = g_cached_encoder_angle;
```

**When this works:**
- Motor has low cogging torque
- Load is light at startup
- Encoder provides accurate position at 0 speed
- FOC can generate enough torque from standstill

**When this fails:**
- Motor won't start (field aligned with rotor)
- Need initial "kick" with open-loop

## Diagnostic Tests

### Test 1: Verify Transition is the Issue

**Monitor these variables:**
```
g_open_loop_elec_before_blend    - Open-loop angle
g_closed_loop_elec_before_blend  - Closed-loop angle  
g_blended_angle_after_rate_limit - Final blended angle
g_debug_measured_mech_rad_per_sec - Velocity
```

**Expected at 4-5 rad/s:**
- If open/closed angles differ significantly (>0.5 rad), blending causes problems
- Blended angle may show discontinuities or rapid changes
- Velocity appears noisy due to torque variations from angle errors

### Test 2: Disable Transition Temporarily

Set both thresholds to same value:
```cpp
const float SPEED_THRESHOLD_LOW = 0.1f;
const float SPEED_THRESHOLD_HIGH = 0.1f;  // Immediate transition
```

**Result:** Motor jumps directly to closed-loop at 0.1 rad/s. If this works smoothly, transition blending was the problem.

### Test 3: Extended Open-Loop

Keep motor in open-loop longer:
```cpp
const float SPEED_THRESHOLD_LOW = 10.0f;
const float SPEED_THRESHOLD_HIGH = 11.0f;
```

**Result:** If motor runs smoothly at 4-5 rad/s now, transition was definitely the issue (but open-loop isn't accurate at high speeds).

## Monitoring Best Practices

**For Transition Debugging:**
```
ESSENTIAL:
- g_debug_measured_mech_rad_per_sec (velocity)
- g_open_loop_elec_before_blend (open-loop angle)
- g_closed_loop_elec_before_blend (closed-loop angle)
- g_blend_factor (shows transition progress 0.0-1.0)

HELPFUL:
- g_amperage_q (Q-axis current - shows torque variations)
- g_cached_encoder_angle (raw encoder for comparison)
- g_angle_to_setPhaseVoltage (final angle used for FOC)
```

**What to Look For:**
- Large angle difference between open/closed during transition
- Sudden jumps in blended angle
- Q-axis current spikes during transition
- Blend factor changing rapidly due to noisy velocity estimate

## Why Kalman Velocity Affects Transition

The blend factor depends on **measured_speed**, which comes from the Kalman filter:

```cpp
float measured_speed = fabs(measured_mechanical_rad_per_sec);  // From Kalman

if (measured_speed < SPEED_THRESHOLD_LOW)
    blend_factor = 0.0f;  // Open-loop
else if (measured_speed > SPEED_THRESHOLD_HIGH)
    blend_factor = 1.0f;  // Closed-loop
else
    blend_factor = (measured_speed - LOW) / (HIGH - LOW);  // Transition
```

**If Kalman velocity is noisy:**
- Blend factor oscillates
- System switches back and forth between modes
- Creates instability

**With low-lag Kalman:**
- Velocity is stable and responsive
- Blend factor changes smoothly
- Transition is clean

## Recommended Configuration

**For Best Results (with new Kalman filter):**

```cpp
// Early transition to closed-loop
const float SPEED_THRESHOLD_LOW = 1.0f;   // Engage closed-loop early
const float SPEED_THRESHOLD_HIGH = 2.0f;  // Complete transition quickly

// Smooth ramp to avoid rapid speed changes through transition
// In main_cpp.cpp:
if (elapsed_us >= 100000) {  // 100ms updates
    rps += 0.2f;  // 2 rad/s² acceleration
}
```

**Why This Works:**
1. Transition happens at 1-2 rad/s (below problem zone)
2. Smooth acceleration (2 rad/s²) prevents rapid oscillation
3. Low-lag Kalman provides stable velocity measurement
4. Closed-loop engaged early leverages encoder accuracy

## Comparison: Before vs After

### Original Configuration
```
Open-loop:   0 - 4 rad/s
Transition:  4 - 5 rad/s  ← PROBLEM ZONE
Closed-loop: 5+ rad/s

Symptoms:
- Flakiness at 4-5 rad/s
- Noisy velocity measurements
- Motor hesitation
```

### Optimized Configuration
```
Open-loop:   0 - 1 rad/s
Transition:  1 - 2 rad/s  ← Smooth transition
Closed-loop: 2+ rad/s

Result:
- Transition completes before 4 rad/s
- Smooth operation at all speeds
- Clean velocity measurements
```

## Summary

**The Problem:**
- Motor "flakiness" at 4-5 rad/s is due to open-loop ↔ closed-loop transition blending
- NOT caused by Kalman filter noise (that's actually quite good now)
- Blending between mismatched angles creates torque ripple

**The Solution:**
- Lower transition thresholds to 1-2 rad/s (Solution 1 - RECOMMENDED)
- Kalman filter is now fast enough to support earlier closed-loop engagement
- Smooth ramp rate (2 rad/s²) helps avoid oscillation through transition

**Expected Outcome:**
- Motor runs smoothly from 0 to 100+ rad/s
- No more "flakiness" at any speed
- Velocity measurements stay clean throughout range

---

**Implementation Status:** Thresholds updated to 1-2 rad/s
**Test:** Ramp motor from 0 to 10 rad/s and verify smooth transition at 1-2 rad/s
**Monitor:** Blend factor should smoothly progress from 0.0 → 1.0 between 1-2 rad/s