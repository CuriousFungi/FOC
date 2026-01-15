# Motor Startup Troubleshooting Guide
## Open-Loop to Closed-Loop Transition Issues

## Problem: Motor Won't Get Past Transition Point

**Symptoms:**
- Motor starts smoothly in open-loop
- Hesitates, stalls, or oscillates at transition speed
- Never achieves stable closed-loop operation
- Velocity measurements show instability at transition

## Root Cause Analysis

The transition from open-loop to closed-loop control is **the most critical and fragile** part of motor startup. When it fails, it's usually due to:

### 1. Phase Angle Mismatch
**Problem:** Open-loop accumulated angle ≠ Encoder angle  
**Why:** Open-loop drifts over time, encoder is absolute  
**Result:** Blending creates sudden phase jump → loss of synchronization

### 2. Insufficient Motor Momentum
**Problem:** Transition happens too early (low speed)  
**Why:** Motor hasn't built up enough inertia  
**Result:** Any disturbance (phase error, load) causes stall

### 3. Velocity Estimate Instability
**Problem:** Noisy or lagging velocity measurement  
**Why:** Affects blend factor calculation  
**Result:** Rapid switching between modes, oscillation

### 4. Load Torque Variation
**Problem:** External load changes during transition  
**Why:** Motor briefly loses optimal torque angle  
**Result:** Can't overcome load, stalls

## Current System Configuration

**File:** `StepperMotor.cpp` line ~2788

```cpp
const float SPEED_THRESHOLD_LOW = 6.0f;   // Fully open-loop below
const float SPEED_THRESHOLD_HIGH = 8.0f;  // Fully closed-loop above
```

**Strategy:**
- 0-6 rad/s: Pure open-loop (motor builds momentum)
- 6-8 rad/s: Blended transition zone
- 8+ rad/s: Pure closed-loop (encoder feedback)

## Why 6-8 rad/s Transition?

**Too Early (1-2 rad/s):**
- ❌ Low motor inertia, stalls easily
- ❌ Large impact from any phase error
- ❌ Weak torque production
- ❌ Motor can't push through transition

**Too Late (15-20 rad/s):**
- ❌ Open-loop accumulates significant drift
- ❌ Large phase mismatch with encoder
- ❌ Violent transition when blending starts

**Optimal (6-8 rad/s):**
- ✅ Motor has sufficient momentum
- ✅ Open-loop hasn't drifted too much
- ✅ Torque production strong enough to handle disturbances
- ✅ Velocity estimate is stable

## Diagnostic Steps

### Step 1: Verify Alignment Succeeded

**Check at startup:**
```
g_sensor_offset_u16           - Should be valid encoder count
g_radian_offset_to_electric_zero - Should be reasonable (0-2π)
m_motor_status                - Should be READY (not CALIBRATION_FAILED)
```

**If alignment failed:**
- Motor won't start properly
- Open-loop angle will be wrong from the beginning
- Transition will definitely fail

### Step 2: Monitor Transition Approach

**Watch these as motor accelerates:**
```
g_debug_measured_mech_rad_per_sec  - Velocity (should ramp smoothly)
g_open_loop_elec_before_blend      - Open-loop angle
g_closed_loop_elec_before_blend    - Encoder angle
```

**At ~5 rad/s (before transition):**
- Calculate angle difference: |open_loop - closed_loop|
- **If difference > 1.0 rad (57°):** Open-loop has drifted significantly
- **If difference > 2.0 rad (115°):** Transition will likely fail

### Step 3: Watch the Transition

**During 6-8 rad/s transition:**
```
g_blend_factor                - Should smoothly 0.0 → 1.0
g_blended_angle_after_rate_limit - Final angle (should be continuous)
g_amperage_q                  - Q-axis current (watch for spikes)
g_debug_measured_mech_rad_per_sec - Velocity (should stay smooth)
```

**Signs of problems:**
- Blend factor oscillating (velocity estimate noisy)
- Blended angle has discontinuities (phase mismatch)
- Q-axis current spikes >5A (motor fighting phase error)
- Velocity drops suddenly (losing synchronization)

### Step 4: Check Post-Transition

**After 8 rad/s (should be pure closed-loop):**
```
g_blend_factor = 1.0          - Fully closed-loop
g_cached_encoder_angle        - Should be used for FOC
```

**If motor runs smoothly:**
- ✅ Transition succeeded
- ✅ Closed-loop control is working
- Continue to target speed

**If motor still unstable:**
- ❌ Problem is NOT the transition
- ❌ Issue is with closed-loop control itself
- Check: Kalman velocity lag, PI gains, feedforward

## Solutions

### Solution 1: Wider Transition Zone (Current)

**Settings:**
```cpp
const float SPEED_THRESHOLD_LOW = 6.0f;
const float SPEED_THRESHOLD_HIGH = 8.0f;  // 2 rad/s transition zone
```

**Pros:**
- Gentler blend over longer period
- More time for angle synchronization
- Reduces sudden phase jumps

**Cons:**
- Longer time in "uncertain" zone
- More opportunity for drift to accumulate

### Solution 2: Narrower Transition Zone

**Settings:**
```cpp
const float SPEED_THRESHOLD_LOW = 7.0f;
const float SPEED_THRESHOLD_HIGH = 7.5f;  // 0.5 rad/s transition
```

**Pros:**
- Quick transition, less time for problems
- Motor committed to one mode or the other

**Cons:**
- Rapid phase change if angles don't match
- Requires better angle synchronization

### Solution 3: Pre-Transition Sync (Implemented)

**Code added at line ~2822:**
```cpp
// Re-synchronize open-loop angle before entering transition
if (measured_speed > (SPEED_THRESHOLD_LOW - 1.0f) && 
    measured_speed < SPEED_THRESHOLD_LOW)
{
    // Gently blend open-loop toward encoder
    open_loop_angle = open_loop_angle * 0.95f + encoder_angle * 0.05f;
}
```

**Pros:**
- Minimizes phase mismatch at transition start
- Smoother blending when transition begins
- Motor already partially following encoder

**Cons:**
- Slightly affects open-loop performance
- Requires tuning of sync factor

### Solution 4: Skip Open-Loop (Advanced)

**For motors with:**
- Low cogging torque
- Light loads at startup
- Good encoder accuracy at zero speed

**Implementation:**
```cpp
// Pure closed-loop from startup
const float SPEED_THRESHOLD_LOW = 0.1f;
const float SPEED_THRESHOLD_HIGH = 0.1f;
```

**Pros:**
- No transition problems!
- Simpler control structure
- Encoder accuracy from the start

**Cons:**
- Motor may not start (field aligned with rotor = zero torque)
- Needs different startup strategy (force alignment pulse)

### Solution 5: I-f Startup (Alternative Method)

**Different approach entirely:**
```
1. Apply fixed voltage at increasing frequency (I-f control)
2. Ramp frequency from 0-10 Hz over 2-3 seconds
3. Motor accelerates following the rotating field
4. At sufficient speed, switch to encoder-based FOC
5. No angle blending needed - discrete switch
```

**Pros:**
- Proven method for variable frequency drives
- Motor naturally synchronizes to rotating field
- Can handle higher inertia loads

**Cons:**
- Requires different control code
- Less efficient at low speeds
- Fixed voltage may not be optimal for all loads

## Tuning Guide

### If Motor Stalls at Transition

**Try these in order:**

1. **Raise transition speed:**
   ```cpp
   SPEED_THRESHOLD_LOW = 8.0f;   // Was 6.0f
   SPEED_THRESHOLD_HIGH = 10.0f; // Was 8.0f
   ```

2. **Widen transition zone:**
   ```cpp
   SPEED_THRESHOLD_LOW = 6.0f;
   SPEED_THRESHOLD_HIGH = 10.0f;  // 4 rad/s transition
   ```

3. **Increase sync factor (more aggressive pre-sync):**
   ```cpp
   float sync_factor = 0.1f;  // Was 0.05f (10% per cycle)
   ```

4. **Slow down acceleration through transition:**
   ```cpp
   // In main_cpp.cpp
   rps += 0.1f;  // Was 0.2f (slower = 1 rad/s² accel)
   ```

### If Transition is Rough but Succeeds

**Symptoms:**
- Motor makes noise during 6-8 rad/s
- Current spikes but doesn't stall
- Eventually runs smooth above 8 rad/s

**Solutions:**

1. **Smoother blend curve (ease-in-ease-out):**
   ```cpp
   // Replace linear blend with cubic
   float t = blend_factor;
   blend_factor = t * t * (3.0f - 2.0f * t);
   ```

2. **Increase sync region:**
   ```cpp
   const float SYNC_MARGIN = 2.0f;  // Was 1.0f
   // Start syncing 2 rad/s before transition
   ```

3. **Rate limit angle changes:**
   ```cpp
   // Already implemented - verify it's active
   // Check g_angle_change_limited vs g_blended_angle
   ```

### If Open-Loop Drifts Too Much

**Symptoms:**
- Large angle difference at 5-6 rad/s
- Open-loop and encoder angles differ by >1 rad

**Causes:**
- Open-loop using target velocity (not actual)
- Motor slipping (load too high)
- Encoder direction inverted

**Solutions:**

1. **Use measured velocity for open-loop (not target):**
   ```cpp
   // Instead of: angle_increment = target_velocity * dt
   angle_increment = measured_velocity * dt;  // Tracks actual speed
   ```

2. **Check encoder direction:**
   ```cpp
   // Verify m_sensor.invert_output() is correct
   // alignSensor() should have set this properly
   ```

3. **Reduce open-loop duration:**
   ```cpp
   SPEED_THRESHOLD_LOW = 4.0f;  // Transition earlier
   ```

## Alternative: Pure Closed-Loop Startup

If transition continues to be problematic, consider this approach:

### Force Alignment + Direct Closed-Loop

```cpp
void startup_force_alignment()
{
    // 1. Apply strong field at electrical angle = 0 for 500ms
    setPhaseVoltage(align_voltage, 0.0f, 0.0f);
    HAL_Delay(500);
    
    // 2. Rotor is now aligned to 0° electrical
    // 3. Read encoder, this is your zero offset
    float zero_offset = m_sensor.read_angle_radians();
    
    // 4. Set zero offset
    m_radian_offset_to_electric_zero = zero_offset;
    
    // 5. Start closed-loop immediately at low speed
    // No open-loop needed - rotor is already aligned!
    for (float speed = 0.1f; speed < 10.0f; speed += 0.1f)
    {
        update_target_rad_per_sec(speed);
        HAL_Delay(100);  // Ramp over 10 seconds
    }
}
```

**Advantages:**
- No transition zone
- No angle blending
- Simpler control logic
- Motor always knows rotor position

**Disadvantages:**
- Rotor moves during alignment (may not be acceptable)
- Requires specific startup sequence
- Can't start "on the fly" if motor is already spinning

## Recommended Configuration

**For most applications with sensor alignment:**

```cpp
// In StepperMotor.cpp
const float SPEED_THRESHOLD_LOW = 6.0f;   // Sufficient momentum
const float SPEED_THRESHOLD_HIGH = 8.0f;  // Safe transition
const float SYNC_MARGIN = 1.0f;           // Pre-sync before transition

// In main_cpp.cpp
rps += 0.2f;  // 2 rad/s² acceleration
```

**Key points:**
- Align sensor at startup (already implemented via alignSensor())
- Build momentum in open-loop to 6 rad/s
- Pre-sync angles starting at 5 rad/s
- Gentle transition over 6-8 rad/s
- Pure closed-loop above 8 rad/s

## Success Criteria

**Transition is working correctly when:**

✅ Motor accelerates smoothly from 0 → 10 rad/s  
✅ No stalling or hesitation at 6-8 rad/s  
✅ Velocity measurement stays smooth throughout  
✅ Q-axis current stays reasonable (<3A typical)  
✅ Blend factor smoothly progresses 0.0 → 1.0  
✅ Motor runs stably in closed-loop above 8 rad/s  

**If these criteria aren't met, revisit the diagnostic steps above.**

---

**Current Status:** Transition set to 6-8 rad/s with pre-sync at 5 rad/s  
**Next Test:** Ramp motor from 0 to 15 rad/s and observe transition behavior  
**Monitor:** g_blend_factor, angle differences, Q-axis current during 6-8 rad/s zone