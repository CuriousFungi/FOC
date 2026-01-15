# Angle Freeze Diagnostics - All Electrical Angles Stuck at 4.82 rad

## Problem Statement

All electrical angle debug variables are frozen at **4.82 radians**:
- `g_closed_loop_elec_before_blend` = 4.82 (constant)
- `g_open_loop_elec_before_blend` = 4.82 (constant)  
- `g_blended_angle_after_rate_limit` = 4.82 (constant)
- `g_blended_angle_before_rate_limit` = 4.82 (constant)
- `g_loopfoc_encoder_read_debug` = 4.82 (constant)
- `g_cached_encoder_angle` = 4.82 (constant)

**However:**
- Motor IS rotating (rough operation)
- `g_as5048_angle` IS updating correctly (changes with motor rotation)
- Duty cycles show patterns (distorted due to fixed commutation angle)

**Critical Insight:** 4.82 rad ≈ 276° ≈ 0.768 × 2π. This is a specific angle, likely set during alignment phase and never updated thereafter.

---

## Root Cause Hypothesis

The issue is **NOT** the epsilon check (that's there for a good reason to avoid sin(0)/cos(0) discontinuities).

**Most Likely Causes:**

### Hypothesis 1: Sensor Read Path Mismatch
- `g_as5048_angle` is updated in one path (working)
- `get_mechanical_phase_angle_radians()` used by control returns stale/cached value
- Two different sensor read mechanisms exist

### Hypothesis 2: fetch_radians() Failing Silently
- `fetch_radians()` returns false (error bit set)
- `get_mechanical_phase_angle_radians()` returns 0.0f on failure
- But angles are stuck at 4.82, not 0.0... so this doesn't fit

### Hypothesis 3: Cached Angle Never Refreshed
- `g_cached_encoder_angle` set once during startup
- `loopFOC()` not being called, OR
- `get_electric_angle_radians()` not executing properly
- Variables retain initialization value (4.82 from alignment)

### Hypothesis 4: Code Path Not Executing
- `update_speed_closed_loop()` not being called
- Different motion control mode active
- Early return happening (but not the epsilon one)

---

## Critical Diagnostic Variables to Add

### Priority 1: Execution Flow Counters

Add these to STM32CubeMonitor to verify functions are executing:

```
g_loopfoc_update_count              // Should increment at 10kHz
g_update_speed_cl_call_count        // Should increment at 1kHz
g_get_electric_angle_call_count     // Tracks get_electric_angle_radians() calls
```

**Expected Values:**
- If running for 1 second:
  - `g_loopfoc_update_count` ≈ 10,000
  - `g_update_speed_cl_call_count` ≈ 1,000
  - `g_get_electric_angle_call_count` ≈ 10,000 (called from loopFOC)

**If any counter is NOT increasing → that function is not executing!**

---

### Priority 2: Sensor Read Path Verification

```
g_as5048_angle                      // External Kalman angle (KNOWN WORKING)
g_encoder_mech_angle_raw            // From get_electric_angle_radians()
g_encoder_elec_angle_raw            // After mech→elec conversion
g_encoder_elec_after_offset         // After offset subtraction
g_calibration_offset_elec           // Calibration offset value
```

**Critical Test:**
- Compare `g_as5048_angle` vs `g_encoder_mech_angle_raw`
- **If they're different:** Two sensor read paths exist! Control is using wrong one.
- **If g_encoder_mech_angle_raw is stuck:** Sensor read in control loop is broken.

---

### Priority 3: fetch_radians() Success/Fail Tracking

```
g_fetch_radians_success_count       // Increments on successful read
g_fetch_radians_fail_count          // Increments on failed read (error bit set)
g_fetch_radians_result_debug        // Last angle returned
g_fetch_radians_raw_count_debug     // Raw 16-bit value from sensor
```

**Critical Test:**
- If `g_fetch_radians_fail_count` is rapidly increasing:
  - Sensor communication failing
  - Error bit (0x4000) is set
  - `get_mechanical_phase_angle_radians()` returns 0.0f
- If `g_fetch_radians_success_count` is increasing:
  - Sensor reads working
  - But control may not be using the updated value

---

### Priority 4: Blend Factor and Speed

```
g_debug_actual_target_rps           // Ramped speed (after acceleration)
g_debug_ramped_speed                // Raw ramp value
g_debug_epsilon_trigger_count       // How many times early return triggered
g_hybrid_blend_factor               // Blend ratio (0.0 to 1.0)
g_hybrid_measured_speed             // Speed used for blend calculation
g_kalman_velocity                   // Kalman filtered velocity
```

**Critical Test:**
- If `g_debug_epsilon_trigger_count` increasing AND motor running:
  - Something's wrong with speed measurement
  - But epsilon early return is CORRECT (don't disable it!)
- If `g_debug_actual_target_rps` is large (>1 rad/s) but epsilon triggers:
  - Logic error in code

---

### Priority 5: Angle Update Path

```
g_open_loop_elec_before_blend       // Open-loop angle before blending
g_closed_loop_elec_before_blend     // Encoder angle before blending  
g_blended_angle_before_rate_limit   // After blending
g_blended_angle_after_rate_limit    // After rate limiting
g_cached_encoder_angle              // Cached in loopFOC()
g_loopfoc_encoder_read_debug        // Angle from get_electric_angle_radians()
```

**If ALL are stuck at 4.82:**
- Angle calculation never executes after initialization
- OR variables are never assigned (code path issue)

---

## Diagnostic Procedure

### Step 1: Verify Function Execution (30 seconds)

Run motor and monitor counters:

| Variable | Expected | If Not Increasing → |
|----------|----------|-------------------|
| `g_loopfoc_update_count` | +10,000/sec | loopFOC() not called |
| `g_update_speed_cl_call_count` | +1,000/sec | update_speed_closed_loop() not called |
| `g_get_electric_angle_call_count` | +10,000/sec | get_electric_angle_radians() not called |

---

### Step 2: Identify Sensor Read Path Issue (1 minute)

Compare sensor angles:

```
Time     g_as5048_angle    g_encoder_mech_angle_raw    Match?
─────────────────────────────────────────────────────────────
3.5s     1.234             1.234                       YES → Same path
3.6s     2.345             2.345                       YES → Same path
```

OR:

```
Time     g_as5048_angle    g_encoder_mech_angle_raw    Match?
─────────────────────────────────────────────────────────────
3.5s     1.234             4.820                       NO → DIFFERENT PATHS!
3.6s     2.345             4.820                       NO → Control using stale value
```

**If different paths detected:**
- `g_as5048_angle` updated by external Kalman filter
- `g_encoder_mech_angle_raw` returned by `m_sensor.get_mechanical_phase_angle_radians()`
- **Root cause:** Control loop not using correct sensor read!

---

### Step 3: Check fetch_radians() Health (1 minute)

Monitor success/fail counters:

**Scenario A: Reads succeeding**
```
g_fetch_radians_success_count: increasing rapidly (+10k/sec)
g_fetch_radians_fail_count: 0 or very low (<1% of success)
→ Sensor communication is WORKING
→ Problem is elsewhere (caching, code path)
```

**Scenario B: Reads failing**
```
g_fetch_radians_success_count: low or not increasing
g_fetch_radians_fail_count: increasing rapidly
→ Sensor communication FAILING
→ Error bit 0x4000 is set
→ Check SPI signals, power, magnet alignment
```

---

### Step 4: Verify Angle Pipeline (2 minutes)

Track angle through transformation pipeline:

```
g_encoder_mech_angle_raw        = 1.234 rad  (mechanical, 0 to 2π)
          ↓ multiply by pole_pairs (50)
g_encoder_elec_angle_raw        = 61.70 rad  (electrical, 0 to 100π)
          ↓ normalize to [0, 2π)
                                = 5.123 rad  (wrapped)
          ↓ subtract calibration offset
g_encoder_elec_after_offset     = 4.820 rad  (aligned to rotor)
          ↓
g_loopfoc_encoder_read_debug    = 4.820 rad  (cached in loopFOC)
          ↓
g_cached_encoder_angle          = 4.820 rad  (used for Park/Inverse Park)
          ↓
g_closed_loop_elec_before_blend = 4.820 rad  (used for blending)
```

**If values are stuck at any stage:**
- That function is not executing
- OR variable assignment is not happening

---

## Likely Root Causes (Ranked by Probability)

### 1. loopFOC() Not Executing (60% probability)

**Symptoms:**
- `g_loopfoc_update_count` not increasing
- `g_cached_encoder_angle` stuck at initialization value (4.82)
- All downstream angles stuck

**Cause:**
- Motion control mode not set to CL_VELOCITY
- loopFOC() disabled or early return
- Timer interrupt not firing

**Solution:**
- Check motion control mode setting
- Verify timer configuration
- Check for early returns in loopFOC()

---

### 2. Sensor Read Returns Stale Value (25% probability)

**Symptoms:**
- `g_loopfoc_update_count` IS increasing
- `g_encoder_mech_angle_raw` stuck but `g_as5048_angle` changing
- Two different read paths exist

**Cause:**
- `get_mechanical_phase_angle_radians()` returns cached/stale value
- Separate Kalman filter updates `g_as5048_angle` correctly
- Control loop uses wrong sensor interface

**Solution:**
- Make control loop use same sensor read as Kalman filter
- Or ensure `get_mechanical_phase_angle_radians()` reads fresh data

---

### 3. update_speed_closed_loop() Not Executing (10% probability)

**Symptoms:**
- `g_update_speed_cl_call_count` not increasing
- Blend variables never update
- Motor still runs (voltage commands from elsewhere)

**Cause:**
- Motion control mode not CL_VELOCITY
- Function not called from loopFOC()
- Different control mode active

**Solution:**
- Set motion control to CL_VELOCITY
- Verify function call in loopFOC() switch statement

---

### 4. Blending Code Not Executing (5% probability)

**Symptoms:**
- Functions executing (counters increasing)
- Angles stuck at 4.82 despite sensor working
- Early return happening for wrong reason

**Cause:**
- Logic error preventing blending code from running
- Conditional wrapping blend section evaluates to false
- Early return for unexpected reason

**Solution:**
- Add debug prints at start of blending section
- Check all conditionals before blend code
- Verify no #if 0 wrapping blend logic

---

## What 4.82 Radians Means

**4.82 rad = 276.2°**

This is likely from:
- **Alignment phase:** Motor held at specific angle during startup
- **Calibration offset:** Electrical angle offset calculated during alignment
- **Initialization:** Variables initialized to this value, never updated

**Why this specific value?**
- Alignment typically uses 0° or 90° electrical
- With 50 pole pairs and calibration offset, could result in 4.82 rad
- This was the "last good value" before angle updates stopped

---

## Action Plan

### Immediate Steps (Next 5 Minutes)

1. **Add these 3 variables to STM32CubeMonitor:**
   - `g_loopfoc_update_count`
   - `g_update_speed_cl_call_count`  
   - `g_encoder_mech_angle_raw`

2. **Run motor for 3 seconds**

3. **Check results:**
   - Are counters increasing? → Functions executing
   - Does `g_encoder_mech_angle_raw` change? → Sensor read working
   - Does it match `g_as5048_angle`? → Same read path

4. **Based on results, proceed to appropriate fix section below**

---

## Fix Scenarios

### Scenario A: Counters NOT Increasing

**If `g_loopfoc_update_count` stuck at 0:**
→ loopFOC() not being called
→ Check motion control mode, timer configuration

**If `g_update_speed_cl_call_count` stuck at 0:**
→ update_speed_closed_loop() not being called
→ Check motion_control switch statement in loopFOC()

---

### Scenario B: Counters Increasing, Angles Stuck

**If counters increase but `g_encoder_mech_angle_raw` stuck:**
→ `get_mechanical_phase_angle_radians()` returns constant
→ Check `fetch_radians()` success/fail counters
→ May be returning cached/stale value

**Solution:**
- Check if `g_fetch_radians_fail_count` increasing
- Verify sensor read actually executes in `get_mechanical_phase_angle_radians()`
- May need to add debug output inside function

---

### Scenario C: Sensor Reads Work, But Not Used

**If `g_as5048_angle` changes but `g_encoder_mech_angle_raw` stuck:**
→ Two different sensor read paths
→ Control uses wrong path

**Solution:**
- Identify where `g_as5048_angle` is updated (Kalman filter?)
- Make control loop use same read mechanism
- Or fix `get_mechanical_phase_angle_radians()` to read fresh data

---

## Important Notes

### DO NOT Disable Epsilon Check!

The epsilon check in `update_speed_closed_loop()` is CRITICAL:

```cpp
if (fabsf(actual_target_rps) < EPSILON_SPEED_RPS)
{
    // Set voltages to zero, angle to 0
    return;  // IMPORTANT: Prevents sin(0)/cos(0) discontinuities!
}
```

**This is NOT the bug.** The epsilon check:
- Prevents discontinuous trig functions at zero velocity
- Necessary for stable operation
- Has detailed comments warning against modification

**If motor is running at t=3.5s, ramped_speed is well above epsilon by then.**

---

### g_cached_encoder_angle Purpose

`g_cached_encoder_angle` serves a critical purpose:

**Why it exists:**
- `loopFOC()` runs at 10kHz (current sensing, Park transform)
- `update_speed_closed_loop()` runs at 1kHz (voltage command)
- Sensor read has jitter/timing issues

**Solution:**
- Read encoder ONCE per loopFOC() cycle
- Cache the value in `g_cached_encoder_angle`
- Use SAME angle for both Park (current→dq) and Inverse Park (voltage→abc)
- Ensures perfect synchronization between current sensing and voltage command

**DO NOT remove this caching mechanism!** It prevents phase mismatch artifacts.

---

## Summary

**Problem:** All electrical angles stuck at 4.82 rad
**NOT caused by:** Epsilon check (that's correct and necessary)
**Most likely cause:** loopFOC() not executing or sensor read returns stale value
**Diagnostic approach:** Add counters first, verify execution flow
**Fix approach:** Based on what diagnostic variables reveal

**Next step:** Add the 3 priority variables and run motor to see what's actually happening!