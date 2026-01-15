# Stuck Angle Diagnostic - All Electrical Angles Frozen at 4.82 rad

## Problem Summary

All electrical angle debug variables are stuck at a constant **4.82 radians**:
- `g_closed_loop_elec_before_blend` = 4.82
- `g_open_loop_elec_before_blend` = 4.82
- `g_blended_angle_after_rate_limit` = 4.82
- `g_blended_angle_before_rate_limit` = 4.82
- `g_loopfoc_encoder_read_debug` = 4.82

**BUT:**
- Motor IS rotating (rough operation)
- `g_as5048_angle` IS changing (encoder reading works)
- Duty cycles show patterns (but distorted)

This indicates the **commutation angle is stuck**, causing poor motor performance.

---

## Root Cause Analysis

### Hypothesis 1: Early Return in update_speed_closed_loop()

**Location:** `StepperMotor.cpp` lines 2533-2539

```cpp
if (fabsf(actual_target_rps) < EPSILON_SPEED_RPS)
{
    g_debug_epsilon_trigger_count++;
    m_voltage.q = 0.0f;
    m_voltage.d = 0.0f;
    setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);
    return;  // ← EXITS BEFORE BLENDING CODE!
}
```

If `actual_target_rps < 0.0001`, the function returns early and **bypasses all angle blending logic**.

**Critical variables to monitor:**
1. `g_debug_actual_target_rps` - What is actual_target_rps value?
2. `g_debug_ramped_speed` - Is ramp working?
3. `g_debug_epsilon_trigger_count` - How often is early return hit?
4. `g_update_speed_cl_call_count` - Is function being called at all?

---

### Hypothesis 2: loopFOC() Not Updating Cached Angle

**Location:** `StepperMotor.cpp` line 1429

```cpp
float encoder_angle_from_get_electric = get_electric_angle_radians();
g_cached_encoder_angle = encoder_angle_from_get_electric;
g_loopfoc_encoder_read_debug = encoder_angle_from_get_electric;
```

If `g_loopfoc_encoder_read_debug` is stuck at 4.82, then `get_electric_angle_radians()` is returning a constant.

**Critical variables to monitor:**
1. `g_loopfoc_update_count` - Is loopFOC() being called?
2. `g_encoder_mech_angle_raw` - What is mechanical angle from sensor?
3. `g_encoder_elec_angle_raw` - Electrical angle before offset subtraction
4. `g_encoder_elec_after_offset` - Electrical angle after offset subtraction
5. `g_calibration_offset_elec` - What is the calibration offset?

---

### Hypothesis 3: Sensor Returns Constant Value

**Location:** `StepperMotor.cpp` line 3531

```cpp
float mech_angle = m_sensor.get_mechanical_phase_angle_radians();
float electic_radians = mechanical_to_electrical_radians(mech_angle);
float raw_angle = electic_radians - m_radian_offset_to_electric_zero;
float result = normalize_radians(raw_angle);
```

If mechanical angle is constant (but `g_as5048_angle` changes), there may be **two different sensor read paths**.

**Critical check:**
- Is `g_as5048_angle` from a DIFFERENT sensor read than the one used in `get_mechanical_phase_angle_radians()`?
- Are there cached/stale angle values being used?

---

### Hypothesis 4: Motion Control Type Wrong

**Location:** `StepperMotor.cpp` line 1468

```cpp
case MOTION_CONTROL_TYPE::CL_VELOCITY:
    update_speed_closed_loop(m_target, DELTA_T_SECONDS);
    break;
```

If motion control type is set to something other than `CL_VELOCITY`, then `update_speed_closed_loop()` may not be called at all!

**Critical variable to monitor:**
1. `m_motion_control` - What mode is active?
2. Add debug output at the start of each case statement

---

## Diagnostic Steps

### Step 1: Verify Function Execution

Add these variables to STM32CubeMonitor:

```
g_update_speed_cl_call_count    // Should increment continuously
g_loopfoc_update_count          // Should increment at 10kHz rate
```

**Expected:**
- `g_update_speed_cl_call_count` increases at 1kHz (1000/sec)
- `g_loopfoc_update_count` increases at 10kHz (10000/sec)

**If counters are NOT increasing:**
- Function not being called → check motion control mode

**If counters ARE increasing:**
- Functions executing but angles not updating → logic error inside functions

---

### Step 2: Check Early Return Condition

Add these variables to STM32CubeMonitor:

```
g_debug_actual_target_rps       // Ramped speed value
g_debug_ramped_speed            // Raw ramp value
g_debug_epsilon_trigger_count   // Early return counter
g_target_rps_to_cl_controller   // Commanded target
```

**Expected:**
- `g_debug_actual_target_rps` should be > 0.0001 when motor running
- `g_debug_epsilon_trigger_count` should NOT increase during motor operation

**If actual_target_rps < 0.0001:**
- Ramp not working or target speed too low
- **SOLUTION:** Increase target speed or disable epsilon check temporarily

---

### Step 3: Check Encoder Angle Pipeline

Add these variables to STM32CubeMonitor:

```
g_encoder_mech_angle_raw        // Mechanical angle from sensor
g_encoder_elec_angle_raw        // After mech→elec conversion
g_calibration_offset_elec       // Calibration offset value
g_encoder_elec_after_offset     // Final electrical angle
g_loopfoc_encoder_read_debug    // Cached in loopFOC
```

**Expected:**
- `g_encoder_mech_angle_raw` should match `g_as5048_angle` (or be related)
- `g_encoder_elec_angle_raw` should show sawtooth pattern (0→2π wrapping)
- `g_encoder_elec_after_offset` should show sawtooth (possibly phase-shifted)

**If g_encoder_mech_angle_raw is constant (but g_as5048_angle changes):**
- **TWO DIFFERENT SENSOR READ PATHS EXIST!**
- One path updates `g_as5048_angle` correctly
- Other path (used for control) returns stale/constant value

---

### Step 4: Check Blending Variables

Add these variables to STM32CubeMonitor:

```
g_hybrid_blend_factor           // Blend ratio (0-1)
g_hybrid_measured_speed         // Speed for blend calc
g_open_loop_elec_before_blend   // Open-loop angle
g_closed_loop_elec_before_blend // Closed-loop angle
```

**Expected:**
- Variables should UPDATE continuously
- Open/closed angles should show sawtooth patterns

**If all stuck at 4.82:**
- Blending code never executes (early return before blending)
- **SOLUTION:** Fix early return condition

---

## Quick Fix Attempts

### Fix 1: Disable Epsilon Check (Temporary)

**File:** `StepperMotor.cpp` line 2525

```cpp
// Comment out the early return
#if 0  // TEMPORARY DISABLE
if (fabsf(actual_target_rps) < EPSILON_SPEED_RPS)
{
    g_debug_epsilon_trigger_count++;
    m_voltage.q = 0.0f;
    m_voltage.d = 0.0f;
    setPhaseVoltage(m_voltage.q, m_voltage.d, 0.0f);
    return;  // ← This prevents blending!
}
#endif
```

**Rebuild and test.** If angles now update, the early return was the problem.

---

### Fix 2: Increase Initial Ramped Speed

**File:** `StepperMotor.cpp` line 2483

```cpp
// Change from 0.001 to higher value
static float ramped_speed = 1.0f;  // Start at 1.0 rad/s (well above epsilon)
```

**Rebuild and test.** If angles now update, ramp was starting too low.

---

### Fix 3: Force Blend Factor = 1.0 (Test Closed-Loop)

**File:** `StepperMotor.cpp` line ~2748

```cpp
// TEMPORARY: Force pure closed-loop
float blend_factor = 1.0f;  // Override calculation
```

**Rebuild and test.** This tests if closed-loop angle path works.

---

### Fix 4: Use Cached Encoder Angle Directly

**File:** `StepperMotor.cpp` line 2952

```cpp
// Instead of blended angle, use cached encoder directly
setPhaseVoltage(m_voltage.q,
                m_voltage.d,
                g_cached_encoder_angle);  // ← Use cached directly
```

**Rebuild and test.** This bypasses blending entirely.

---

## Expected Behavior (When Fixed)

### Angle Variables Should Show:

**Electrical angles (0 to 2π):**
```
Time    g_cached_encoder   g_open_loop    g_closed_loop   g_blended
        _angle             _elec          _elec           _after_limit
────────────────────────────────────────────────────────────────────
3.0s    1.23              1.18           1.23            1.20
3.1s    2.45              2.40           2.45            2.42
3.2s    3.67              3.62           3.67            3.64
3.3s    4.89              4.84           4.89            4.86
3.4s    6.11              6.06           6.11            6.08
3.5s    0.15              0.10           0.15            0.12  ← Wrapped
3.6s    1.37              1.32           1.37            1.34
```

All angles should:
- ✅ Show sawtooth pattern (0→2π→0)
- ✅ Track together (small differences during transition)
- ✅ Update continuously every sample
- ✅ NOT be constant!

---

## STM32CubeMonitor Sampling Rate

### Question: Can sampling rate be increased?

**Answer:** Yes, but limited by:

1. **STM32 ST-Link bandwidth:** ~1-10 kHz practical limit
2. **Number of variables:** More variables = lower rate per variable
3. **STM32CubeMonitor configuration:**
   - Open acquisition properties
   - Set "Acquisition Frequency" 
   - Try 1 kHz, 5 kHz, 10 kHz
   - Higher rates may cause data loss

**Recommendations:**
- Start with 1 kHz (1000 samples/sec)
- Reduce number of monitored variables (10-20 max)
- Use triggered acquisition to capture critical events
- For 10 kHz sampling, use on-chip logging to RAM and dump after

**For your angle issue:**
- 1 kHz is sufficient to see if angles are stuck or updating
- If they update at all, you'll see it even at 100 Hz

---

## Most Likely Cause

Based on symptoms:
1. Motor runs (rough) → voltage commands are being sent
2. Encoder works → `g_as5048_angle` updates
3. All electrical angles stuck → **control loop not executing properly**

**90% probability:** Early return at line 2533 due to `actual_target_rps < epsilon`

**Check:** `g_debug_epsilon_trigger_count` - is it incrementing rapidly?

**Fix:** Either:
- Disable epsilon check temporarily (`#if 0`)
- Increase initial `ramped_speed` to 1.0 rad/s
- Lower `EPSILON_SPEED_RPS` to 1.0e-6

---

## Next Steps

1. **Add diagnostic variables to monitor** (listed in Step 1-4 above)
2. **Run motor and capture 5 seconds of data**
3. **Check which hypothesis matches the data:**
   - If `g_debug_epsilon_trigger_count` increasing → Early return problem
   - If `g_loopfoc_update_count` not increasing → loopFOC() not called
   - If `g_encoder_mech_angle_raw` stuck → Sensor read path issue
4. **Apply appropriate fix from Quick Fix Attempts section**
5. **Verify angles now update** (sawtooth pattern)
6. **Motor should run smoothly** once angles update correctly

The angles being stuck at exactly 4.82 rad is actually **helpful** - it's a specific value that's probably the last valid angle before the code stopped updating. This is a logic/control-flow bug, not a hardware issue!

---

## Summary

**Problem:** Electrical angles frozen at 4.82 rad
**Symptom:** Rough motor operation, duty cycles distorted
**Root Cause:** Most likely early return preventing angle updates
**Solution:** Check epsilon condition, disable if needed
**Verify:** Monitor diagnostic variables listed above

**The blending code exists and is correct - it's just not executing!**