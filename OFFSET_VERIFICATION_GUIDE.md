# Offset Verification and Angle Pipeline Diagnostic Guide

## Problem Summary

Motor runs rough with all electrical angles stuck at constant values, but encoder (`g_as5048_angle`) updates correctly. This indicates the angle calculation pipeline has an issue.

You mentioned: "I've had other tests where the motor spun far faster, quieter, and with less current."

This suggests the offset or angle transformation is incorrect, causing poor commutation efficiency.

---

## Angle Transformation Pipeline

The correct flow should be:

```
1. Raw Encoder Read
   ↓
   g_as5048_angle (mechanical, 0 to 2π)
   
2. Apply Direction Inversion (if needed)
   ↓
   m_sensor.get_mechanical_phase_angle_radians()
   
3. Convert Mechanical → Electrical
   ↓
   electic_radians = mech_angle × pole_pairs (50)
   g_encoder_elec_angle_raw (0 to 100π, then normalized to 0 to 2π)
   
4. Subtract Calibration Offset
   ↓
   raw_angle = electic_radians - m_radian_offset_to_electric_zero
   
5. Normalize to [0, 2π)
   ↓
   result = normalize_radians(raw_angle)
   g_encoder_elec_after_offset
   
6. Cache for Control Loop
   ↓
   g_cached_encoder_angle (used in loopFOC at 10kHz)
   
7. Use in Blending
   ↓
   g_closed_loop_elec_before_blend
   
8. Blend with Open-Loop
   ↓
   g_blended_angle_before_rate_limit
   
9. Rate Limit
   ↓
   g_blended_angle_after_rate_limit
   
10. Apply to Motor
    ↓
    setPhaseVoltage(Uq, Ud, electrical_angle)
```

---

## Critical Variables to Monitor

### Add ALL of these to STM32CubeMonitor:

```
Priority 1 - Execution Flow:
☐ g_loopfoc_update_count              // Is loopFOC() running? (Should be ~10k/sec)
☐ g_update_speed_cl_call_count        // Is update_speed_closed_loop() running? (~1k/sec)
☐ g_get_electric_angle_call_count     // Is get_electric_angle_radians() called? (~10k/sec)

Priority 2 - Angle Pipeline:
☐ g_as5048_angle                      // Raw encoder (KNOWN GOOD - shows sawtooth)
☐ g_encoder_mech_angle_raw            // After get_mechanical_phase_angle_radians()
☐ g_encoder_elec_angle_raw            // After mech→elec conversion (before offset)
☐ g_calibration_offset_elec           // The offset value (constant)
☐ g_encoder_elec_after_offset         // After offset subtraction
☐ g_cached_encoder_angle              // Cached in loopFOC()
☐ g_loopfoc_encoder_read_debug        // Same as cached (for verification)

Priority 3 - Blending Variables:
☐ g_closed_loop_elec_before_blend     // Encoder angle input to blending
☐ g_open_loop_elec_before_blend       // Accumulated angle input to blending
☐ g_blended_angle_before_rate_limit   // Blended result
☐ g_blended_angle_after_rate_limit    // Final commutation angle
☐ g_hybrid_blend_factor               // Blend ratio (0.0 to 1.0)

Priority 4 - Sensor Health:
☐ g_fetch_radians_success_count       // Successful reads
☐ g_fetch_radians_fail_count          // Failed reads (should be ~0)
```

---

## Expected Behavior (When Working Correctly)

### Mechanical Angle
```
Time    g_as5048_angle    g_encoder_mech_angle_raw    Match?
────────────────────────────────────────────────────────────
57s     4.123             4.123                       ✓ YES
58s     5.234             5.234                       ✓ YES
59s     0.345             0.345                       ✓ YES (wrapped)
60s     1.456             1.456                       ✓ YES
```

**If they DON'T match:** Two different sensor read paths exist!

---

### Electrical Angle (Before Offset)

With 50 pole pairs, electrical angle cycles 50× faster than mechanical:

```
g_encoder_elec_angle_raw = g_encoder_mech_angle_raw × 50

Then normalized to [0, 2π):
result = fmod(raw_value, 2π)
```

**Expected pattern:** Rapid sawtooth (0→2π) wrapping 50 times per mechanical rotation

```
Time    g_encoder_mech_angle_raw    g_encoder_elec_angle_raw
─────────────────────────────────────────────────────────────
57.00s  0.100 rad                   5.000 rad (wrapped)
57.01s  0.150 rad                   1.500 rad (wrapped)
57.02s  0.200 rad                   4.000 rad (wrapped)
```

**If stuck at constant:** Mechanical angle not updating or conversion failing

---

### Calibration Offset

This is calculated during alignment and should be CONSTANT:

```
g_calibration_offset_elec = m_radian_offset_to_electric_zero
```

**Typical value:** 0 to 2π rad (depends on rotor position during alignment)

**Your current value:** Unknown - CRITICAL TO CHECK!

**If offset is wrong:**
- Motor produces no torque (field aligned with magnet)
- Motor runs rough (field misaligned)
- High current, low efficiency

---

### Electrical Angle (After Offset)

```
g_encoder_elec_after_offset = normalize(g_encoder_elec_angle_raw - g_calibration_offset_elec)
```

**Expected:** Should show sawtooth pattern (0→2π) synchronized with current for maximum torque

**If stuck at constant:** This is your current problem!

---

## Diagnostic Scenarios

### Scenario 1: All Angles Update Correctly (Ideal)

```
g_as5048_angle:                    SAWTOOTH (0→2π, slow)      ✓
g_encoder_mech_angle_raw:          SAWTOOTH (matches above)   ✓
g_encoder_elec_angle_raw:          SAWTOOTH (0→2π, fast 50×)  ✓
g_encoder_elec_after_offset:       SAWTOOTH (0→2π, fast 50×)  ✓
g_cached_encoder_angle:            SAWTOOTH (same as above)   ✓
g_closed_loop_elec_before_blend:   SAWTOOTH                   ✓
g_blended_angle_after_rate_limit:  SAWTOOTH                   ✓

Result: Motor runs smoothly, good torque, low current
```

---

### Scenario 2: Mech Angle Stuck (Current Problem?)

```
g_as5048_angle:                    SAWTOOTH (updates)         ✓
g_encoder_mech_angle_raw:          CONSTANT (4.82 rad)        ✗ PROBLEM!
g_encoder_elec_angle_raw:          CONSTANT                   ✗
g_encoder_elec_after_offset:       CONSTANT                   ✗
All downstream angles:             CONSTANT                   ✗

Diagnosis: get_mechanical_phase_angle_radians() returns stale/cached value
          OR loopFOC() not calling get_electric_angle_radians()

Solution: Check g_loopfoc_update_count and g_get_electric_angle_call_count
```

---

### Scenario 3: Offset Wrong But Angles Update

```
g_as5048_angle:                    SAWTOOTH                   ✓
g_encoder_mech_angle_raw:          SAWTOOTH                   ✓
g_encoder_elec_angle_raw:          SAWTOOTH                   ✓
g_calibration_offset_elec:         WRONG VALUE (e.g., 0.0)    ✗
g_encoder_elec_after_offset:       SAWTOOTH (wrong phase)     ⚠
g_cached_encoder_angle:            SAWTOOTH (wrong phase)     ⚠

Result: Motor runs but poorly
        - High current draw
        - Low torque
        - Rough operation
        - Inefficient (heat)

Solution: Recalibrate offset by running alignment again
```

---

### Scenario 4: Direction Inverted

```
All angles update correctly BUT:
Motor rotates opposite direction from commanded
OR torque is weak/inconsistent

Diagnosis: m_invert_output flag set incorrectly during calibration

Solution: Re-run alignSensor() calibration routine
```

---

## Offset Verification Tests

### Test 1: Check if Offset is Applied

**Current state in your graph:**
- g_as5048_angle = changing (good)
- All electrical angles = 4.82 (stuck)

**Question:** Is 4.82 rad the offset value itself?

**Add to monitor:** `g_calibration_offset_elec`

**If g_calibration_offset_elec = 4.82:**
→ Offset is being used as the angle (BUG!)
→ Should be: angle = raw - offset, not angle = offset

**If g_calibration_offset_elec ≠ 4.82:**
→ 4.82 is an initialization value that never updates

---

### Test 2: Verify Offset Calculation

**During alignment, offset is calculated as:**

```cpp
// Alignment holds motor at electrical angle 0 rad
// Reads encoder mechanical position
float mech_zero = m_sensor.get_mechanical_phase_angle_radians();

// Converts to electrical
m_radian_offset_to_electric_zero = mechanical_to_electrical_radians(mech_zero);
```

**Expected offset value:**
- Should be stable (constant after alignment)
- Typically 0 to 2π rad
- Depends on initial rotor position

**To verify offset is correct:**

1. Stop motor
2. Manually rotate rotor by hand
3. Monitor: `g_encoder_elec_after_offset`
4. Should see continuous sawtooth as rotor turns
5. Should complete one electrical cycle (0→2π) per pole pair

**If offset correct:** Electrical angle tracks rotor position smoothly
**If offset wrong:** Electrical angle jumps, discontinuities, or constant

---

### Test 3: Compare Good vs Bad Runs

**You mentioned:** "I've had other tests where the motor spun far faster, quieter, and with less current."

**Critical question:** What was different in the good runs?

Possibilities:
- Different calibration offset value
- Different alignment result
- Different pole pair count setting
- Code changes that affected angle calculation
- Motor started from different initial position

**Action:** 
1. Save current offset value: `g_calibration_offset_elec`
2. Re-run alignment/calibration
3. Compare new offset to old offset
4. Test motor performance with new offset

---

## Common Offset Problems

### Problem 1: Offset = 0.0 (Not Calibrated)

```
g_calibration_offset_elec = 0.0
```

**Symptom:** Motor has no torque or very weak torque
**Cause:** Alignment never ran or failed
**Solution:** Run alignSensor() during startup

---

### Problem 2: Offset = π/2 Off (90° Error)

**Symptom:** Motor runs but inefficiently, high current
**Cause:** d-axis and q-axis swapped in Park/Inverse Park
**Solution:** Add π/2 to offset or swap current transformation

---

### Problem 3: Offset Updates During Operation

**Symptom:** Motor stutters, torque varies
**Cause:** Offset being recalculated incorrectly
**Solution:** Ensure offset only set during alignment, not in control loop

---

### Problem 4: Direction Inversion Not Applied

**Symptom:** Motor torque opposite sign from commanded
**Cause:** Encoder direction not inverted when needed
**Solution:** Check `m_invert_output` flag, re-run calibration

---

## Quick Diagnostic Procedure (5 minutes)

### Step 1: Add These 5 Variables
```
g_loopfoc_update_count
g_encoder_mech_angle_raw
g_encoder_elec_angle_raw
g_calibration_offset_elec
g_encoder_elec_after_offset
```

### Step 2: Run Motor for 5 Seconds

### Step 3: Analyze Results

**Check A: Is loopFOC() running?**
```
g_loopfoc_update_count should increase by ~50,000 in 5 seconds
If NOT → loopFOC() not being called (motion control issue)
```

**Check B: Does mech angle update?**
```
g_encoder_mech_angle_raw should show sawtooth pattern
If STUCK → sensor read in control loop is broken
If UPDATES → Continue to Check C
```

**Check C: Does elec angle (raw) update?**
```
g_encoder_elec_angle_raw should show rapid sawtooth (50× faster)
If STUCK → mechanical_to_electrical_radians() failing
If UPDATES → Continue to Check D
```

**Check D: What is offset value?**
```
g_calibration_offset_elec should be constant (0 to 2π)
If = 4.82 → This might be the issue!
If reasonable → Continue to Check E
```

**Check E: Does elec angle (after offset) update?**
```
g_encoder_elec_after_offset should show sawtooth
If STUCK → offset subtraction or normalize failing
If UPDATES → Problem is downstream (caching or blending)
```

---

## Fixing Offset Issues

### Fix 1: Force Offset to Known Value (Temporary Test)

**File:** `StepperMotor.cpp` line ~3548

```cpp
// TEMPORARY: Override offset to test
float raw_angle = electic_radians - 0.0f;  // No offset
// float raw_angle = electic_radians - m_radian_offset_to_electric_zero;
```

**Test:** Motor should run (possibly with wrong torque direction)
**If this makes angles update:** Offset calculation is the issue

---

### Fix 2: Re-run Calibration

**Add to startup code:**

```cpp
// Force re-calibration on every boot
m_radian_offset_to_electric_zero = NOT_SET;
alignSensor();
```

**Monitor:** `g_calibration_offset_elec` during alignment
**Should see:** Value stabilize to constant after alignment completes

---

### Fix 3: Verify Pole Pair Count

**File:** Check motor initialization

```cpp
// Should be 50 for your motor
pole_pairs = 50;
```

**If wrong pole pair count:**
- Electrical angle calculation completely wrong
- Motor won't run or runs very poorly

---

### Fix 4: Check Direction Inversion

**During alignment, `need_inversion` is determined:**

```cpp
bool need_inversion = (final_elec > initial_elec);
m_sensor.invert_output(need_inversion);
```

**Verify:**
```
g_align_direction_inversion should be 0.0 or 1.0
```

**If inversion wrong:**
- Motor torque inverted
- Positive velocity command → negative rotation (or vice versa)

---

## Expected Motor Performance (When Fixed)

### Good Performance Indicators:
- ✅ Smooth rotation with no vibration
- ✅ Low current draw (< 2A at moderate speed)
- ✅ Quiet operation (minimal cogging noise)
- ✅ Fast acceleration response
- ✅ Stable speed holding
- ✅ All angle variables showing sawtooth patterns

### Bad Performance (Current State):
- ❌ Rough operation, vibration
- ❌ High current draw
- ❌ Angles stuck at constants
- ❌ Poor torque efficiency
- ❌ Noisy operation

---

## Summary

**Current Problem:** All electrical angles stuck at 4.82 rad

**Most Likely Causes (Ranked):**
1. **loopFOC() not running** (60% probability)
   - Check: `g_loopfoc_update_count` not increasing
   
2. **Sensor read returns stale value** (25% probability)
   - Check: `g_encoder_mech_angle_raw` stuck but `g_as5048_angle` updates
   
3. **Offset calculation wrong** (10% probability)
   - Check: `g_calibration_offset_elec` value and how it's used
   
4. **Downstream caching issue** (5% probability)
   - Check: Earlier angles update but `g_cached_encoder_angle` stuck

**Next Steps:**
1. Add the 5 critical variables from Step 1 above
2. Run motor for 5 seconds
3. Follow the diagnostic checks A through E
4. Report which check fails first
5. Apply appropriate fix based on diagnosis

**Key Insight:** Since `g_as5048_angle` updates correctly, the sensor hardware is fine. The problem is in the software angle pipeline between the sensor read and the motor commutation.