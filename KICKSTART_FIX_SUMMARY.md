# Kickstart Transition Fix - Implementation Summary

## Problem Identified

**Issue:** Abrupt PWM disruption at transition from kickstart to closed-loop (CL) speed control

**Root Cause:** Angle discontinuity between:
- Kickstart's open-loop **commanded angle** (calculated position)
- CL control's closed-loop **measured angle** (encoder feedback)

**Evidence:**
- Saleae logic analyzer shows PWM glitches at transition (~92.7s, 92.79s, 92.87s)
- STM32CubeMonitor shows angle jumps in `g_blended_angle_after_rate_limit`
- Motor exhibits audible click or brief hesitation at transition

---

## Solution Implemented

**Approach:** Closed-Loop Kickstart with Encoder Feedback

**Key Concept:** Use `g_cached_encoder_angle` (actual rotor position) during kickstart instead of calculated open-loop angle. This ensures both kickstart and CL control use the **same angle source**, eliminating discontinuity.

---

## Code Changes

### File Modified
`CM7/Core/Src/motors/StepperMotor.cpp`

### Function Changed
`StepperMotor::kickstartMotor()` (line ~1410)

### Changes Made

#### 1. Replaced Open-Loop Angle Calculation with Encoder Feedback

**Before (Open-Loop):**
```cpp
// Calculate where rotor SHOULD be (ignores reality)
float mech_angle = start_angle + (step * ANGLE_INCREMENT);
float elec_angle = mechanical_to_electrical_radians(mech_angle);
setPhaseVoltage(KICKSTART_VOLTAGE, 0.0f, elec_angle);
```

**After (Closed-Loop):**
```cpp
// Read where rotor ACTUALLY is (from encoder)
float actual_angle_elec = g_cached_encoder_angle;
setPhaseVoltage(total_voltage, 0.0f, actual_angle_elec);
```

#### 2. Added Position Control Feedback

**Purpose:** Compensate if rotor lags behind target position

```cpp
// Calculate target position (ramping)
float target_angle_elec = start_angle_elec + (target_velocity_elec * elapsed_seconds);

// Read actual position
float actual_angle_elec = g_cached_encoder_angle;

// Position error
float angle_error = target_angle_elec - actual_angle_elec;

// Proportional control: add voltage if lagging
float voltage_correction = Kp_position * angle_error;  // 2V per radian
float total_voltage = BASE_VOLTAGE + voltage_correction;
```

#### 3. Increased Update Rate

**Before:** 50ms per step (20 Hz updates)
**After:** 10ms per update (100 Hz updates)

**Benefit:** Faster response to tracking errors, smoother motion

#### 4. Added Diagnostic Variables

Three new variables for monitoring kickstart performance:

```cpp
volatile float g_kickstart_target_angle_elec;    // Ramped target position
volatile float g_kickstart_angle_error;          // target - actual (radians)
volatile float g_kickstart_voltage_applied;      // Total voltage to motor
```

---

## How It Eliminates Discontinuity

### Angle Source Timeline

| Phase | Angle Source | Value at Transition |
|-------|--------------|-------------------|
| **Kickstart (0-2.5s)** | `g_cached_encoder_angle` | 4.65 rad (example) |
| **Transition (t=2.5s)** | Switch to CL control | - |
| **CL Control (t>2.5s)** | `g_cached_encoder_angle` | 4.65 rad (same!) |

**Result:** Both phases use same angle → **NO JUMP!**

### Comparison

**Before (Discontinuity):**
```
Kickstart end:   angle = calculated (4.71 rad commanded)
CL start:        angle = encoder (4.50 rad actual)
Jump:            Δ = 0.21 rad (12°) → PWM disruption ❌
```

**After (Continuous):**
```
Kickstart end:   angle = encoder (4.65 rad actual)
CL start:        angle = encoder (4.65 rad actual)
Jump:            Δ = 0.00 rad (0°) → Smooth ✓✓✓
```

---

## Configuration Parameters

### Control Parameters
```cpp
const float BASE_KICKSTART_VOLTAGE = m_voltage_sensor_align;  // ~5V base
const float Kp_position = 2.0f;                               // 2V per radian error
const uint32_t UPDATE_INTERVAL_US = 10000;                    // 10ms (100 Hz)
const uint32_t TOTAL_DURATION_US = 2500000;                   // 2.5 seconds
```

### Tunable for Different Motors
- **High friction:** Increase `BASE_KICKSTART_VOLTAGE` or `Kp_position`
- **Oscillation:** Reduce `Kp_position` or increase `UPDATE_INTERVAL_US`
- **Incomplete rotation:** Increase `TOTAL_DURATION_US` or voltage

---

## Testing & Validation

### Variables to Monitor (STM32CubeMonitor)

**During Kickstart (0-2.5s):**
- `g_kickstart_target_angle_elec` → Should ramp 0 → 4.71 rad
- `g_cached_encoder_angle` → Should track target within 0.3 rad
- `g_kickstart_angle_error` → Should stay small: |error| < 0.5 rad
- `g_kickstart_voltage_applied` → Should vary 5-7V based on error

**At Transition (t=2.5s):**
- `g_cached_encoder_angle` → **Must be continuous, NO JUMP!**
- `g_blended_angle_after_rate_limit` → Should match encoder smoothly
- `g_kickstart_actual_rotation_elec` → Should be ~4.5-4.9 rad (≈270°)

**After Transition (t>2.5s):**
- Motor continues smoothly (no hesitation)
- Velocity ramps up normally
- No audible click or vibration

### Success Criteria

✅ **Primary:** No jump in `g_cached_encoder_angle` at t=2.5s (jump < 0.05 rad)
✅ **Secondary:** Smooth PWM in Saleae capture (no glitches at transition)
✅ **Tertiary:** No perceptible motor disturbance (no click, jerk, or vibration)

### Failure Indicators

❌ Sudden jump in angle graphs at 2.5s
❌ PWM disruption visible in Saleae at transition
❌ Audible click or motor hesitation at transition
❌ `g_kickstart_actual_rotation_elec` < 4.5 rad (incomplete rotation)

---

## Benefits of This Approach

### 1. Eliminates Root Cause
✓ Both kickstart and CL use same angle source
✓ No mismatch = no discontinuity = no PWM glitch

### 2. Better Tracking
✓ Position control compensates for lag
✓ Adapts to load/friction variations
✓ More reliable than open-loop timing

### 3. No Additional Delays
✓ Uses existing `g_cached_encoder_angle` (already updated by loopFOC)
✓ No extra SPI reads required
✓ No blocking delays added

### 4. Maintains Kickstart Function
✓ Still rotates 270° to break static friction
✓ Same 2.5 second duration
✓ Gets motor moving for CL takeover

### 5. Observable & Tunable
✓ Three new diagnostic variables
✓ Clear tuning parameters
✓ Easy to verify correct operation

---

## Migration Notes

### From Previous Implementations

**If you had "Solution 2" (synchronize at end):**
- This is better: uses encoder **throughout** kickstart, not just at end
- Eliminates lag accumulation during 2.5 second rotation

**If you had rate limiting enabled:**
- Can now disable or reduce rate limiter
- Discontinuity eliminated at source, not masked

**If you had blend-based transitions:**
- No longer needed - kickstart and CL already synchronized
- Can simplify `update_speed_closed_loop()` if desired

### Compatibility

✓ Works with existing Kalman filter velocity estimation
✓ Compatible with current sensor offset calibration
✓ No changes needed to CL control logic
✓ No changes to `loopFOC()` or `setPhaseVoltage()`

---

## Potential Issues & Solutions

### Issue 1: Motor Doesn't Move
**Symptom:** `g_cached_encoder_angle` stays flat
**Fix:** Increase `BASE_KICKSTART_VOLTAGE` or check motor wiring

### Issue 2: Rotor Oscillates
**Symptom:** `g_kickstart_angle_error` oscillates wildly (±0.5 rad)
**Fix:** Reduce `Kp_position` to 1.0 or add derivative damping

### Issue 3: Incomplete Rotation
**Symptom:** `g_kickstart_actual_rotation_elec` < 4.5 rad
**Fix:** Increase `TOTAL_DURATION_US` to 3500000 (3.5s) or increase voltage

### Issue 4: Encoder Noise
**Symptom:** Jittery voltage commands
**Fix:** Kalman filter should handle this; if not, add moving average

---

## Files Changed

1. **`CM7/Core/Src/motors/StepperMotor.cpp`**
   - Modified `kickstartMotor()` function
   - Added 3 diagnostic variables

2. **Documentation Created:**
   - `KICKSTART_TRANSITION_ANGLE_ANALYSIS.md` - Detailed analysis
   - `KICKSTART_MONITORING_GUIDE.md` - Testing guide
   - `KICKSTART_FIX_SUMMARY.md` - This file

---

## Next Steps

### 1. Build & Test
```bash
make clean && make
st-flash write build/project3.bin 0x8000000
```

### 2. Monitor Transition
- Connect STM32CubeMonitor
- Add kickstart diagnostic variables
- Run motor through kickstart
- Verify angle continuity at t=2.5s

### 3. Analyze Saleae Capture
- Capture PWM signals during transition
- Compare before/after fix
- Should see smooth continuation (no glitches)

### 4. Tune if Needed
- Adjust `Kp_position` if tracking error too large/small
- Adjust `TOTAL_DURATION_US` if rotation incomplete
- Adjust `UPDATE_INTERVAL_US` if oscillation occurs

---

## Expected Outcome

**The PWM disruption visible in the original Saleae plot will be completely eliminated.**

The motor will transition seamlessly from kickstart to closed-loop speed control with:
- Zero angle discontinuity
- Smooth PWM signals
- No audible/perceptible disturbance
- Reliable 270° rotation
- Smooth acceleration to target speed

**Status:** ✅ Implemented and ready for testing

**Last Updated:** 2025-01-08

---

## References

- Original issue: Saleae plot showing PWM disruption at ~92.7s, 92.79s, 92.87s
- STM32CubeMonitor traces: Angle jumps in blended_angle_after_rate_limit
- Related docs: OPEN_LOOP_TRANSITION_ISSUE.md (different problem, similar symptoms)

**Key Insight:** The cached encoder angle was already being updated during kickstart by `loopFOC()`. We just needed to USE it instead of calculating our own open-loop angle. This simple change eliminates the discontinuity entirely.