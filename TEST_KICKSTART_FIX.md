# Quick Test Guide: Kickstart Transition Fix

## What Was Fixed

**Problem:** PWM disruption at transition from kickstart to closed-loop speed control

**Solution:** Closed-loop kickstart using encoder feedback instead of open-loop angle calculation

**Expected Result:** Smooth, seamless transition with no angle discontinuity

---

## Quick Test (5 Minutes)

### 1. Build & Flash
```bash
cd C:\_projects\FOC\cube\project3
make clean && make
st-flash write build/project3.bin 0x8000000
```

### 2. Open STM32CubeMonitor

Add these 4 critical variables:
1. `g_kickstart_target_angle_elec`
2. `g_cached_encoder_angle`
3. `g_kickstart_angle_error`
4. `g_kickstart_actual_rotation_elec`

### 3. Run Motor

Start the motor (kickstart happens automatically in `initFOC()`)

### 4. Watch for Success

**During kickstart (0-2.5 seconds):**
- `g_kickstart_target_angle_elec` ramps from 0 → 4.71 rad ✓
- `g_cached_encoder_angle` follows within 0.3 rad ✓
- `g_kickstart_angle_error` stays small (< 0.5 rad) ✓

**At transition (t=2.5s):**
- **`g_cached_encoder_angle` is CONTINUOUS (no jump!)** ✓✓✓
- `g_kickstart_actual_rotation_elec` ≈ 4.5-4.9 rad ✓

**After transition:**
- Motor continues smoothly (no click, no hesitation) ✓
- Velocity ramps up normally ✓

---

## Pass/Fail Criteria

### ✅ PASS
- Angle continuous at transition (no jump > 0.05 rad)
- PWM smooth in Saleae (no glitches)
- No audible click or motor hesitation

### ❌ FAIL
- Sudden jump in `g_cached_encoder_angle` at t=2.5s
- PWM disruption visible in Saleae
- Motor clicks, jerks, or hesitates at transition

---

## Troubleshooting

### Motor Doesn't Move During Kickstart

**Check:** `g_kickstart_voltage_applied`
- If < 3V: Increase `BASE_KICKSTART_VOLTAGE` in code
- If > 3V: Check motor wiring, enable signal, PWM output

### Motor Oscillates/Vibrates

**Check:** `g_kickstart_angle_error`
- If oscillates wildly (±0.5 rad): Reduce `Kp_position` from 2.0 to 1.0
- If stable but motor vibrates: Increase `UPDATE_INTERVAL_US` to 20000

### Incomplete Rotation (< 270°)

**Check:** `g_kickstart_actual_rotation_elec`
- If < 4.5 rad: Increase `TOTAL_DURATION_US` from 2500000 to 3500000
- Or increase `BASE_KICKSTART_VOLTAGE` by 20-50%

### Angle Jump Still Occurs

**Verify:** `g_cached_encoder_angle` is updating during kickstart
- Should change every ~25-100μs
- If frozen: `loopFOC()` not running - check interrupts

---

## Saleae Comparison

### Before Fix
```
Phase A PWM: ▓▓▓░░░▓▓▓ → ▓█░░░░█▓▓  (glitch at transition)
Phase B PWM: ░░▓▓▓░░▓▓ → ░░░██░▓▓░  (disruption)
Phase C PWM: ▓░░▓▓▓░░▓ → █░░░░░▓░▓  (pattern broken)
                        ↑ transition point
```

### After Fix (Expected)
```
Phase A PWM: ▓▓▓░░░▓▓▓ → ▓▓▓░░░▓▓▓  (smooth continuation)
Phase B PWM: ░░▓▓▓░░▓▓ → ░░▓▓▓░░▓▓  (no disruption)
Phase C PWM: ▓░░▓▓▓░░▓ → ▓░░▓▓▓░░▓  (pattern maintained)
                        ↑ seamless transition
```

---

## Key Diagnostic Variable

**Most Important:** `g_cached_encoder_angle`

This ONE variable tells you if the fix worked:
- **Continuous at t=2.5s** → ✅ Fix successful
- **Jumps at t=2.5s** → ❌ Fix not working

Graph it in CubeMonitor and zoom in on the 2.5s mark. You should see a smooth line with NO vertical jump.

---

## What Changed in Code

**File:** `CM7/Core/Src/motors/StepperMotor.cpp`
**Function:** `kickstartMotor()` (line ~1414)

**Before:**
```cpp
// Open-loop: command calculated angles
float elec_angle = calculate_from_step_number(step);
setPhaseVoltage(VOLTAGE, 0, elec_angle);
```

**After:**
```cpp
// Closed-loop: use actual encoder angle
float actual_angle = g_cached_encoder_angle;
float voltage = BASE_VOLTAGE + Kp * (target - actual);
setPhaseVoltage(voltage, 0, actual_angle);  // ← Uses REAL position
```

**Result:** Both kickstart and CL use `g_cached_encoder_angle` → no mismatch → no discontinuity!

---

## Full Documentation

For detailed analysis and tuning:
- `KICKSTART_TRANSITION_ANGLE_ANALYSIS.md` - Root cause analysis
- `KICKSTART_MONITORING_GUIDE.md` - Complete monitoring guide
- `KICKSTART_FIX_SUMMARY.md` - Implementation summary

---

## Expected Test Duration

- **Build/flash:** 30 seconds
- **Setup monitoring:** 1 minute
- **Run test:** 5 seconds (kickstart + transition)
- **Verify results:** 1 minute

**Total: ~3 minutes**

---

## Success Indicator

**If the fix worked, you'll see this in your graphs:**

```
g_cached_encoder_angle:
    ╱
   ╱
  ╱
 ╱    ← Smooth continuous line through transition
╱
└────────────────────────────────
0s   1s   2s  2.5s  3s   4s   5s
            ↑
       transition point
     (no visible jump!)
```

**Status:** Ready for testing
**Confidence:** High (addresses root cause directly)
**Risk:** Low (uses existing infrastructure)

---

## One-Line Summary

**Uses encoder feedback during kickstart instead of calculated angles, ensuring kickstart and closed-loop use the same angle source, eliminating the discontinuity.**