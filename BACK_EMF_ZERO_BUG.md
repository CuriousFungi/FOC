# Back-EMF Zero Bug - Critical Finding

## Problem Summary

At 35 rad/s, the back-EMF calculation is producing **ZERO volts** instead of the expected ~26V. This completely disables feedforward compensation, forcing the controller to rely entirely on PI feedback, causing:
- High current oscillations (±3A swings)
- Poor efficiency
- Motor instability at high speeds
- Rough operation

---

## Observed Data (from graphs at 35 rad/s)

```
g_debug_amperage_q_for_bemf        = -3 to +3A (large oscillations, avg ~0A)
g_desired_voltage_q_before_clamp   = 20.7V
g_m_voltage_q_in_controller        = 20.6V
g_debug_back_emf_q_axis            = 0V (!!!) SHOULD BE ~26V
g_debug_perm_magnet_flux           = 0.015 Wb ✓ correct
```

---

## Expected vs Actual

### Expected Calculation:
```
At 35 rad/s mechanical:
ω_electrical = 35 × 50 pole_pairs = 1750 electrical rad/s
back_emf = ω_electrical × (L×I_q + Ψ_magnet)
         = 1750 × (0.0032×0 + 0.015)
         = 1750 × 0.015
         = 26.25V
```

### Actual Measurement:
```
g_debug_back_emf_q_axis = 0V (or near-zero)
```

**Difference: 26V missing!**

---

## Root Cause Hypotheses

### Hypothesis 1: measured_mechanical_rad_per_sec is ZERO (Most Likely)

**Problem:** The velocity measurement (`g_kalman_velocity`) might be zero or not being read correctly.

**Check:**
```cpp
// Line ~2617
const float measured_electrical_rad_per_sec = mechanical_to_electrical_radians(measured_mechanical_rad_per_sec);

// If measured_mechanical_rad_per_sec = 0:
measured_electrical_rad_per_sec = 0
back_emf = 0 × (anything) = 0V
```

**New debug variable added:** `g_debug_measured_mech_rad_per_sec`
- Monitor this! Should be ~35 rad/s
- If it's 0 → velocity feedback is broken
- If it's correct → problem is elsewhere

---

### Hypothesis 2: g_kalman_velocity is Not Being Read

**Problem:** The Kalman filter velocity might not be updating or being read incorrectly.

**Code path (line ~2576):**
```cpp
extern volatile float g_kalman_velocity;
float measured_mechanical_rad_per_sec = g_kalman_velocity;
```

**Check:**
- Is `g_kalman_velocity` updating in graphs? (should show sawtooth pattern)
- Is it being read at the right time?
- Is the `extern` declaration correct?

---

### Hypothesis 3: Velocity Sign is Wrong

**Problem:** If velocity is negative and calculation makes back-EMF negative, but it's being zeroed somewhere.

**Check:**
- `g_kalman_velocity` sign (should match motor direction)
- After `mechanical_to_electrical_radians()`, sign should be preserved
- Back-EMF should have same sign as velocity

---

### Hypothesis 4: Code Path Not Executing

**Problem:** The back-EMF calculation might be in a disabled `#if 0` block or early return.

**Already verified:** Line 2574 shows `#if 1` (enabled), so this path is active.

---

## Impact on System

### Without Back-EMF Compensation:

**Controller sees:**
```
base_voltage_q = measured_current_q × PHASE_RESISTANCE + fabs(0)
               = measured_current_q × 1.4Ω + 0V
               = ~3V (at 2A current)
```

**Reality needs:**
```
total_voltage = I×R + back_emf
              = 3V + 26V
              = 29V
```

**Missing: 26V!**

The PI controller tries to make up the difference with feedback correction:
```
desired_voltage = base_voltage_q + feedback_correction
                = 3V + 17V
                = 20V
```

But feedback is slow, oscillatory, and fights the wrong value.

---

## Why This Explains All Symptoms

### Symptom 1: Inductance Fix Had No Effect
- Inductance term is part of back-EMF calculation
- But entire back-EMF is zero!
- Changing L×I from 3.2×I to 0.0032×I doesn't matter when total is multiplied by zero

### Symptom 2: Current Oscillates ±3A
- No feedforward → controller relies on pure feedback
- PI controller hunts for correct voltage
- Large integral term builds up → oscillation

### Symptom 3: Motor Stops at High Speed
- Back-EMF grows with speed, but controller doesn't know
- Applies constant ~20V regardless of speed
- At high speed, 20V isn't enough → motor loses torque → stops

### Symptom 4: Rough Operation
- Feedforward should provide smooth, predictable voltage
- Without it, controller is reactive, not predictive
- Results in torque ripple and vibration

---

## Diagnostic Steps

### Step 1: Add New Debug Variable
**Already done** - `g_debug_measured_mech_rad_per_sec` added to code

### Step 2: Rebuild and Monitor
Add to STM32CubeMonitor:
```
g_debug_measured_mech_rad_per_sec   → Should match actual speed (~35 rad/s)
g_kalman_velocity                   → Should match above
g_debug_measured_elec_rad_per_sec   → Should be mech × 50 (~1750 rad/s)
g_debug_back_emf_q_axis             → Should be ~26V, currently shows 0V
```

### Step 3: Identify Where Calculation Breaks

**If g_debug_measured_mech_rad_per_sec = 0:**
→ Problem: Velocity measurement is zero
→ Check: `g_kalman_velocity` updates
→ Fix: Ensure Kalman filter is running and updating

**If g_debug_measured_mech_rad_per_sec ≈ 35 rad/s:**
→ Velocity measurement is correct
→ Problem: Something between velocity and back-EMF is broken
→ Check: `g_debug_measured_elec_rad_per_sec` (should be 1750 rad/s)

**If g_debug_measured_elec_rad_per_sec ≈ 1750 rad/s:**
→ Electrical speed calculation correct
→ Problem: Final back-EMF calculation is broken
→ Check: Line 2619-2620 calculation logic

---

## Expected Fix

Once velocity measurement is correct:

**Before (broken):**
```
back_emf = 0V
feedforward = I×R + 0 = 3V
feedback = 17V (fighting to compensate)
total = 20V
Current: ±3A oscillations
```

**After (fixed):**
```
back_emf = 26V
feedforward = I×R + 26V = 29V (but clamped to 24V)
feedback = small correction (< 2V)
total = 24V (at limit, but expected)
Current: steady 1-2A
```

---

## Immediate Actions

1. **Rebuild code** with new `g_debug_measured_mech_rad_per_sec` variable
2. **Monitor** at 35 rad/s:
   - `g_debug_measured_mech_rad_per_sec`
   - `g_kalman_velocity`
   - `g_debug_measured_elec_rad_per_sec`
   - `g_debug_back_emf_q_axis`
3. **Identify** which variable is zero/wrong
4. **Fix** the broken calculation step
5. **Verify** back-EMF shows ~26V at 35 rad/s after fix

---

## Related Issues This Will Fix

Once back-EMF calculation works:
- ✅ Smooth high-speed operation (feedforward provides correct voltage)
- ✅ Low, stable current (no oscillations)
- ✅ Efficient operation (less I²R losses)
- ✅ Inductance fix will become relevant (L×I term will contribute)
- ✅ Motor reaches higher speeds (proper voltage compensation)

---

## Summary

**Critical Bug:** Back-EMF calculation produces 0V instead of 26V at 35 rad/s

**Most Likely Cause:** `measured_mechanical_rad_per_sec` is zero (velocity not being read)

**Impact:** Complete loss of feedforward compensation → controller uses only feedback → oscillations and poor performance

**Next Step:** Monitor `g_debug_measured_mech_rad_per_sec` to confirm velocity is being read correctly

**Expected:** This is THE root cause of all high-speed problems. Fixing this will dramatically improve motor performance! 🎯