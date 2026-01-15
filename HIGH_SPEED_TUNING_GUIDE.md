# High-Speed Motor Tuning Guide - Back-EMF and Feedforward Compensation

## Problem Statement

Motor starts spinning fairly fast but then:
1. Hits a "rough patch" (oscillation/instability)
2. Stops completely (loses synchronization)

**Root Cause:** At higher speeds, back-EMF and inductive effects dominate, requiring proper compensation.

---

## Current Motor Parameters

**From `main_cpp.cpp` line ~121:**
```cpp
pole_pairs = 50
phase_resistance = 1.4 Ω
KV = 1.0 (NEEDS VERIFICATION!)
phase_inductance = 3.2 mH
voltage_limit = 20.0 V
power_supply = 20.0 V
```

**From `StepperMotor.cpp` line ~396:**
```cpp
PERM_MAGNET_FLUX_LINKAGE = 0.005 Wb (5 milliWebers) (NEEDS VERIFICATION!)
```

---

## Physics at High Speed

### Back-EMF (Counter-Electromotive Force)

**Formula:**
```
V_bemf = K_e × ω_electrical
K_e = permanent magnet flux linkage (Wb)
ω_electrical = mechanical speed × pole_pairs (electrical rad/s)
```

**Current setting:**
```
PERM_MAGNET_FLUX_LINKAGE = 0.005 Wb
```

**At 10 rad/s mechanical (moderate speed):**
```
ω_electrical = 10 × 50 = 500 electrical rad/s
V_bemf = 0.005 × 500 = 2.5 V
```

**At 30 rad/s mechanical (high speed):**
```
ω_electrical = 30 × 50 = 1500 electrical rad/s
V_bemf = 0.005 × 1500 = 7.5 V
```

**Problem:** If flux linkage constant is wrong, back-EMF compensation fails!

---

## Symptoms and Diagnosis

### Symptom 1: Motor Stops at Specific Speed

**Indicates:** Voltage saturation (can't overcome back-EMF)

**Happens when:**
```
V_bemf + I×R ≥ V_supply
```

**At your 20V supply:**
```
Available voltage = 20V
Back-EMF at 30 rad/s = 7.5V (assuming 0.005 Wb is correct)
Resistive drop at 2A = 2 × 1.4 = 2.8V
Total needed = 7.5 + 2.8 = 10.3V ✓ Should work

But if flux linkage is actually 0.010 Wb:
Back-EMF at 30 rad/s = 15V
Total needed = 15 + 2.8 = 17.8V ✓ Still OK

But if flux linkage is actually 0.015 Wb:
Back-EMF at 30 rad/s = 22.5V
Total needed = 22.5 + 2.8 = 25.3V ✗ SATURATES at 20V!
```

**Action:** Measure actual back-EMF constant!

---

### Symptom 2: Motor Oscillates Then Stops

**Indicates:** Feedforward mismatch or controller instability

**Happens when:**
- Back-EMF constant wrong → incorrect feedforward
- PI gains too high at high speed
- Phase lag compensation missing

**Current feedforward (line ~2610):**
```cpp
const float measured_electrical_rad_per_sec = mechanical_to_electrical_radians(measured_mechanical_rad_per_sec);
const float mag_flux_linkage_q = inductance(measured_electrical_rad_per_sec) * m_amperage.q;
const float back_emf_q_axis = measured_electrical_rad_per_sec
                            * (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE);

float base_voltage_q = measured_current_q * PHASE_RESISTANCE
                     + fabs(back_emf_q_axis);
```

**This is correct form!** But depends on accurate constants.

---

## Critical Constants to Verify

### 1. Permanent Magnet Flux Linkage (K_e)

**Current value:** `0.005 Wb` (line 396)

**How to measure:**

#### Method A: Spin Motor by Hand
1. Disconnect motor from driver (no power)
2. Connect oscilloscope to phase A and phase B
3. Spin motor shaft at known speed (use drill or hand-crank)
4. Measure peak-to-peak voltage on scope
5. Calculate:
   ```
   V_bemf_pk-pk = measured voltage (V)
   ω_mech = measured mechanical speed (rad/s)
   ω_elec = ω_mech × pole_pairs (electrical rad/s)
   
   K_e = V_bemf_pk-pk / (√2 × ω_elec)
   ```

#### Method B: Coast Down Test
1. Run motor at high speed in software
2. Set voltage to 0 (coast down)
3. Monitor motor terminals with scope
4. Measure generated voltage vs speed
5. Calculate K_e from slope

#### Method C: KV Rating (if known)
```
KV = RPM per volt (back-EMF constant)
K_e (electrical) = 60 / (2π × KV × pole_pairs) Wb

Example: If motor is 100 KV rating:
K_e = 60 / (2π × 100 × 50) = 0.00191 Wb
```

**Typical range for steppers:** 0.001 to 0.020 Wb

**Your current 0.005 Wb seems reasonable but MUST be verified!**

---

### 2. Phase Resistance

**Current value:** `1.4 Ω` (line 121)

**How to verify:**
1. Measure DC resistance with multimeter between phases
2. Should be ~1.4 Ω
3. At high frequency (10 kHz electrical), AC resistance increases due to skin effect

**Current code handles frequency-dependent resistance:**
```cpp
PHASE_RESISTANCE_1KHZ = 24.29 Ω   // AC impedance at 1 kHz
PHASE_RESISTANCE_10KHZ = 212 Ω    // AC impedance at 10 kHz
```

**Note:** These seem very high! At 10 kHz, 212 Ω is enormous.
- **Verify these measurements!**
- May include reactive (inductive) component
- Should use only resistive part for I×R calculation

---

### 3. Phase Inductance

**Current value:** `3.2 mH` (line 126)

**How to verify:**
1. LCR meter measurement (already done):
   - 3.9 mH at 1 kHz
   - 3.2 mH at 10 kHz
2. These values are reasonable for stepper motor

**At high speed, inductance limits current slew rate:**
```
di/dt = V/L

At 20V and 3.2 mH:
di/dt = 20 / 0.0032 = 6250 A/s

To change current by 2A:
Δt = 2 / 6250 = 0.32 ms = 320 μs

At 40 kHz control loop (25 μs period):
Can only change current by: 2A × 25μs/320μs = 0.156 A per iteration
```

**This limits transient response at high speed!**

---

## Diagnostic Procedure

### Step 1: Add High-Speed Debug Variables

Add to STM32CubeMonitor:

```
g_base_voltage_q_feedforward        // Feedforward voltage
g_feedback_voltage_correction       // Feedback correction
g_desired_voltage_q_before_clamp    // Total voltage before limit
g_m_voltage_q_in_controller         // Actual voltage after limit
g_velocity_error_for_debug          // Speed error
g_amperage_q                        // Q-axis current
g_kalman_velocity                   // Measured speed
g_target_rps_to_cl_controller       // Target speed
```

### Step 2: Run Motor to Failure Point

1. Start motor at low speed (5 rad/s)
2. Gradually increase speed command
3. Watch where it becomes rough
4. Note speed where it stops

**Record:**
- Speed at roughness onset: _______ rad/s
- Speed at motor stop: _______ rad/s
- `g_m_voltage_q_in_controller` at stop: _______ V
- `g_desired_voltage_q_before_clamp` at stop: _______ V

### Step 3: Check for Voltage Saturation

**If `g_desired_voltage_q_before_clamp > g_m_voltage_q_in_controller`:**
→ Voltage limit is saturating
→ Can't deliver commanded voltage
→ Motor loses torque

**Solutions:**
1. Increase voltage limit (if supply allows)
2. Reduce flux linkage constant (if it's overestimated)
3. Reduce maximum speed command

### Step 4: Check Feedforward vs Feedback

**Plot ratio:**
```
feedforward_ratio = g_base_voltage_q_feedforward / g_m_voltage_q_in_controller
```

**Ideal:** feedforward_ratio > 0.8 (feedforward dominates at steady state)

**If feedforward_ratio < 0.5:**
→ Feedforward is weak, feedback doing most of the work
→ Indicates flux linkage or resistance values are wrong

**If feedback correction oscillates:**
→ PI gains may be too high
→ Or feedforward mismatch causing hunting

---

## Tuning Procedure

### Fix 1: Measure and Correct Flux Linkage

**Currently:** `PERM_MAGNET_FLUX_LINKAGE = 0.005 Wb`

**Test values to try (line 396):**
```cpp
// Too low (weak back-EMF compensation):
PERM_MAGNET_FLUX_LINKAGE = 0.002f

// Current:
PERM_MAGNET_FLUX_LINKAGE = 0.005f

// Higher (stronger back-EMF compensation):
PERM_MAGNET_FLUX_LINKAGE = 0.010f

// Much higher:
PERM_MAGNET_FLUX_LINKAGE = 0.015f
```

**Procedure:**
1. Try 0.010 Wb first (double current value)
2. Test if motor runs faster/smoother
3. If still stops, try 0.015 Wb
4. If becomes unstable, reduce to 0.007 Wb
5. Binary search to optimal value

**Correct value when:**
- Motor reaches commanded speed smoothly
- `g_desired_voltage_q_before_clamp` doesn't saturate
- Minimal feedback correction at steady state

---

### Fix 2: Increase Voltage Limit (if available)

**Current:** `voltage_limit = 20.0 V`

**If you have higher supply voltage available:**
```cpp
// In main_cpp.cpp line ~126:
voltage_limit = 24.0f,         // voltage limit (if supply supports it)
power_supply_voltage = 24.0f,  // actual supply voltage
```

**Check:**
- L298N driver datasheet max voltage
- Actual power supply capability
- Motor insulation rating

**Note:** Higher voltage → more speed capability, but more heat!

---

### Fix 3: Reduce PI Gains at High Speed

**Current gains (line ~2649):**
```cpp
const float Kp_velocity = 0.5f;   // Proportional gain
const float Ki_velocity = 2.0f;   // Integral gain
```

**These may be too aggressive at high speed!**

**Try gain scheduling:**
```cpp
// Low speed (< 10 rad/s): High gains for stiff control
const float Kp_low = 0.5f;
const float Ki_low = 2.0f;

// High speed (> 20 rad/s): Lower gains for stability
const float Kp_high = 0.1f;
const float Ki_high = 0.5f;

// Interpolate based on speed
float speed_ratio = fminf(measured_mechanical_rad_per_sec / 20.0f, 1.0f);
float Kp_velocity = Kp_low + speed_ratio * (Kp_high - Kp_low);
float Ki_velocity = Ki_low + speed_ratio * (Ki_high - Ki_low);
```

---

### Fix 4: Add Velocity-Dependent Current Limit

**Problem:** At high speed, may be commanding too much current

**Current code (line ~2619):**
```cpp
measured_current_q = fmaxf(fminf(measured_current_q, MAX_FEEDFORWARD_CURRENT), MIN_FEEDFORWARD_CURRENT);
```

**Improvement:** Reduce current limit at high speed
```cpp
// At high speed, reduce current to avoid saturation
float speed_factor = fminf(measured_mechanical_rad_per_sec / 30.0f, 1.0f);
float current_limit_adjusted = m_current_limit * (1.0f - 0.5f * speed_factor);
// At 0 rad/s: full current limit
// At 30 rad/s: 50% current limit

measured_current_q = fmaxf(fminf(measured_current_q, current_limit_adjusted), MIN_FEEDFORWARD_CURRENT);
```

---

## Expected Results After Tuning

### Before (Current State):
- ❌ Motor starts fast but stops at ~20-30 rad/s
- ❌ Rough operation before stopping
- ❌ Voltage saturates or oscillates

### After (Tuned):
- ✅ Motor reaches commanded speed smoothly
- ✅ Stable operation at high speeds (30+ rad/s)
- ✅ Low current draw (efficient)
- ✅ No oscillations or roughness

---

## Quick Test Matrix

| Flux Linkage (Wb) | Expected Behavior | Action |
|-------------------|-------------------|--------|
| 0.002 | Weak back-EMF comp, may overshoot | Too low |
| 0.005 | Current setting | Baseline |
| 0.010 | Stronger comp, better high-speed | Try this! |
| 0.015 | Very strong comp, may limit speed | If 0.010 not enough |
| 0.020 | Maximum reasonable | Upper bound |

**Start with 0.010 Wb and test!**

---

## Advanced: Field Weakening (Future)

If motor still can't reach desired speed even with tuning:

**Field weakening injects negative D-axis current:**
```cpp
// At high speed, inject negative Id to weaken flux
if (measured_mechanical_rad_per_sec > 25.0f)
{
    float field_weakening_current = -0.5f * (measured_mechanical_rad_per_sec - 25.0f);
    m_voltage.d = field_weakening_current * PHASE_RESISTANCE;
}
```

**This reduces back-EMF, allowing higher speed, but:**
- Reduces torque capability
- More complex to tune
- Not needed if flux linkage constant is correct

---

## Summary

**Most Likely Issue:** `PERM_MAGNET_FLUX_LINKAGE = 0.005` is too low

**Quick Fix:** Try `PERM_MAGNET_FLUX_LINKAGE = 0.010f` or `0.015f`

**Proper Fix:** Measure actual back-EMF constant using one of the methods above

**Expected Outcome:** Motor runs smoothly to high speeds without stopping

**Time Estimate:**
- Quick test with different constants: 10 minutes
- Proper back-EMF measurement: 30 minutes
- Full tuning with gain scheduling: 1 hour

Start by doubling the flux linkage constant and see if motor behavior improves! 🎯