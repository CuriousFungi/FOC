# PWM Shoot-Through Diagnosis and Fix

## Date: 2025
## Issue: H-Bridge Shoot-Through Preventing Motor Rotation

---

## Problem Summary

**Symptom:** Motor makes noise but doesn't spin. Oscilloscope shows D4 (high-side) and D5 (low-side) are both active simultaneously, causing shoot-through that shorts the power supply.

**Evidence:**
- D5 stays HIGH when D4 is HIGH (should be complementary)
- PWM duty cycles going negative (incorrect)
- One phase stuck at 100%, others varying
- Motor draws current but produces no torque

---

## Root Cause Analysis

### What We Know Works ✅

1. **Kalman Filter Velocity Estimation** - Working perfectly
2. **Current Sensing** - ADC reading ~1.5A sinusoidal waveforms correctly
3. **Inverse Park Transform** - U_alpha and U_beta calculated correctly
4. **Duty Cycle Logic** - Code explicitly sets one channel to 0 when other is active

### The Problem ⚠️

The issue is **NOT** in the FOC algorithm or duty cycle calculation. The problem is in how **bi-directional H-bridge control** is being implemented.

---

## Understanding H-Bridge Control

### H-Bridge Architecture

For a single motor winding (e.g., Phase A):

```
        VCC
         |
     [Q1]---[Q2]
         |   |
    1A --+   +-- 1B  (Motor winding connected between 1A and 1B)
         |   |
     [Q3]---[Q4]
         |
        GND
```

### Two Control Strategies

#### **Strategy 1: Sign/Magnitude (Current Implementation - WRONG for this hardware)**

- One PWM signal controls magnitude (duty cycle)
- One GPIO signal controls direction (high/low)
- **Problem:** Your code sends PWM to BOTH 1A and 1B based on voltage sign
- **Result:** When U_alpha changes sign, both pins can be active → shoot-through

#### **Strategy 2: Locked Anti-Phase (CORRECT for your hardware)**

- Both pins get PWM signals that are **complementary**
- Pin 1A: duty cycle D
- Pin 1B: duty cycle (100% - D)
- Center point (50% duty) = zero voltage across winding
- >50% duty = positive voltage
- <50% duty = negative voltage

---

## Current Code Analysis

### File: `StepperDriver.hpp` - Function `set_pwm_duty_cycle()`

**Current Logic (Lines 285-310):**
```cpp
// When U_alpha >= 0:
duty_cycle_1B = duty_cycle_alpha * hifactor_1;
duty_cycle_1A = 0.0f;  // Set to zero

// When U_alpha < 0:
duty_cycle_1A = duty_cycle_alpha * lofactor_1;
duty_cycle_1B = 0.0f;  // Set to zero
```

**Problem:** This assumes you can set one side to 0% and the other to X%. But this causes shoot-through when transitioning between positive and negative voltages.

---

## The Fix: Locked Anti-Phase PWM

### Concept

Instead of:
- U_alpha > 0: 1B = 60%, 1A = 0%  ❌
- U_alpha < 0: 1B = 0%, 1A = 60%  ❌

Do this:
- U_alpha > 0: 1B = 80%, 1A = 20%  ✅ (80-20=60% effective)
- U_alpha < 0: 1B = 20%, 1A = 80%  ✅ (20-80=-60% effective)
- U_alpha = 0: 1B = 50%, 1A = 50%  ✅ (50-50=0% effective)

### Mathematical Formulation

```
base_duty = 0.5  // Center point (50%)

// For Phase 1 (U_alpha):
duty_cycle_1A = base_duty - (U_alpha / V_supply) * 0.5
duty_cycle_1B = base_duty + (U_alpha / V_supply) * 0.5

// For Phase 2 (U_beta):
duty_cycle_2A = base_duty - (U_beta / V_supply) * 0.5
duty_cycle_2B = base_duty + (U_beta / V_supply) * 0.5
```

**Key Points:**
- Both channels are ALWAYS active
- They are complementary around 50%
- Net voltage = (1B - 1A) * V_supply
- No shoot-through because we never have 100% + 0%

---

## Implementation Steps

### Step 1: Modify `set_pwm_duty_cycle()` in `StepperDriver.hpp`

Replace lines 259-365 with:

```cpp
void set_pwm_duty_cycle(float U_alpha, 
                        float U_beta,  
                        float hifactor_1, 
                        float lofactor_1, 
                        float hifactor_2, 
                        float lofactor_2)
{        
    if(!m_initialized)
    {
        return;
    }

    // Clamp voltages to configured limits
    U_alpha = symetric_clamp(U_alpha, m_voltage_limit);
    U_beta  = symetric_clamp(U_beta,  m_voltage_limit);

    // Calculate normalized voltage (-1 to +1)
    float alpha_normalized = U_alpha / m_power_supply_voltage;
    float beta_normalized  = U_beta  / m_power_supply_voltage;

    // Apply duty cycle limiting
    alpha_normalized = symetric_clamp(alpha_normalized, m_duty_cycle_limit.result(1.0f));
    beta_normalized  = symetric_clamp(beta_normalized,  m_duty_cycle_limit.result(1.0f));

    // LOCKED ANTI-PHASE PWM
    // Base duty = 50%, then add/subtract based on desired voltage
    // This ensures both channels are always active and complementary
    const float BASE_DUTY = 0.5f;

    // Phase 1 (U_alpha) - channels 1A and 1B
    float duty_cycle_1A = BASE_DUTY - (alpha_normalized * 0.5f * lofactor_1);
    float duty_cycle_1B = BASE_DUTY + (alpha_normalized * 0.5f * hifactor_1);

    // Phase 2 (U_beta) - channels 2A and 2B
    float duty_cycle_2A = BASE_DUTY - (beta_normalized * 0.5f * lofactor_2);
    float duty_cycle_2B = BASE_DUTY + (beta_normalized * 0.5f * hifactor_2);

    // Safety clamp to [0, 1] range
    duty_cycle_1A = constrain(duty_cycle_1A, 0.0f, 1.0f);
    duty_cycle_1B = constrain(duty_cycle_1B, 0.0f, 1.0f);
    duty_cycle_2A = constrain(duty_cycle_2A, 0.0f, 1.0f);
    duty_cycle_2B = constrain(duty_cycle_2B, 0.0f, 1.0f);

    // Update global debug variables
    g_dutycycle_1A = duty_cycle_1A;
    g_dutycycle_1B = duty_cycle_1B;
    g_dutycycle_2A = duty_cycle_2A;
    g_dutycycle_2b = duty_cycle_2B;

    // Write to hardware
    set_dutycycles(duty_cycle_1A, duty_cycle_1B, duty_cycle_2A, duty_cycle_2B);
    
    return;
}
```

### Step 2: Add `constrain()` Helper Function

Add this to `StepperDriver.hpp` if not already present:

```cpp
inline float constrain(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
```

---

## Expected Results After Fix

### Oscilloscope Measurements - Steady State Example

**Test Condition:** U_alpha = +10V, V_supply = 20V, PWM = 20kHz (50μs period)

### Before (Sign/Magnitude):
```
Pin 1A: Always LOW (0% duty) - no switching
Pin 1B: Switching at 50% duty (25μs HIGH, 25μs LOW)

Problem: Only one pin switches at a time. When voltage polarity 
reverses, must switch which pin is active → shoot-through risk.
```

### After (Locked Anti-Phase):
```
Pin 1A: Switching at 25% duty (12.5μs HIGH per 50μs period)
Pin 1B: Switching at 75% duty (37.5μs HIGH per 50μs period)

When measured on scope:
- 1A and 1B are never both HIGH at the same time
- 1A HIGH period occurs when 1B is LOW
- 1B HIGH period occurs when 1A is LOW
- Duty sum: 25% + 75% = 100%
- Net voltage: (75% - 25%) × 20V = +10V ✓
```

**Key Verification:**
- Both pins show PWM activity (not stuck at 0% or 100%)
- Duty cycles are complementary (sum to ~100%)
- No simultaneous HIGH states (shoot-through)

---

## Verification Checklist

After implementing the fix:

- [ ] Monitor `g_dutycycle_1A` and `g_dutycycle_1B` - should sum to ~100%
- [ ] Monitor `g_dutycycle_2A` and `g_dutycycle_2b` - should sum to ~100%
- [ ] Check oscilloscope: D4 and D5 should be complementary PWM
- [ ] Verify no shoot-through current spikes
- [ ] Motor should produce smooth rotation
- [ ] Current waveforms should remain sinusoidal (~1.5A)
- [ ] Velocity control loop should track setpoint

---

## Alternative Solution (If Above Doesn't Work)

If your H-bridge driver has **built-in complementary logic**, you may only need to drive ONE pin per phase:

```cpp
// Only drive one pin per phase, let driver generate complement
if (U_alpha >= 0) {
    duty_cycle_1A = fabs(U_alpha) / m_power_supply_voltage;
    // Don't write to 1B at all, or write inverted signal
} else {
    duty_cycle_1A = 1.0f - (fabs(U_alpha) / m_power_supply_voltage);
}
```

Check your H-bridge driver datasheet (L298N) to see if it has complementary input mode.

---

## Debug Variables to Monitor

Add these to your watch window:

```cpp
// Input voltages
g_U_alpha_debug
g_U_beta_debug

// Calculated duty cycles (should be ~40-60% range, complementary)
g_dutycycle_1A
g_dutycycle_1B
g_dutycycle_2A
g_dutycycle_2b

// Sum checks (should equal ~1.0)
g_dutycycle_1A + g_dutycycle_1B
g_dutycycle_2A + g_dutycycle_2b

// Control voltages
g_voltage_q
g_voltage_d  // Should be 0

// Velocity
g_kalman_velocity
```

---

## Theory: Why This Happens

### FOC Generates Bipolar Voltages

The inverse Park transform outputs:
- `U_alpha = -sin(θ) * Uq`
- `U_beta = cos(θ) * Uq`

These are **sinusoidal and bipolar** (swing positive and negative).

### H-Bridge Needs Special Care

A single MOSFET can't conduct in reverse. To get bipolar voltage across a winding, you need:

1. **Full H-bridge** with 4 FETs
2. **Locked anti-phase control** to avoid shoot-through
3. **Dead-time insertion** (optional, hardware handles this if using complementary timer outputs)

Your STM32 timer has dead-time configured (200 ticks), but you're not using the complementary timer outputs (CH1N, CH2N). You're using two independent channels (CH1, CH2), so you must manually create complementary PWM in software.

---

## Next Steps After Motor Spins

Once shoot-through is fixed and motor spins:

1. **Tune velocity controller gains** (Kp, Ki, Kd)
2. **Verify current limiting** works correctly
3. **Test acceleration ramps** (0.1 rad/s²)
4. **Implement field weakening** if needed (set Ud ≠ 0)
5. **Add position control loop** on top of velocity loop

---

## References

- **SimpleFOC Library**: Check their H-bridge driver implementation
- **STM32 Timer Cookbook**: Complementary PWM configuration
- **L298N Datasheet**: Your specific H-bridge driver requirements
- **Application Note AN4776**: STM32 motor control best practices

---

## Summary

**Root Cause:** Sign/magnitude PWM control with independent channels causes shoot-through during polarity transitions.

**Solution:** Implement locked anti-phase PWM where both H-bridge sides are driven with complementary duty cycles centered at 50%.

**Expected Outcome:** No shoot-through, smooth motor rotation, properly functioning FOC velocity control.

---

## Contact

If this fix doesn't resolve the issue, check:
1. H-bridge driver type and wiring
2. Timer dead-time configuration
3. GPIO alternate function mapping
4. Power supply current capability (should handle inrush)

Good luck! 🚀