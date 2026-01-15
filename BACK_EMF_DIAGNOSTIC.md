# Back-EMF and Inductance Diagnostic Guide

## Problem Statement

After correcting the inductance from 3.2 H to 0.0032 H, **no effect was observed** on motor behavior. This suggests either:
1. The inductance term is negligible compared to other terms
2. The back-EMF calculation isn't being used properly
3. Something else is limiting motor performance
4. The code path isn't actually executing

---

## Critical Debug Variables Added

New variables have been added to diagnose the back-EMF calculation:

```cpp
g_debug_inductance_value           // Inductance value used (should be ~0.0032-0.0039 H)
g_debug_mag_flux_linkage_q         // L × I_q (should be small, ~0.001-0.01 Wb)
g_debug_back_emf_q_axis            // Total back-EMF calculated (V)
g_debug_perm_magnet_flux           // Permanent magnet flux constant (Wb)
g_debug_measured_elec_rad_per_sec  // Electrical speed (rad/s)
g_debug_amperage_q_for_bemf        // Q-axis current used in calculation (A)
```

**Add these to STM32CubeMonitor immediately!**

---

## Back-EMF Calculation Breakdown

### Formula (line 2609-2611):

```cpp
mag_flux_linkage_q = inductance × I_q
back_emf = ω_electrical × (mag_flux_linkage_q + PERM_MAGNET_FLUX_LINKAGE)
```

### Two Components:

1. **Inductance term:** `ω × L × I_q`
2. **Permanent magnet term:** `ω × Ψ_magnet`

**Total:** `back_emf = ω × (L×I_q + Ψ_magnet)`

---

## Expected Values at 30 rad/s

### With Correct Inductance (0.0032 H):

**Electrical speed:**
```
ω_elec = 30 rad/s × 50 pole_pairs = 1500 electrical rad/s
```

**Inductance term (depends on I_q):**
```
If I_q = 0.1 A:  L×I_q = 0.0032 × 0.1 = 0.00032 Wb
If I_q = 1.0 A:  L×I_q = 0.0032 × 1.0 = 0.0032 Wb
If I_q = 2.0 A:  L×I_q = 0.0032 × 2.0 = 0.0064 Wb
```

**Permanent magnet term:**
```
Ψ_magnet = 0.015 Wb (current setting after you increased it)
```

**Total flux linkage:**
```
At I_q = 0.1 A:  Total = 0.00032 + 0.015 = 0.01532 Wb
At I_q = 1.0 A:  Total = 0.0032 + 0.015 = 0.0182 Wb
At I_q = 2.0 A:  Total = 0.0064 + 0.015 = 0.0214 Wb
```

**Back-EMF at 30 rad/s:**
```
At I_q = 0.1 A:  V_bemf = 1500 × 0.01532 = 23.0 V
At I_q = 1.0 A:  V_bemf = 1500 × 0.0182 = 27.3 V
At I_q = 2.0 A:  V_bemf = 1500 × 0.0214 = 32.1 V
```

**Key insight:** The inductance term contributes only 2-5V even at 2A!

---

## Why Inductance Fix Had No Effect

### Hypothesis 1: Current is Very Low (Most Likely)

If `m_amperage.q` is only ~0.1-0.3A during operation:

**With WRONG inductance (3.2 H):**
```
L×I_q = 3.2 × 0.1 = 0.32 Wb
back_emf = 1500 × (0.32 + 0.015) = 502.5 V
```

**With CORRECT inductance (0.0032 H):**
```
L×I_q = 0.0032 × 0.1 = 0.00032 Wb
back_emf = 1500 × (0.00032 + 0.015) = 23.0 V
```

**But if permanent magnet flux is dominant:**
```
Permanent magnet term: 1500 × 0.015 = 22.5 V
Inductance term (wrong): 1500 × 0.32 = 480 V
Inductance term (correct): 1500 × 0.00032 = 0.48 V

Change in back-EMF = 480 - 0.48 = 479.5 V
But if I_q is very small, this might not matter!
```

**Check:** Monitor `g_debug_amperage_q_for_bemf`
- If < 0.5A → Inductance term is negligible
- If > 1.5A → Inductance term should be significant

---

### Hypothesis 2: Permanent Magnet Flux Dominates

If `PERM_MAGNET_FLUX_LINKAGE = 0.015 Wb` is much larger than `L×I_q`:

**At I_q = 0.5A:**
```
L×I_q (wrong) = 3.2 × 0.5 = 1.6 Wb   → 53% of total (3.2 Wb)
L×I_q (correct) = 0.0032 × 0.5 = 0.0016 Wb → 10% of total (0.0166 Wb)

Total (wrong) = 1.6 + 0.015 = 1.615 Wb
Total (correct) = 0.0016 + 0.015 = 0.0166 Wb

Back-EMF (wrong) = 1500 × 1.615 = 2422 V
Back-EMF (correct) = 1500 × 0.0166 = 24.9 V

Huge difference! Should have seen effect!
```

**Unless:** Current is so low that inductance term is always negligible

---

### Hypothesis 3: Voltage Already Saturated

If motor is **already hitting the 20V voltage limit** before the fix:

```
Before fix: desired_voltage = 502V, actual = 20V (saturated)
After fix: desired_voltage = 25V, actual = 20V (still saturated!)

Motor sees same 20V either way → no change in behavior!
```

**Check:** 
```
g_desired_voltage_q_before_clamp  → What controller wants
g_m_voltage_q_in_controller       → What controller gets (max 20V)

If both are saturated (> 20V) → voltage limit is the real problem
```

---

### Hypothesis 4: Something Else is Stopping Motor

The motor might be stopping due to:
- Loss of synchronization (angle calculation issue)
- Current limiting
- Torque insufficient for load
- PI controller instability
- Electrical angle error

**Not** due to voltage/back-EMF!

---

## Diagnostic Procedure

### Step 1: Monitor Back-EMF Components (2 minutes)

Add these variables to STM32CubeMonitor:

**Primary diagnostics:**
```
g_debug_back_emf_q_axis           → Total back-EMF (V)
g_debug_amperage_q_for_bemf       → Current used (A)
g_debug_perm_magnet_flux          → Permanent magnet flux (Wb)
g_debug_mag_flux_linkage_q        → L×I term (Wb)
g_debug_inductance_value          → Inductance used (H)
```

**Voltage diagnostics:**
```
g_base_voltage_q_feedforward      → Feedforward voltage
g_feedback_voltage_correction     → Feedback correction
g_desired_voltage_q_before_clamp  → Total desired
g_m_voltage_q_in_controller       → Actual applied (≤20V)
```

**Speed diagnostics:**
```
g_kalman_velocity                 → Actual speed (rad/s)
g_target_rps_to_cl_controller     → Target speed (rad/s)
g_velocity_error_for_debug        → Speed error (rad/s)
```

---

### Step 2: Run Motor to Failure Point (1 minute)

1. Start motor at low speed (5 rad/s)
2. Gradually increase to point where it stops
3. Record values at failure

**Record these values at motor stop:**
```
Speed: ____________ rad/s
g_debug_back_emf_q_axis: ____________ V
g_debug_amperage_q_for_bemf: ____________ A
g_debug_mag_flux_linkage_q: ____________ Wb
g_debug_perm_magnet_flux: ____________ Wb
g_desired_voltage_q_before_clamp: ____________ V
g_m_voltage_q_in_controller: ____________ V
```

---

### Step 3: Analyze Results

#### Scenario A: Low Current (I_q < 0.5A)

```
g_debug_amperage_q_for_bemf = 0.2 A
g_debug_mag_flux_linkage_q = 0.0006 Wb
g_debug_perm_magnet_flux = 0.015 Wb

Inductance contribution: 0.0006 / (0.0006 + 0.015) = 4%
Permanent magnet dominates: 96%
```

**Conclusion:** Inductance term is negligible → fix has no effect
**Real problem:** Permanent magnet flux might be wrong, OR voltage saturation

---

#### Scenario B: Voltage Saturation

```
g_desired_voltage_q_before_clamp = 28V
g_m_voltage_q_in_controller = 20V
Difference = 8V (40% saturation!)
```

**Conclusion:** Hitting voltage limit
**Solution:** 
- Increase voltage limit (if supply allows)
- Reduce speed target
- Verify permanent magnet flux constant

---

#### Scenario C: High Back-EMF Estimate

```
g_debug_back_emf_q_axis = 500V
g_debug_perm_magnet_flux = 0.015 Wb
At 30 rad/s: 500V / 1500 rad/s = 0.33 Wb total flux

This is HUGE! Much larger than expected.
```

**Conclusion:** Permanent magnet flux constant is WAY too high
**Solution:** Reduce `PERM_MAGNET_FLUX_LINKAGE` back to 0.005 or lower

---

#### Scenario D: Reasonable Values, Still Stops

```
g_debug_back_emf_q_axis = 25V (reasonable)
g_desired_voltage_q_before_clamp = 18V (below limit)
g_m_voltage_q_in_controller = 18V (no saturation)
Motor still stops at 25 rad/s
```

**Conclusion:** Problem is NOT back-EMF related!
**Likely causes:**
- Angle calculation issue (synchronization loss)
- Current control problem
- PI controller instability
- Mechanical issue

---

## Why Inductance Term is Small

### Physical Explanation

**Inductance term:** `ω × L × I_q`

At moderate speeds and currents:
```
ω = 1500 electrical rad/s
L = 0.0032 H
I_q = 1.0 A

Inductance term = 1500 × 0.0032 × 1.0 = 4.8 V
```

**Permanent magnet term:** `ω × Ψ_magnet`
```
Ψ_magnet = 0.015 Wb

Permanent term = 1500 × 0.015 = 22.5 V
```

**Ratio:** Inductance contributes only 4.8 / 27.3 = **17.6%** of total back-EMF

**This is why fixing inductance might not have visible effect!**

The permanent magnet flux dominates, and that's controlled by `PERM_MAGNET_FLUX_LINKAGE`, not phase inductance.

---

## Action Plan

### Priority 1: Check Current Level

**Monitor:** `g_debug_amperage_q_for_bemf`

**If < 0.5A:** Inductance term is negligible, no wonder fix had no effect
**If > 1.5A:** Should see some effect, check voltage saturation

---

### Priority 2: Check Voltage Saturation

**Monitor:** `g_desired_voltage_q_before_clamp` vs `g_m_voltage_q_in_controller`

**If desired >> actual:** Hitting voltage limit
**Solution:** 
- Increase voltage_limit to 24V (if hardware supports)
- OR reduce PERM_MAGNET_FLUX_LINKAGE

---

### Priority 3: Verify Permanent Magnet Flux

**Monitor:** `g_debug_back_emf_q_axis` and `g_debug_perm_magnet_flux`

**Calculate back-EMF constant:**
```
K_e = back_emf / (ω_electrical)
K_e = g_debug_back_emf_q_axis / (g_kalman_velocity × 50)

Example at 30 rad/s:
If back_emf = 25V:
K_e = 25 / (30 × 50) = 25 / 1500 = 0.0167 Wb

This should match PERM_MAGNET_FLUX_LINKAGE (approximately)
```

**If K_e >> 0.015:** Permanent magnet flux is overestimated
**If K_e << 0.015:** Permanent magnet flux is underestimated

---

## Most Likely Scenario

Based on your observation that inductance fix had **no effect:**

**Hypothesis:** Motor current is LOW (< 0.5A) at the speed where it stops

This makes the inductance term:
```
L × I_q = 0.0032 × 0.3 = 0.001 Wb

Compared to permanent magnet flux (0.015 Wb), this is only 6%
```

**The real issue is probably:**
1. **Voltage saturation** - hitting 20V limit
2. **Permanent magnet flux wrong** - PERM_MAGNET_FLUX_LINKAGE needs tuning
3. **Something else entirely** - synchronization, PI gains, etc.

---

## Summary

**The inductance fix may be correct but irrelevant** because:
- Inductance term is small compared to permanent magnet term
- Current is low during operation
- Permanent magnet flux dominates back-EMF calculation

**Next steps:**
1. Monitor the new debug variables
2. Check if voltage is saturating
3. Verify permanent magnet flux constant is correct
4. Look at other potential issues (synchronization, PI tuning)

**Report back with values of the diagnostic variables and we'll identify the real bottleneck!** 🎯