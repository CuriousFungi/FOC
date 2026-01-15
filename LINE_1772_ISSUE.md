# Line 1772 Return Statement Issue - Critical Finding

## Problem Summary

**Line 1772 in StepperMotor.cpp:** `return;` statement after velocity control

**Observed Behavior:**
- **With `return;` ENABLED (normal code):**
  - Motor runs poorly at high speed
  - Back-EMF calculation shows 0V (should be 26V)
  - `g_debug_perm_magnet_flux` shows ~200 with huge spikes (should be constant 0.015)
  - Current oscillates ±3A
  
- **With `return;` DISABLED (line commented out):**
  - Motor runs MUCH BETTER
  - Debug signals don't work properly
  - Falls through to position control code

---

## Root Cause Analysis

### What Happens With `return;` ENABLED (Current Code)

```cpp
void StepperMotor::control_loop_25us()
{
    if (m_motion_control == MOTION_CONTROL_TYPE::CL_VELOCITY ||
        m_motion_control == MOTION_CONTROL_TYPE::OL_VELOCITY)
    {
        static uint8_t decimation_counter = 0;
        decimation_counter++;

        if (decimation_counter >= 4)  // 40kHz / 4 = 10kHz
        {
            decimation_counter = 0;
            winding_currents result = update_amperage();
            loopFOC(result.winding_amperage_a, result.winding_amperage_b);
            update_ramp();
        }
        return;  // ← LINE 1772: Exit here for velocity control
    }
    
    // Position control code below (never reached in velocity mode)
    // ...
}
```

**Control flow:**
1. Runs at 40kHz via timer interrupt
2. Decimates to 10kHz (only every 4th call)
3. Calls `loopFOC()` which calls `update_speed_closed_loop()` 
4. Returns (exits function)
5. **setPhaseVoltage() was already called inside update_speed_closed_loop()**

---

### What Happens With `return;` DISABLED (Your Test)

```cpp
void StepperMotor::control_loop_25us()
{
    if (m_motion_control == MOTION_CONTROL_TYPE::CL_VELOCITY ||
        m_motion_control == MOTION_CONTROL_TYPE::OL_VELOCITY)
    {
        // ... velocity control code ...
        // return;  // ← DISABLED!
    }
    
    // Falls through to position control code!
    float curr_mech_rad = read_angle_radians_from_buffer_with_offset();
    float curr_elec_rad = mechanical_to_electrical_radians(curr_mech_rad);
    float delta_mech_rad = m_commanded_speed_radians_per_sec * elapsed_time.get() * 0.000001f;
    float delta_elec_rad = mechanical_to_electrical_radians(delta_mech_rad);
    float cmd_elec_rad = curr_elec_rad + delta_elec_rad;
    
    setPhaseVoltage(m_voltage.q, m_voltage.d, cmd_elec_rad);  // ← CALLED AGAIN!
}
```

**Control flow:**
1. Velocity control executes (at 10kHz decimated)
2. Does NOT return
3. **Falls through to position control code** (runs at 40kHz!)
4. Position control calls `setPhaseVoltage()` with different angle
5. **Two different control algorithms fighting each other!**

---

## Why Motor Runs Better Without `return;`

**Paradox:** Two competing controllers should make things WORSE, not better!

**Hypothesis:** The position control path provides a **working angle-based commutation** that bypasses the broken back-EMF calculation in velocity control.

**Position control (line 1789-1808):**
```cpp
float curr_mech_rad = read_angle_radians_from_buffer_with_offset();
float curr_elec_rad = mechanical_to_electrical_radians(curr_mech_rad);
// ... calculate delta based on commanded speed ...
setPhaseVoltage(m_voltage.q, m_voltage.d, cmd_elec_rad);
```

This uses:
- **Current angle from encoder** (not Kalman velocity)
- **Direct angle calculation** (not back-EMF based)
- **Runs at 40kHz** (4× faster than velocity control)

**The position control accidentally provides correct angle-based commutation!**

---

## Why Debug Signals Corrupted

### The `g_debug_perm_magnet_flux` Corruption (~200 value)

**Expected:** `g_debug_perm_magnet_flux = 0.015` (constant)

**Observed:** ~200 with huge spikes (±100)

**Cause:** Variable corruption due to:

1. **Timing conflict:** 
   - Velocity control updates debug vars at 10kHz
   - Position control runs at 40kHz
   - Variables being written from two contexts

2. **Memory corruption:**
   - Stack overflow
   - Buffer overrun
   - Two code paths writing to overlapping memory

3. **Floating point precision issues:**
   - Division by small number
   - Uninitialized variable being read

4. **Compiler optimization issues:**
   - Variables optimized away or reordered
   - Race condition between reads/writes

---

## The Real Problem: Back-EMF is Zero

**Why velocity control fails:**

From your earlier graphs, we saw:
```
g_debug_back_emf_q_axis = 0V (should be 26V at 35 rad/s)
```

**This means the velocity control feedforward is completely broken!**

Without feedforward:
- Controller has no idea what voltage is needed
- Relies entirely on slow PI feedback
- Results in oscillations and poor performance

**The position control "works" because:**
- It doesn't use back-EMF calculation
- It directly uses encoder angle for commutation
- Simple but effective (though not optimal)

---

## Recommended Solutions

### Solution 1: Fix the Back-EMF Calculation (Proper Fix)

**Find why `g_debug_back_emf_q_axis = 0`:**

Add debug variable (already added):
```cpp
g_debug_measured_mech_rad_per_sec
```

Monitor at 35 rad/s:
- Should show ~35 rad/s
- If shows 0 → velocity feedback broken
- If shows 35 → calculation logic broken

**Most likely cause:** `g_kalman_velocity` is not being read correctly

**Fix:** Ensure velocity measurement is working in `update_speed_closed_loop()`

---

### Solution 2: Hybrid Approach (Temporary Workaround)

**Use position control angle with velocity control voltage:**

```cpp
// In update_speed_closed_loop(), replace angle calculation:

// OLD (broken - uses blending with zero back-EMF):
// float electrical_angle_radians = ... blending logic ...

// NEW (working - use encoder directly):
float electrical_angle_radians = g_cached_encoder_angle;

// Keep the voltage calculation (PI controller, etc)
// Just use encoder angle instead of blended angle
```

This gives:
- ✅ Working angle (from encoder)
- ✅ Working voltage control (PI with correct feedforward once back-EMF fixed)
- ✅ Proper velocity control behavior

---

### Solution 3: Investigate Variable Corruption

**The ~200 value for `g_debug_perm_magnet_flux` suggests deeper issues:**

Possible causes:
1. **Stack overflow** - check stack size
2. **Buffer overrun** - check array bounds
3. **Uninitialized variable** - ensure all paths initialize
4. **Pointer corruption** - check for wild pointers
5. **Interrupt race condition** - protect critical sections

**Diagnostic:**
```cpp
// Add at start of update_speed_closed_loop():
if (PERM_MAGNET_FLUX_LINKAGE < 0.001f || PERM_MAGNET_FLUX_LINKAGE > 0.1f)
{
    // Corruption detected!
    g_debug_corruption_detected = 1.0f;
    // Trigger breakpoint or logging
}
```

---

## Immediate Actions

### Test 1: Verify Velocity Measurement

**Add to monitor:**
```
g_debug_measured_mech_rad_per_sec  → Should be ~35 at 35 rad/s
g_kalman_velocity                  → Should match above
g_debug_measured_elec_rad_per_sec  → Should be ~1750 (mech × 50)
```

**If velocity is 0:**
- Back-EMF calculation fails (0 × anything = 0)
- This explains why velocity control doesn't work
- Position control works because it doesn't use velocity

---

### Test 2: Check for Stack Overflow

**In your linker script, increase stack size:**
```
_Min_Stack_Size = 0x1000; /* Increase from default */
```

**Monitor:**
```
__get_MSP()  // Main stack pointer
```

If stack pointer is near limit, overflow is corrupting variables.

---

### Test 3: Try Hybrid Solution

**Temporarily use encoder angle in velocity control:**

In `update_speed_closed_loop()`, around line ~2880:

```cpp
// TEMPORARY: Use encoder angle directly instead of blending
extern volatile float g_cached_encoder_angle;
float electrical_angle_radians = g_cached_encoder_angle;

// Comment out the blending section
// float electrical_angle_radians = (1.0f - blend_factor) * open_loop_electrical 
//                                + blend_factor * closed_loop_electrical;
```

This should give:
- ✅ Working velocity control (with encoder angle)
- ✅ Keep return; statement enabled
- ✅ No fallthrough to position control

---

## Summary

**Line 1772 Issue is a Symptom, Not the Cause:**

**Real Problem:** Back-EMF calculation returns 0V
→ Velocity control feedforward broken
→ Motor runs poorly

**Why Disabling Return "Works":**
Position control provides working angle-based commutation
→ Bypasses broken back-EMF calculation  
→ Motor runs better (but debug vars corrupted)

**Proper Solution:**
Fix velocity measurement so back-EMF calculation works
→ Velocity control will work correctly
→ Keep return; statement enabled
→ Don't need position control fallthrough

**Next Step:**
Monitor `g_debug_measured_mech_rad_per_sec` to find why back-EMF is zero! 🎯

---

## Expected Values Summary

| Variable | Expected at 35 rad/s | Your Value | Status |
|----------|---------------------|------------|--------|
| `g_debug_measured_mech_rad_per_sec` | 35 | ??? | Unknown |
| `g_debug_measured_elec_rad_per_sec` | 1750 | ??? | Unknown |
| `g_debug_back_emf_q_axis` | 26V | 0V | ❌ BROKEN |
| `g_debug_perm_magnet_flux` | 0.015 | 200 | ❌ CORRUPTED |
| `g_debug_amperage_q_for_bemf` | 1-2A | ±3A | ❌ OSCILLATING |

Fix the velocity measurement and all of these should fall into place!