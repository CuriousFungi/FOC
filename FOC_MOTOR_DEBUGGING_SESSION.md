# FOC Motor Control Debugging Session

**Date:** 2024
**System:** STM32H755 Dual-Core with Field-Oriented Control (FOC)
**Motor:** Stepper motor, 50 pole pairs, 1.4Ω resistance, 3.2mH inductance
**Power Supply:** 20V
**Encoder:** AS5048A magnetic encoder with Kalman filter

---

## Table of Contents

1. [Initial Problem: Linker Error](#initial-problem-linker-error)
2. [Current Sensing Calibration](#current-sensing-calibration)
3. [DC Resistance vs AC Impedance](#dc-resistance-vs-ac-impedance)
4. [PI Velocity Controller Implementation](#pi-velocity-controller-implementation)
5. [Velocity Measurement Issues](#velocity-measurement-issues)
6. [Motor Heating Problem](#motor-heating-problem)
7. [Current Status](#current-status)
8. [Key Lessons Learned](#key-lessons-learned)

---

## Initial Problem: Linker Error

### Issue
```
undefined reference to `udpate_amperage'
```

### Root Cause
Function had typo in name: `udpate_amperage` instead of `update_amperage`

Additionally, there was a C/C++ linkage mismatch:
- **Declaration** in `StepperMotor.cpp`: `extern "C" winding_currents udpate_amperage(void);`
- **Definition** in `main_cpp.cpp`: `winding_currents udpate_amperage(void)` (missing `extern "C"`)

### Solution
1. Renamed `udpate_amperage` → `update_amperage` (9 occurrences)
2. Added `extern "C"` to function definition

```cpp
// Fixed in main_cpp.cpp
extern "C" winding_currents update_amperage(void)
{
    // ... current sensing code
}
```

---

## Current Sensing Calibration

### Problem
Current readings showed large DC offsets:
- `g_winding_amps_a` = 8A ± 3A (should be 0A ± current)
- `g_winding_amps_b` = 6A ± 2A
- `g_amperage_q` = ~10A DC offset
- `g_amperage_d` = ~-10A DC offset

### Root Cause
ACS712 current sensor zero-point voltage was not calibrated correctly:
- Code assumed: 2.5V at zero current
- Actual measured: ~4.0V at zero current
- Error: 1.5V → 1.5V / 0.185 V/A = **8A DC offset**

### Solution: Auto-Calibration at Startup

Implemented automatic zero-point calibration:

```cpp
void calibrate_current_sensor_zero_point_polled(void)
{
    const uint32_t NUM_SAMPLES = 500;
    
    float sum_a = 0.0f;
    float sum_b = 0.0f;
    
    for (uint32_t i = 0; i < NUM_SAMPLES; i++)
    {
        uint16_t raw_a = 0;
        uint16_t raw_b = 0;
        read_adc_channels_polled(&raw_a, &raw_b);
        
        float voltage_a = (raw_a / 65535.0f) * 5.0f;
        float voltage_b = (raw_b / 65535.0f) * 5.0f;
        
        sum_a += voltage_a;
        sum_b += voltage_b;
        
        HAL_Delay(2);  // 2ms between samples
    }
    
    g_calibrated_zero_a = sum_a / NUM_SAMPLES;
    g_calibrated_zero_b = sum_b / NUM_SAMPLES;
    
    // Validate results (1.0V to 4.5V expected)
    if (g_calibrated_zero_a < 1.0f || g_calibrated_zero_a > 4.5f ||
        g_calibrated_zero_b < 1.0f || g_calibrated_zero_b > 4.5f)
    {
        g_calibration_complete = 2;  // Failed
    }
    else
    {
        g_calibration_complete = 1;  // Success
    }
}
```

**Calibration runs at startup:**
- PWM is at 0% (H-bridge disabled)
- Motor is not enabled yet
- No current flowing through windings
- Takes 1 second (500 samples × 2ms)
- Fully automatic, no manual intervention required

**Results after calibration:**
- `g_winding_amps_a` = 0A ± 4A (centered correctly!)
- `g_winding_amps_b` = 0A ± 4A
- `g_amperage_q` = ~3A (realistic torque current)
- `g_amperage_d` = ~0A (good FOC alignment)

---

## DC Resistance vs AC Impedance

### Problem
Feedforward voltage calculation was commanding 50V when only 20V was available:
```
g_base_voltage_q_feedforward = 50V (way too high!)
g_voltage_q = 20V (saturated at limit)
```

### Root Cause
LCR meter measurements were being used incorrectly:

**Measured impedances:**
- 1 kHz: 24.29Ω
- 10 kHz: 212Ω
- DC: 1.4Ω

These AC impedance values include inductive reactance:
```
Z = √(R² + (ωL)²)
```

**The code was using AC impedance for DC current calculation:**
```cpp
// WRONG - Using AC impedance (24.29Ω) for DC current
float base_voltage_q = 2.0A × resistance(target_electrical_rad_per_sec)
                     + fabs(back_emf_q_axis);
// Result: 2.0A × 24.29Ω = 48.58V ← Way too high!
```

### Solution: Use DC Resistance for Resistive Drop

In FOC, the DQ frame current is **DC** (constant in rotating reference frame):

```cpp
// CORRECT - Use DC resistance (1.4Ω) for steady-state I×R drop
float base_voltage_q = STARTUP_CURRENT_LIMIT * PHASE_RESISTANCE
                     + fabs(back_emf_q_axis);
// Result: 2.0A × 1.4Ω + back_emf = 2.8V + back_emf ← Realistic!
```

**When to use each value:**

| Component | Value to Use | Reason |
|-----------|--------------|--------|
| **Resistive drop: R×I** | DC resistance (1.4Ω) | DQ current is DC |
| **Inductive drop: L×dI/dt** | AC inductance (3.2-3.9mH) | Current changing at electrical frequency |
| **Back-EMF: ω×(L×I + λ)** | AC inductance | Flux rotating at electrical frequency |

**Results after fix:**
- `g_base_voltage_q_feedforward` = 3-5V (realistic!)
- `g_voltage_q` = 3-5V (not saturated)
- Motor control much more stable

---

## PI Velocity Controller Implementation

### Problem
Large steady-state velocity error:
- Commanded: 45 rad/s
- Actual: ~1 rad/s
- Error: 44 rad/s (not converging to zero)

### Root Cause
Controller was proportional-only (P) with no integral term:

```cpp
// OLD - P-only controller
const float Kp = 0.5f;
float feedback = Kp × error;
```

**P-only controller cannot eliminate steady-state error!**

Additionally:
- Feedforward assumed fixed 2A current (wrong for no-load operation)
- Feedback saturated at 10V (insufficient)

### Solution: PI Controller with Measured Current Feedforward

#### 1. Added Integral Term

```cpp
// PI Controller
const float Kp_velocity_mech = 1.5f;   // Proportional gain
const float Ki_velocity_mech = 0.5f;   // Integral gain

static float velocity_error_integral = 0.0f;

// Integration with anti-windup
velocity_error_integral += velocity_error_mech_rad_per_sec * 0.0001f;  // dt = 100us
velocity_error_integral = clamp(velocity_error_integral, ±30.0f);

// Calculate PI feedback
float proportional_term = Kp_velocity_mech * velocity_error_mech_rad_per_sec;
float integral_term = Ki_velocity_mech * velocity_error_integral;
float feedback_correction = proportional_term + integral_term;

// Clamp to ±18V
feedback_correction = clamp(feedback_correction, ±18.0f);
```

#### 2. Fixed Feedforward to Use Measured Current

**OLD (Wrong for no-load):**
```cpp
// Assumed motor always draws 2A
float base_voltage_q = 2.0A × 1.4Ω + back_emf;
                     = 2.8V + back_emf  ← 2.8V too high for no-load!
```

**NEW (Correct for any load):**
```cpp
// Use actual measured current
float measured_current_q = fabs(m_amperage.q);
measured_current_q = clamp(measured_current_q, 0.1A, m_current_limit);

float base_voltage_q = measured_current_q * PHASE_RESISTANCE + back_emf;
// At no-load: 0.3A × 1.4Ω + back_emf = 0.42V + back_emf ← Correct!
```

#### 3. Anti-Windup Protection

```cpp
// Back off integrator when saturated
if ((feedback_correction > MAX_FEEDBACK_CORRECTION && error > 0) ||
    (feedback_correction < -MAX_FEEDBACK_CORRECTION && error < 0))
{
    velocity_error_integral *= 0.99f;  // Reduce accumulation
}

// Reset integrator when stopped
if (!m_enabled || (fabs(error) < 0.05f && fabs(target) < 0.01f))
{
    velocity_error_integral = 0.0f;
}
```

**Expected behavior:**
- Velocity error converges to zero over ~5-10 seconds
- Integral term accumulates to compensate for friction/load
- No steady-state error

---

## Velocity Measurement Issues

### Problem
Motor getting hot even though running, with large velocity error persisting:
- `g_cmd_rps` = 50 rad/s (command)
- `g_kalman_velocity` = 45 rad/s (actual encoder reading)
- `g_measured_velocity_for_control` = 10 rad/s (what controller sees!)
- **35 rad/s measurement error → excessive voltage → circulating currents → HEAT!**

### Root Cause
Spike rejection filter was too aggressive:

```cpp
// BROKEN - Rejected valid velocity updates
const float MAX_VELOCITY_CHANGE_PER_SAMPLE = 10.0f;  // 10 rad/s limit

float velocity_change = measured_velocity - prev_velocity;
if (fabs(velocity_change) > MAX_VELOCITY_CHANGE_PER_SAMPLE)
{
    measured_velocity = prev_velocity;  // REJECT! Stays at old value forever
}
```

**Why this failed:**
- Control loop: 10 kHz (every 100μs)
- Encoder updates: 1-5 kHz (every 200-1000μs)

When encoder updates with new velocity (e.g., jumps from 0 to 45 rad/s):
- Change = 45 rad/s > 10 rad/s limit
- **REJECTED permanently!**
- Controller stuck seeing stale/wrong velocity

### Solution: Disable Spike Rejection

```cpp
// FIXED - Trust the Kalman filter (it already provides smoothing)
// No spike rejection - only clamp physically impossible values

if (fabs(measured_mechanical_rad_per_sec) > 150.0f)
{
    // Only reject truly unrealistic velocities
    measured_mechanical_rad_per_sec = clamp(measured_mechanical_rad_per_sec, ±150.0f);
}
```

**Results after fix:**
- `g_measured_velocity_for_control` now matches `g_kalman_velocity`
- Velocity error is realistic (not artificially inflated)
- Controller uses correct feedback

---

## Motor Heating Problem

### Issue
Motor getting "fairly hot" during operation, even with no mechanical load.

**With proper FOC, an unloaded motor should barely warm up!**

### Power Dissipation Analysis

**Case 1: Velocity Measurement Broken (Before Fix)**
```
Controller thinks: 10 rad/s
Actual velocity: 45 rad/s
Voltage error: Commands excessive voltage to "catch up"

Current = (20V - 13.4V back-EMF) / 1.4Ω = 4.7A
Power = I²R = 4.7² × 1.4Ω = 31 Watts wasted as heat! 🔥
```

**Case 2: Velocity Measurement Fixed (After Fix)**
```
Controller knows: 45 rad/s
Actual velocity: 45 rad/s
Voltage correct: Only what's needed

Current = (14V - 13.4V) / 1.4Ω = 0.43A
Power = I²R = 0.43² × 1.4Ω = 0.26 Watts ❄️
```

**31W vs 0.26W = 120× reduction in wasted power!**

### Remaining Issues (Last Plot Analysis)

From final plot at t=32.36s:
```
Command: 17 rad/s
Measured: 3 rad/s
Error: 14 rad/s
Feedback: 18V (saturated)
```

**Motor stuck at 3 rad/s when commanded 17 rad/s** despite:
- Velocity measurement now working correctly ✅
- Feedback saturated (controller trying hard) ⚠️

**Possible causes (needs more data):**
1. **Mechanical binding** - shaft not free to rotate
2. **Wrong pole pairs** - FOC commanding wrong electrical angles
3. **D-axis current high** - current misaligned, wasted as heat
4. **Voltage saturation** - need to verify if `g_voltage_q` hitting 20V limit
5. **Encoder direction reversed** - motor spinning wrong way

---

## Current Status

### ✅ Fixed Issues

1. **Function naming and linkage** - `update_amperage()` with `extern "C"`
2. **Current sensing calibration** - Auto-calibrates at startup
3. **DC vs AC resistance** - Using correct values in FOC calculations
4. **PI velocity controller** - Integral term eliminates steady-state error
5. **Feedforward calculation** - Uses measured current, not fixed 2A assumption
6. **Velocity measurement** - Spike rejection disabled, uses Kalman filter directly

### ⚠️ Remaining Issues

1. **Motor stuck at low speed** (3 rad/s instead of 17 rad/s)
2. **Excessive heating** (indicates wasted power)
3. **Feedback saturated** (18V limit, can't provide more torque)

### 🔍 Diagnostic Data Needed

To complete diagnosis, need to monitor:

**Voltage signals:**
```c
g_voltage_q                      // Actual applied voltage (is it saturating at 20V?)
g_desired_voltage_q_before_clamp // What controller wants to command
g_base_voltage_q_feedforward     // Feedforward component
g_back_emf_q_axis                // Back-EMF at measured velocity
```

**Current signals:**
```c
g_amperage_q                     // Q-axis current (torque-producing)
g_amperage_d                     // D-axis current (should be ~0A for good FOC!)
g_winding_amps_a                 // Phase A RMS current
g_winding_amps_b                 // Phase B RMS current
g_feedforward_current_q          // Current used in feedforward
```

**Physical verification:**
- Is motor shaft actually spinning at ~30 RPM (3 rad/s)?
- Can shaft be turned freely by hand when motor disabled?
- Verify pole pairs = 50 (or should it be 25?)

---

## Key Lessons Learned

### 1. C/C++ Linkage Matters
- `extern "C"` required for both declaration AND definition
- C++ name mangling breaks linking if mismatch
- Use consistent linkage throughout

### 2. Current Sensor Calibration is Critical
- Never assume 2.5V zero-point without measurement
- Auto-calibration at startup is best practice
- Validate calibration results (1.0-4.5V range)

### 3. DC Resistance ≠ AC Impedance
- **LCR measurements are correct** - they measure total impedance
- **But FOC needs different values for different purposes:**
  - DC resistance for I×R drop in DQ frame
  - AC inductance for back-EMF and dI/dt terms
- DQ current is DC (constant in rotating frame)
- Don't use AC impedance for DC calculations!

### 4. PI Controller Prevents Steady-State Error
- P-only controller cannot eliminate offset
- Integral term accumulates until error → 0
- Anti-windup essential to prevent integrator runaway
- Reset integrator when motor stopped

### 5. Feedforward Must Match Operating Condition
- Fixed current assumption fails for varying loads
- Use measured `I_q` for accurate feedforward
- No-load: ~0.3A, loaded: 1-2A
- Prevents large voltage errors

### 6. Be Careful with Filtering
- Encoder update rate ≠ control loop rate
- Velocity can appear to "jump" (this is normal!)
- Kalman filter already provides smoothing
- Additional filtering can break measurements
- Only reject truly impossible values (>150 rad/s)

### 7. Temperature is Best FOC Quality Indicator
- Good FOC: motor barely warms up
- Bad FOC: motor heats significantly even with no load
- Heat = wasted I²R losses from circulating currents
- Monitor temperature during testing

### 8. Verify Measurements Match Reality
- Never assume encoder reading is correct
- Cross-check: does measured velocity match visual RPM?
- Check all signals in chain: raw → filtered → control
- One wrong signal breaks entire control loop

---

## Debug Variables Reference

### Velocity Signals
```c
g_as5048_velocity              // Raw encoder velocity
g_kalman_velocity              // Kalman filter output
g_velocity_before_filter       // Before spike rejection
g_velocity_after_clamp         // After 150 rad/s limit
g_measured_velocity_for_control // Final value used by controller
g_cmd_rps                      // Velocity command
g_velocity_error_for_debug     // Error: target - measured
```

### PI Controller Internals
```c
g_velocity_correction_p        // Proportional term (V)
g_velocity_correction_i        // Integral term (V)
g_velocity_error_integral      // Accumulated error (rad)
g_feedback_voltage_correction  // Total P+I after clamping
```

### Voltage Calculation
```c
g_base_voltage_q_feedforward   // Feedforward component
g_back_emf_q_axis              // Back-EMF voltage
g_desired_voltage_q_before_clamp // Total before 20V limit
g_voltage_q                    // Final applied voltage
```

### Current Measurement
```c
g_winding_amps_a               // Phase A current (A)
g_winding_amps_b               // Phase B current (A)
g_amperage_q                   // Q-axis current (torque)
g_amperage_d                   // D-axis current (flux, should be ~0)
g_feedforward_current_q        // Current used in feedforward
```

### Calibration Status
```c
g_calibration_complete         // 0=not run, 1=success, 2=failed
g_calibrated_zero_a            // Zero-point voltage A (3.5-4.5V)
g_calibrated_zero_b            // Zero-point voltage B (3.5-4.5V)
g_calibration_min_a            // Min voltage during cal
g_calibration_max_a            // Max voltage during cal
```

---

## Code Changes Summary

### Files Modified

1. **`project3/CM7/Core/Src/main_cpp.cpp`**
   - Renamed `udpate_amperage()` → `update_amperage()`
   - Added `extern "C"` to function definition
   - Added `calibrate_current_sensor_zero_point_polled()`
   - Added calibration debug variables
   - Modified `update_ramp()` to limit velocity to 45 rad/s

2. **`project3/CM7/Core/Src/motors/StepperMotor.cpp`**
   - Renamed `udpate_amperage()` → `update_amperage()` in declaration
   - Fixed DC resistance usage (PHASE_RESISTANCE instead of resistance())
   - Implemented PI controller (added Ki term)
   - Fixed feedforward to use measured `I_q`
   - Disabled spike rejection filter
   - Added comprehensive debug variables
   - Increased feedback limit from 10V → 18V
   - Added anti-windup logic

### Key Constants

```cpp
// Motor parameters (main_cpp.cpp)
stepper = StepperMotor(
    50,      // pole pairs
    1.4f,    // phase resistance (Ω)
    1.0f,    // KV rating
    3.2f,    // phase inductance (mH)
    20.0f,   // voltage limit (V)
    20.0f    // power supply voltage (V)
);

// Measured AC impedances (StepperMotor.cpp)
PHASE_RESISTANCE_1KHZ = 24.29Ω   // LCR measured at 1kHz
PHASE_RESISTANCE_10KHZ = 212Ω    // LCR measured at 10kHz
PHASE_RESISTANCE = 1.4Ω          // DC resistance (used for I×R drop)

// PI Controller gains (StepperMotor.cpp)
Kp_velocity_mech = 1.5f          // Proportional gain
Ki_velocity_mech = 0.5f          // Integral gain
MAX_FEEDBACK_CORRECTION = 18.0f  // Feedback voltage limit
INTEGRAL_MAX = 30.0f             // Integrator anti-windup limit (rad)
```

---

## Next Steps

1. **Collect missing diagnostic data:**
   - Add voltage signals (`g_voltage_q`, `g_desired_voltage_q_before_clamp`)
   - Add DQ current signals (`g_amperage_d` is critical!)
   - Monitor during acceleration and steady-state

2. **Verify motor is actually spinning:**
   - Visual check: ~30 RPM at 3 rad/s, ~162 RPM at 17 rad/s
   - Hand check: shaft spins freely when disabled?

3. **Check FOC alignment:**
   - `g_amperage_d` should be ~0A (if large, FOC is misaligned)
   - If misaligned: verify pole pairs, sensor offset, electrical angle calculation

4. **Check voltage saturation:**
   - If `g_voltage_q` = 20V constantly → voltage limited
   - May need higher supply voltage or lower target speed

5. **Verify pole pairs:**
   - Check motor datasheet: 50 pole pairs or 25?
   - Wrong pole pairs = wrong electrical angle = poor FOC = HEAT

6. **Tune PI gains if needed:**
   - If oscillates: reduce Kp to 1.0, reduce Ki to 0.3
   - If too slow: increase Kp to 2.0
   - Monitor `g_velocity_error_for_debug` for convergence

---

## References

### Motor Theory
- **Back-EMF:** V_emf = ω_e × (L×I_q + λ_pm)
- **Resistive drop:** V_R = I × R_dc (use DC resistance in DQ frame)
- **Inductive drop:** V_L = L × dI/dt (use AC inductance)
- **Total voltage:** V_q = I×R + L×dI/dt + back_emf

### FOC Fundamentals
- **DQ transformation:** Converts 3-phase AC → 2-axis DC
- **Q-axis:** Torque-producing current (perpendicular to flux)
- **D-axis:** Flux-producing current (should be ~0 for PMSM/stepper)
- **Good FOC:** I_d ≈ 0, all current in I_q for maximum torque/amp

### PI Controller
- **Proportional:** Fast response, but steady-state error
- **Integral:** Eliminates steady-state error, but can wind up
- **Anti-windup:** Essential to prevent integrator saturation
- **Tuning:** Start conservative (Kp=1.5, Ki=0.5), adjust based on response

---

**End of Debugging Session Documentation**

*For questions or follow-up, refer to the diagnostic variables and next steps sections.*