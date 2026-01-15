# PWM Fix Testing and Verification Guide

## Quick Reference: What Changed

### Before (Sign/Magnitude - CAUSED SHOOT-THROUGH)
```cpp
if (U_alpha >= 0) {
    duty_cycle_1A = 0%;           // OFF
    duty_cycle_1B = 60%;          // ON
} else {
    duty_cycle_1A = 60%;          // ON
    duty_cycle_1B = 0%;           // OFF
}
```
**Problem:** During transitions, both could be momentarily active → shoot-through

### After (Locked Anti-Phase - PREVENTS SHOOT-THROUGH)
```cpp
const float BASE_DUTY = 0.5f;  // 50% center point

duty_cycle_1A = 0.5 - (U_alpha/V_supply) * 0.5;  // 20-80% range
duty_cycle_1B = 0.5 + (U_alpha/V_supply) * 0.5;  // 80-20% range

// Net voltage = (1B - 1A) * V_supply
```
**Solution:** Both channels always active, complementary around 50% → no shoot-through

---

## Testing Procedure

### Step 1: Rebuild and Flash
1. Save all files
2. Clean build: `make clean` (or use IDE clean)
3. Rebuild: `make` (or use IDE build)
4. Flash to STM32H7
5. Reset the board

### Step 2: Monitor Debug Variables

Add these to your debugger watch window:

#### Input Voltages (From FOC)
```
g_U_alpha_debug     // Should be: -20V to +20V sinusoidal
g_U_beta_debug      // Should be: -20V to +20V sinusoidal (90° phase shift)
g_voltage_q         // Should be: 8-20V (depends on speed)
g_voltage_d         // Should be: 0V (no field weakening)
```

#### Duty Cycles (NEW - Should be Complementary)
```
g_dutycycle_1A      // Should be: 0.2 to 0.8 (20% to 80%)
g_dutycycle_1B      // Should be: 0.8 to 0.2 (80% to 20%)
g_dutycycle_2A      // Should be: 0.2 to 0.8 (20% to 80%)
g_dutycycle_2b      // Should be: 0.8 to 0.2 (80% to 20%)
```

#### Verification Checks
```
g_dutycycle_1A + g_dutycycle_1B  // Should equal: ~1.0 (100%)
g_dutycycle_2A + g_dutycycle_2b  // Should equal: ~1.0 (100%)
```

#### Velocity Control
```
g_kalman_velocity              // Actual velocity (rad/s)
g_target_rad_per_sec          // Target velocity (rad/s)
g_velocity_error_for_debug    // Error (should converge to ~0)
```

#### Current Sensing
```
g_current_a_filtered          // Phase A current (~1.5A sinusoidal)
g_current_b_filtered          // Phase B current (~1.5A sinusoidal)
```

---

## Expected Behavior

### Duty Cycle Patterns

#### When U_alpha = 0V (motor at 0° or 180° electrical):
- `g_dutycycle_1A = 0.50` (50%)
- `g_dutycycle_1B = 0.50` (50%)
- Net voltage = (0.5 - 0.5) × 20V = **0V** ✅

#### When U_alpha = +10V (positive torque):
- `g_dutycycle_1A = 0.25` (25%)
- `g_dutycycle_1B = 0.75` (75%)
- Net voltage = (0.75 - 0.25) × 20V = **+10V** ✅

#### When U_alpha = -10V (negative torque):
- `g_dutycycle_1A = 0.75` (75%)
- `g_dutycycle_1B = 0.25` (25%)
- Net voltage = (0.25 - 0.75) × 20V = **-10V** ✅

### Sum Check (CRITICAL)
At ALL times, regardless of U_alpha or U_beta value:
```
g_dutycycle_1A + g_dutycycle_1B ≈ 1.0 ± 0.05
g_dutycycle_2A + g_dutycycle_2b ≈ 1.0 ± 0.05
```

If the sum is NOT close to 1.0, something is wrong!

---

## Oscilloscope Verification

### Setup
- **PWM Frequency:** 20 kHz (50μs period)
- **Supply Voltage:** 20V
- **Steady-State Condition:** U_alpha = +10V (desired voltage)

### What You Should See

**Pin 1A (Channel 1):**
```
One PWM period (50μs):
    ___
   |   |_________________________________________
   |12.5μs|        37.5μs LOW                   |
   
   Duty cycle = 12.5/50 = 25%
```

**Pin 1B (Channel 2):**
```
One PWM period (50μs):
    _______________________________________
   |                                       |___
   |        37.5μs HIGH                    |12.5μs|
   
   Duty cycle = 37.5/50 = 75%
```

### Key Points to Verify

1. **Both pins PWM every cycle** - neither is stuck at 0% or 100%

2. **Complementary timing:**
   - When 1A is HIGH, 1B is LOW
   - When 1A is LOW, 1B is HIGH
   - They switch at the same time (with small dead-time)

3. **Duty cycle math:**
   - 1A duty: 25%
   - 1B duty: 75%
   - Sum: 25% + 75% = 100% ✓
   - Net voltage: (75% - 25%) × 20V = 10V ✓

4. **Dead-time (optional):**
   - Brief moment where both are LOW during transitions
   - Typically 1-2μs (200 timer ticks configured)
   - Prevents shoot-through during switching

### Other Test Cases

**U_alpha = 0V (zero voltage):**
- 1A duty: 50%
- 1B duty: 50%
- Net: (50% - 50%) × 20V = 0V

**U_alpha = -10V (negative voltage):**
- 1A duty: 75%
- 1B duty: 25%
- Net: (25% - 75%) × 20V = -10V

**U_alpha = +20V (maximum positive):**
- 1A duty: 0%
- 1B duty: 100%
- Net: (100% - 0%) × 20V = +20V

---

## Troubleshooting

### Problem: Duty cycles NOT summing to 1.0

**Possible Causes:**
1. `hifactor_1/2` or `lofactor_1/2` not equal to 1.0
2. Duty cycle limiting too aggressive
3. Voltage clamping issue

**Fix:**
```cpp
// In StepperMotor constructor, verify:
m_hifactor_a = 1.0f;
m_lofactor_a = 1.0f;
m_hifactor_b = 1.0f;
m_lofactor_b = 1.0f;
```

### Problem: Motor still doesn't spin

**Check:**
1. Are duty cycles actually changing? (Not stuck at 0.5/0.5)
2. Is `g_voltage_q` non-zero? (Should be 8-20V when running)
3. Is `g_kalman_velocity` updating? (Encoder working?)
4. Is motor enabled? (`g_motor_enabled_status = 1`)
5. Power supply adequate? (Should handle 3-4A current draw)

### Problem: Motor vibrates but doesn't rotate smoothly

**Likely Causes:**
1. Commutation angle offset wrong (check `m_sensor_offset`)
2. Velocity controller gains need tuning (Kp too high/low)
3. Current limiting too aggressive
4. Encoder direction inverted

**Tune:**
```cpp
// Try reducing Kp:
float Kp = 0.2f;  // Down from 0.5
```

### Problem: Duty cycles go outside [0, 1] range

This shouldn't happen with the `constrain()` function, but if it does:

**Check:**
1. `m_power_supply_voltage` set correctly? (Should be 20.0f)
2. `m_voltage_limit` set correctly? (Should be 20.0f)
3. `U_alpha` and `U_beta` being clamped before calculation?

---

## Performance Metrics

### Success Criteria

| Metric | Expected Value | Tolerance |
|--------|---------------|-----------|
| Duty sum (1A+1B) | 1.0 | ±0.05 |
| Duty sum (2A+2B) | 1.0 | ±0.05 |
| Duty cycle range | 0.2 - 0.8 | ±0.1 |
| Phase current | 1.5A peak | ±0.5A |
| Velocity error | 0 rad/s | ±0.1 rad/s |
| Shoot-through events | 0 | 0 |

### Baseline Values (for comparison)

With 20V supply and velocity control active:
- `g_voltage_q`: 8-15V (depends on speed)
- `g_U_alpha`: -15V to +15V (sinusoidal)
- `g_U_beta`: -15V to +15V (sinusoidal, 90° shift)
- `g_dutycycle_1A`: 0.25-0.75 (varies sinusoidally)
- `g_dutycycle_1B`: 0.75-0.25 (complementary)

---

## If Motor Works: Next Steps

1. **Tune Velocity Controller**
   - Adjust `Kp`, `Ki`, `Kd` for optimal response
   - Test different target velocities
   - Verify overshoot and settling time

2. **Test Acceleration/Deceleration**
   - Increase ramp rate from 0.1 rad/s²
   - Verify smooth transitions
   - Check current limiting during transients

3. **Implement Position Control**
   - Add outer position loop
   - Use velocity loop as inner loop
   - Test point-to-point moves

4. **Add Safety Features**
   - Overcurrent protection
   - Overvoltage protection
   - Encoder fault detection
   - Thermal management

---

## Reference: Key Equations

### Locked Anti-Phase PWM
```
normalized_voltage = U / V_supply       // Range: -1 to +1
base_duty = 0.5                          // Center point: 50%

duty_A = base_duty - normalized_voltage × 0.5
duty_B = base_duty + normalized_voltage × 0.5

net_voltage = (duty_B - duty_A) × V_supply = U
```

### Inverse Park Transform
```
U_alpha = cos(θ) × Ud - sin(θ) × Uq
U_beta  = sin(θ) × Ud + cos(θ) × Uq

// For FOC with Ud = 0:
U_alpha = -sin(θ) × Uq
U_beta  = cos(θ) × Uq
```

---

## Contact/Support

If the motor still doesn't work after this fix:

1. **Check Hardware:**
   - H-bridge wiring correct?
   - Power supply adequate (4A+)?
   - Motor phases connected properly?
   - Encoder working? (check `g_sensor_angle`)

2. **Check Software:**
   - Timer initialized correctly?
   - GPIO alternate functions correct?
   - Dead-time configured? (200 ticks in TIM1/TIM8)

3. **Alternative Solutions:**
   - Try using timer complementary outputs (CH1N, CH2N)
   - Try different H-bridge driver (if L298N has issues)
   - Try open-loop mode first (disable velocity feedback)

---

## Summary

**What we fixed:** Changed from sign/magnitude PWM (one side on, one side off) to locked anti-phase PWM (both sides always active, complementary around 50%).

**Why it matters:** Prevents shoot-through during voltage polarity transitions in the H-bridge.

**Expected result:** Motor spins smoothly without shoot-through, velocity control loop functions correctly.

Good luck! 🎉