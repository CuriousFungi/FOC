# PWM Shoot-Through Fix - Quick Reference

## Problem
Motor made noise but didn't spin. Root cause: **H-bridge shoot-through** from sign/magnitude PWM control.

## Solution Implemented
Changed from **Sign/Magnitude PWM** to **Locked Anti-Phase PWM** in `StepperDriver.hpp`

---

## What Changed

### Before (WRONG - Caused Shoot-Through)
```cpp
// One channel ON, other channel OFF
if (U_alpha >= 0) {
    duty_1A = 0%;      // OFF
    duty_1B = 60%;     // ON
} else {
    duty_1A = 60%;     // ON
    duty_1B = 0%;      // OFF
}
// Problem: Transition between modes → shoot-through
```

### After (CORRECT - Prevents Shoot-Through)
```cpp
// Both channels ALWAYS active, complementary around 50%
const float BASE_DUTY = 0.5f;
duty_1A = 0.5 - (U_alpha/V_supply) * 0.5;  // 20-80%
duty_1B = 0.5 + (U_alpha/V_supply) * 0.5;  // 80-20%

// duty_1A + duty_1B always equals 1.0
// Net voltage = (duty_1B - duty_1A) * V_supply = U_alpha
```

---

## Expected Behavior

### Duty Cycle Verification
Monitor these in debugger:
```
g_dutycycle_1A + g_dutycycle_1B ≈ 1.0  (100%)
g_dutycycle_2A + g_dutycycle_2b ≈ 1.0  (100%)
```

### Example Values
| U_alpha | duty_1A | duty_1B | Net Voltage |
|---------|---------|---------|-------------|
| 0V      | 50%     | 50%     | 0V          |
| +10V    | 25%     | 75%     | +10V        |
| -10V    | 75%     | 25%     | -10V        |

---

## Testing Checklist

- [ ] Rebuild and flash firmware
- [ ] Check: `g_dutycycle_1A + g_dutycycle_1B ≈ 1.0`
- [ ] Check: `g_dutycycle_2A + g_dutycycle_2b ≈ 1.0`
- [ ] Verify: No shoot-through current spikes on scope
- [ ] Verify: Motor spins smoothly
- [ ] Verify: Current waveforms sinusoidal (~1.5A)
- [ ] Verify: Velocity control tracks setpoint

---

## Files Modified

1. **StepperDriver.hpp** (lines 241-330)
   - Replaced sign/magnitude logic with locked anti-phase
   - Added `constrain()` helper function
   - Simplified duty cycle calculation

---

## Detailed Documentation

- **PWM_SHOOTTHROUGH_DIAGNOSIS.md** - Full problem diagnosis and fix
- **PWM_FIX_TESTING_GUIDE.md** - Testing procedures and troubleshooting
- **LOCKED_ANTIPHASE_PWM_THEORY.md** - Mathematical theory and explanation

---

## Why This Works

**Locked anti-phase PWM** ensures both H-bridge sides are always switching with complementary duty cycles:
- No polarity transitions that could cause overlap
- Both sides always complementary → no shoot-through possible
- Linear voltage control from -V_supply to +V_supply
- Perfect for FOC's bipolar sinusoidal voltages

---

## Troubleshooting

### Motor still doesn't spin?
1. Check duty cycle sums equal 1.0
2. Verify `g_voltage_q` is non-zero (8-20V)
3. Check motor enabled: `g_motor_enabled_status = 1`
4. Verify encoder working: `g_kalman_velocity` updating
5. Check power supply adequate (4A+)

### Duty cycles not summing to 1.0?
Check in `StepperMotor` constructor:
```cpp
m_hifactor_a = 1.0f;
m_lofactor_a = 1.0f;
m_hifactor_b = 1.0f;
m_lofactor_b = 1.0f;
```

---

## Next Steps After Motor Works

1. Tune velocity controller (Kp, Ki, Kd)
2. Test acceleration/deceleration ramps
3. Implement position control loop
4. Add safety features (overcurrent, etc.)

---

## Key Equation
```
duty_A = 0.5 - (U / V_supply) × 0.5
duty_B = 0.5 + (U / V_supply) × 0.5

Net voltage = (duty_B - duty_A) × V_supply = U ✓
```

---

**Status:** ✅ Fix implemented, ready for testing

Good luck! 🚀