# Testing Quick Reference - Direction Calibration Fix

## Pre-Flight Checklist

Before flashing firmware:

- [ ] Verify `NUM_POLE_PAIRS = 50` in main_cpp.cpp (for 200-step motor)
- [ ] Backup current firmware (in case rollback needed)
- [ ] Encoder connected and powered
- [ ] Motor phases connected to driver
- [ ] Power supply voltage correct (20V configured)

## Flash and Initial Test

### 1. Flash Firmware
```
Build project → Flash to STM32H755
```

### 2. Monitor Alignment (First 10 seconds)
Watch debug variables during `initFOC()`:

```
Time: 0-5s     → SPI initialization, buffers filling
Time: 5-6s     → First alignment step (0° electrical)
Time: 6-7s     → Second alignment step (+90° electrical)  
Time: 7-8s     → Direction determination
Time: 8-9s     → Final alignment step (0° electrical again)
Time: 9-10s    → Offset calculation complete
```

### 3. Check Calibration Results

**Critical Debug Variables:**

| Variable | Expected Value | What It Means |
|----------|----------------|---------------|
| `g_align_mech_angle_0` | 0.0 to 6.28 | Mechanical angle at 0° electrical |
| `g_align_mech_angle_90` | 0.0 to 6.28 | Mechanical angle at +90° electrical |
| `g_align_mech_delta` | **±0.0314 rad** | Change in angle (±1.8°) |
| `g_align_direction_inversion` | 0.0 or 1.0 | 1.0 = inversion applied |
| `g_align_offset_calculated` | 0 to 314 rad | Electrical offset (0 to 50×2π) |

**Pass Criteria:**
- ✅ `mech_delta` is ±0.03 to ±0.04 radians (not 0, not >0.1)
- ✅ Direction inversion is 0.0 or 1.0 (not random)
- ✅ Offset is reasonable (not 0, not NaN)

**Fail Indicators:**
- ❌ `mech_delta` near zero → motor didn't move during alignment
- ❌ `mech_delta` > 0.1 rad → encoder wrapped around or wrong pole pairs
- ❌ Offset = 0 → calibration failed to calculate
- ❌ Any value = 0xDEAD or NaN → encoder read failure

## Motor Operation Tests

### Test 1: Open-Loop Velocity (Baseline)

```cpp
stepper.m_motion_control = MOTION_CONTROL_TYPE::OL_VELOCITY;
stepper.move(5.0f);  // 5 rad/s
```

**Expected:**
- Motor rotates smoothly in one direction
- No cogging, juddering, or stalling
- Speed is approximately correct (~47 RPM)

**If motor fights or cogs:**
→ Direction or offset is still wrong

### Test 2: Closed-Loop Velocity (Full Test)

```cpp
stepper.m_motion_control = MOTION_CONTROL_TYPE::CL_VELOCITY;
stepper.move(10.0f);  // 10 rad/s
```

**Expected:**
- Motor accelerates smoothly to target speed
- Encoder angle increases continuously
- Current (Iq) is smooth and proportional to load
- Speed regulation works (maintains target even with load)

**Monitor these:**
```
g_encoder_mech_angle_raw    → Increases 0 to 2π per rotation
g_encoder_elec_angle_raw    → Cycles 0 to 2π fifty times per rotation
g_amperage_q                → Smooth, ~0.5-2A depending on load
g_amperage_d                → Near zero (< 0.2A)
g_velocity_error_for_debug  → Converges to zero
```

### Test 3: Direction Reversal

```cpp
stepper.move(10.0f);   // Forward
delay(3000);
stepper.move(-10.0f);  // Reverse
```

**Expected:**
- Motor reverses smoothly
- No hesitation or stalling at zero crossing
- Same smooth operation in both directions

## Diagnostic Flowchart

```
┌─────────────────────────────┐
│   Alignment Complete?       │
└──────────┬──────────────────┘
           │
           ├─NO──→ Check encoder connection, SPI clock
           │
           ├─YES
           ▼
┌─────────────────────────────┐
│  mech_delta ≈ ±0.03 rad?    │
└──────────┬──────────────────┘
           │
           ├─NO──→ Wrong NUM_POLE_PAIRS or encoder not moving
           │
           ├─YES
           ▼
┌─────────────────────────────┐
│   Motor rotates smoothly    │
│   in open-loop?             │
└──────────┬──────────────────┘
           │
           ├─NO──→ Phase wiring issue or wrong direction sense
           │        Try inverting: need_inversion = (mech_delta > 0)
           │
           ├─YES
           ▼
┌─────────────────────────────┐
│   Motor tracks target       │
│   in closed-loop?           │
└──────────┬──────────────────┘
           │
           ├─NO──→ Offset wrong or encoder noise
           │        Check g_encoder_elec_angle_raw updates smoothly
           │
           ├─YES
           ▼
┌─────────────────────────────┐
│      SUCCESS! ✓             │
└─────────────────────────────┘
```

## Common Issues and Fixes

### Issue 1: Motor vibrates/cogs instead of rotating

**Symptoms:**
- High-frequency vibration
- Motor oscillates but doesn't rotate
- High current (> 3A)

**Diagnosis:**
```
Check: g_angle_to_setPhaseVoltage
→ Should ramp smoothly 0 to 2π repeatedly
→ If stuck at one value: angle not updating
→ If jumping randomly: encoder problem
```

**Fix:**
- Verify encoder is actually moving (check `g_as5048_angle`)
- Check if direction inversion is backwards
- Try opposite direction sense in alignSensor()

### Issue 2: Motor rotates but wrong speed

**Symptoms:**
- Motor rotates smoothly but too fast/slow
- Speed doesn't match target

**Diagnosis:**
```
Check: NUM_POLE_PAIRS
50 pole pairs = 200 steps/rev
100 pole pairs = 400 steps/rev
```

**Fix:**
- Verify motor specification (steps per revolution)
- Adjust `NUM_POLE_PAIRS` in main_cpp.cpp
- Re-run alignment after changing

### Issue 3: Works in one direction, not the other

**Symptoms:**
- Forward rotation: smooth
- Reverse rotation: cogs or stalls

**Diagnosis:**
```
Check: Phase voltages (g_U_alpha, g_U_beta)
→ Should be sinusoidal in both directions
→ If clipped: PWM duty cycle issue
→ If asymmetric: driver problem
```

**Fix:**
- Check driver enable pins
- Verify power supply can source and sink current
- Inspect all three phase connections

### Issue 4: Encoder angle stuck at zero

**Symptoms:**
- `g_as5048_angle` always 0.0
- `g_encoder_mech_angle_raw` never changes
- Motor runs open-loop OK but closed-loop fails

**Diagnosis:**
```
Check: SPI communication
→ Monitor g_fetch_radians_success_count
→ Should increment at 40 kHz (40,000 per second)
→ If not incrementing: SPI not working
```

**Fix:**
- Verify SPI clock is running (TIM1 triggering)
- Check CS, MOSI, MISO, SCK connections
- Verify AS5048A power (3.3V or 5V)

## Quick Fixes to Try

### Fix 1: Invert Direction Sense (if motor fights)

In `StepperMotor.cpp`, line ~1103:
```cpp
// Change from:
bool need_inversion = (mech_delta < 0.0f);

// To:
bool need_inversion = (mech_delta > 0.0f);
```

### Fix 2: Try Negative Pole Pairs (if angle advances wrong rate)

In `main_cpp.cpp`, line ~123:
```cpp
// Change from:
50,  // NUM_POLE_PAIRS

// To:
-50,  // NUM_POLE_PAIRS (negative)
```

### Fix 3: Disable Offset (to test if offset calculation is wrong)

In `StepperMotor.cpp`, get_electric_angle_radians():
```cpp
// Temporarily comment out offset:
// float raw_angle = electic_radians - m_radian_offset_to_electric_zero;
float raw_angle = electic_radians;  // Test without offset
```

### Fix 4: Increase Alignment Voltage (if motor doesn't move during alignment)

In `StepperMotor.cpp` or motor initialization:
```cpp
m_voltage_sensor_align = 3.0f;  // Increase from default (was 2.0V)
```

## Success Criteria Checklist

Calibration phase:
- [x] `g_align_mech_delta` is ±0.03 rad (not zero, not huge)
- [x] `g_align_direction_inversion` is 0 or 1 (not undefined)
- [x] `g_align_offset_calculated` is reasonable (not 0, not NaN)

Open-loop operation:
- [x] Motor rotates smoothly at commanded speed
- [x] No cogging or vibration
- [x] Works in both directions

Closed-loop operation:
- [x] Encoder angle updates smoothly (0 to 2π per rotation)
- [x] Motor tracks target speed within ±10%
- [x] Velocity error converges to near zero
- [x] Current (Iq) is smooth (< 3A)
- [x] Direction reversal works smoothly

## Measurements to Record

Record these values for analysis:

```
=== Calibration Results ===
g_align_mech_angle_0:         _______
g_align_mech_angle_90:        _______
g_align_mech_delta:           _______
g_align_direction_inversion:  _______
g_align_offset_calculated:    _______

=== Runtime Performance ===
Target speed (rad/s):         10.0
Actual speed (rad/s):         _______
Velocity error:               _______
Iq current (A):               _______
Id current (A):               _______

=== Pass/Fail ===
Alignment successful:         [ ] YES  [ ] NO
Open-loop smooth:             [ ] YES  [ ] NO
Closed-loop tracks target:    [ ] YES  [ ] NO
Bidirectional operation:      [ ] YES  [ ] NO

Overall:                      [ ] PASS [ ] FAIL
```

## Contact Info / Support

If all tests fail:
1. Double-check encoder wiring (SPI connections)
2. Verify motor phases are connected correctly (ABC order matters)
3. Check power supply voltage matches configuration (20V)
4. Review alignment debug variables for obvious failures
5. Try the "Quick Fixes" above one at a time

---

**Remember:** The alignment procedure runs automatically on power-up. If you need to re-run alignment, power cycle the board (don't just reset).