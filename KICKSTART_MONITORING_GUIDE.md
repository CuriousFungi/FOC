# Closed-Loop Kickstart Monitoring Guide

Quick reference for testing the angle discontinuity fix.

---

## Quick Start

### 1. Build and Flash
```bash
# Build the project
make clean && make

# Flash to STM32
st-flash write build/project3.bin 0x8000000
```

### 2. Connect STM32CubeMonitor

Add these variables to monitor the kickstart transition:

---

## Critical Variables to Monitor

### Group 1: Kickstart Tracking (Active during 0-2.5s)

| Variable | Expected Behavior | Problem Indicators |
|----------|-------------------|-------------------|
| `g_kickstart_target_angle_elec` | Linear ramp: 0 → 4.71 rad over 2.5s | Not ramping = not running |
| `g_cached_encoder_angle` | Tracks target with small lag (<0.3 rad) | Flat = motor not moving |
| `g_kickstart_angle_error` | Small oscillation around 0.1-0.2 rad | >0.5 rad = tracking failure |
| `g_kickstart_voltage_applied` | 5-7V, varies with error | Constant = no position control |
| `g_kickstart_just_completed` | Briefly true at end | Stays false = kickstart not finishing |

### Group 2: Transition Smoothness (At t=2.5s)

| Variable | Expected Behavior | Problem Indicators |
|----------|-------------------|-------------------|
| `g_cached_encoder_angle` | Continuous, no sudden jump | Jump = discontinuity still exists |
| `g_blended_angle_after_rate_limit` | Matches encoder smoothly | Jump = handoff problem |
| `g_voltage_q` | May change value but smoothly | Spike = voltage disturbance |
| `g_kickstart_actual_rotation_elec` | ~4.71 rad (270°) | <4.5 rad = incomplete rotation |
| `g_debug_ramped_speed` | Should equal target (10 rad/s) immediately | Decreasing = deceleration bug |
| `g_debug_actual_target_rps` | Should be 10 rad/s from initFOC | 0.5 rad/s = old code still active |
| `m_target` | Should be 10 rad/s after kickstart | 0.5 rad/s = wrong initialization |

### Group 3: Post-Transition (After t=2.5s)

| Variable | Expected Behavior | Problem Indicators |
|----------|-------------------|-------------------|
| `g_cached_encoder_angle` | Continues increasing (motor accelerating) | Decreasing = motor reversing |
| `g_measured_velocity_for_control` | Ramps up from ~1 rad/s | Stays at 0 = motor stalled |
| `g_voltage_q` | Adjusts based on velocity error | 0V = motor disabled |

---

## Timeline View

```
Time     Event                          Monitor
─────────────────────────────────────────────────────────────────
0.0s     Kickstart starts               g_kickstart_target_angle_elec = 0
                                        g_cached_encoder_angle = 0
                                        g_kickstart_angle_error = 0

0.5s     25% rotation                   g_kickstart_target_angle_elec ≈ 1.18 rad
                                        g_cached_encoder_angle ≈ 1.0-1.2 rad
                                        g_kickstart_angle_error ≈ 0.1-0.2 rad
                                        g_kickstart_voltage_applied ≈ 5.5V

1.25s    50% rotation                   g_kickstart_target_angle_elec ≈ 2.36 rad
                                        g_cached_encoder_angle ≈ 2.2-2.4 rad
                                        g_kickstart_angle_error ≈ 0.1-0.2 rad

2.0s     80% rotation                   g_kickstart_target_angle_elec ≈ 3.77 rad
                                        g_cached_encoder_angle ≈ 3.6-3.8 rad

2.5s     ★ TRANSITION ★                 g_kickstart_target_angle_elec = 4.71 rad
         (Critical moment!)             g_cached_encoder_angle ≈ 4.5-4.7 rad
                                        g_kickstart_actual_rotation_elec ≈ 4.5-4.7 rad
                                        ┌──────────────────────────────────┐
                                        │ NO JUMP - angle should be smooth │
                                        └──────────────────────────────────┘

2.5s+    CL speed control active        g_cached_encoder_angle continues smoothly
                                        g_measured_velocity_for_control > 0
                                        g_voltage_q adjusts via PI controller
```

---

## Success Criteria Checklist

### ✅ Kickstart Phase (0-2.5s)

- [ ] `g_kickstart_target_angle_elec` ramps linearly from 0 to ~4.71 rad
- [ ] `g_cached_encoder_angle` follows target (lag < 0.3 rad)
- [ ] `g_kickstart_angle_error` stays small (|error| < 0.5 rad)
- [ ] `g_kickstart_voltage_applied` varies between 5-8V
- [ ] `g_kickstart_actual_rotation_elec` = 4.5-4.9 rad at end

### ✅ Transition Phase (t=2.5s)

- [ ] **NO jump in `g_cached_encoder_angle`** (most critical!)
- [ ] **NO jump in `g_blended_angle_after_rate_limit`**
- [ ] `g_voltage_q` transitions smoothly (no spike)
- [ ] PWM signals smooth in Saleae (no glitches/disruption)

### ✅ Post-Transition Phase (t>2.5s)

- [ ] Motor continues rotating (encoder increasing)
- [ ] Velocity ramps up smoothly
- [ ] No audible click/vibration at 2.5s mark
- [ ] Motor reaches target speed (e.g., 20 rad/s)

---

## NEW: Transition Deceleration Bug (FIXED)

### Problem Description
**Symptoms:**
- Motor rotates during kickstart (0-2.5s)
- At transition (~2.5s), motor suddenly **decelerates or stops**
- PWM continues but motor doesn't accelerate
- `g_debug_ramped_speed` shows value decreasing toward 0.5 rad/s

**Root Cause:**
After kickstart, `m_target` was set to 0.5 rad/s, but motor was moving at ~2 rad/s. The ramped_speed static variable tried to decelerate to match the low target, causing motor to stop.

**Fix Applied:**
1. `m_target` now set to 10.0 rad/s after kickstart (not 0.5)
2. `ramped_speed` initializes to `target_mechanical_rps` on first call after kickstart
3. `update_ramp()` properly initializes `last_ramp_time` to prevent huge elapsed_us

**Verify Fix:**
```
g_debug_actual_target_rps:  Should be 10.0 rad/s immediately after kickstart
g_debug_ramped_speed:       Should initialize to 10.0, then ramp up (not down!)
g_voltage_q:                Should increase (accelerating), not decrease
```

**If Motor Still Stops:**
- Check `m_target` in debugger - should be 10.0 after `initFOC()`
- Check `g_kickstart_just_completed` - should briefly be true
- Check `ramped_speed` initialization - should match target, not stay at 0.5

---

## Failure Modes and Diagnostics

### Problem: Motor Not Moving During Kickstart

**Symptoms:**
- `g_cached_encoder_angle` stays flat/constant
- `g_kickstart_angle_error` grows continuously
- `g_kickstart_actual_rotation_elec` < 1.0 rad

**Diagnosis:**
```
Check: g_kickstart_voltage_applied
  If < 3V: Voltage too low, increase BASE_KICKSTART_VOLTAGE
  If > 3V: Motor issue (wiring, enable signal, PWM output)

Check: g_kickstart_angle_error
  If growing linearly: Position control not working
  If constant: Motor mechanically stuck
```

**Fix:**
- Increase `BASE_KICKSTART_VOLTAGE` in code
- Check motor wiring and enable signal
- Verify PWM outputs with oscilloscope

---

### Problem: Rotor Oscillates During Kickstart

**Symptoms:**
- `g_kickstart_angle_error` oscillates wildly (±0.5 rad)
- `g_kickstart_voltage_applied` swings between 0.5V and limit
- Motor vibrates/buzzes during kickstart

**Diagnosis:**
```
Check: g_kickstart_angle_error oscillation frequency
  If < 1 Hz (slow): Underdamped position control
  If > 5 Hz (fast): Encoder noise or electrical resonance

Check: g_kickstart_voltage_applied
  If hitting limits: Kp too high (too aggressive)
```

**Fix:**
- Reduce `Kp_position` from 2.0 to 1.0
- Increase `UPDATE_INTERVAL_US` to 20000 (slower)
- Add derivative damping (Kd term)

---

### Problem: Angle Jump Still Occurs at Transition

**Symptoms:**
- Sudden jump in `g_cached_encoder_angle` at t=2.5s
- PWM glitches visible in Saleae at transition
- Audible click or motor hesitation

**Diagnosis:**
```
Check: g_kickstart_end_angle_elec vs g_cached_encoder_angle (first CL value)
  If different: Angle source mismatch still exists
  Check: Is loopFOC() updating g_cached_encoder_angle during kickstart?

Check: g_blended_angle_after_rate_limit
  If jumps: Issue in update_speed_closed_loop initialization
```

**Fix:**
- Verify `loopFOC()` runs during kickstart (not blocked)
- Check `m_kickstart_active` flag is set/cleared correctly
- Ensure `g_cached_encoder_angle` updates every ~25-100μs

---

### Problem: Incomplete Rotation (<270°)

**Symptoms:**
- `g_kickstart_actual_rotation_elec` < 4.5 rad
- Motor stops before completing rotation
- Velocity too low in CL phase

**Diagnosis:**
```
Check: Final value of g_kickstart_angle_error at t=2.5s
  If large positive: Rotor lagging, needs more voltage/time
  If oscillating: Position control unstable

Check: g_kickstart_voltage_applied
  If hitting m_voltage_limit: Need higher voltage or longer duration
```

**Fix:**
- Increase `TOTAL_DURATION_US` to 3500000 (3.5 seconds)
- Increase `BASE_KICKSTART_VOLTAGE` by 20-50%
- Increase `Kp_position` to 3.0 for more aggressive tracking

---

## Saleae Logic Analyzer View

### What to Look For at Transition

**Before Fix (Old Open-Loop Kickstart):**
```
D4 (Phase A): ▓▓▓░░░▓▓▓  ← Consistent pattern
D5 (Phase B): ░░▓▓▓░░▓▓  ← Consistent pattern
D6 (Phase C): ▓░░▓▓▓░░▓  ← Consistent pattern
              ↓
              TRANSITION (t=2.5s)
              ↓
D4 (Phase A): ▓█░░░░█▓▓  ← Glitch! Sudden change
D5 (Phase B): ░░░██░▓▓░  ← Disruption
D6 (Phase C): █░░░░░▓░▓  ← Pattern broken
```

**After Fix (Closed-Loop Kickstart):**
```
D4 (Phase A): ▓▓▓░░░▓▓▓  ← Consistent pattern
D5 (Phase B): ░░▓▓▓░░▓▓  ← Consistent pattern
D6 (Phase C): ▓░░▓▓▓░░▓  ← Consistent pattern
              ↓
              TRANSITION (t=2.5s)
              ↓
D4 (Phase A): ▓▓▓░░░▓▓▓  ← Smooth continuation
D5 (Phase B): ░░▓▓▓░░▓▓  ← No disruption
D6 (Phase C): ▓░░▓▓▓░░▓  ← Pattern maintained
```

**Success = No visible disruption in PWM pattern at transition point**

---

## Quick Test Script

### Automated Test Sequence

1. **Reset motor**
2. **Trigger kickstart** (happens automatically in `initFOC()`)
3. **Monitor for 5 seconds** (covers transition at 2.5s)
4. **Ramp velocity** to target speed

### Expected Output Log

```
[0.00s] Kickstart started
[0.00s] g_kickstart_start_angle_elec = 0.123
[0.50s] g_kickstart_angle_error = 0.15 rad (tracking OK)
[1.00s] g_kickstart_angle_error = 0.18 rad (tracking OK)
[1.50s] g_kickstart_angle_error = 0.12 rad (tracking OK)
[2.00s] g_kickstart_angle_error = 0.20 rad (tracking OK)
[2.50s] ★ TRANSITION ★
[2.50s] g_kickstart_end_angle_elec = 4.65 rad
[2.50s] g_kickstart_actual_rotation_elec = 4.53 rad ✓
[2.50s] Angle jump = 0.00 rad ✓✓✓ SUCCESS!
[2.51s] CL speed control active
[3.00s] Velocity = 2.5 rad/s (ramping up)
[4.00s] Velocity = 10.2 rad/s (tracking target)
```

---

## Tuning Quick Reference

| Issue | Parameter | Direction | Typical Values |
|-------|-----------|-----------|----------------|
| Rotor lags target | `Kp_position` | Increase | 1.0 → 2.0 → 3.0 |
| Rotor oscillates | `Kp_position` | Decrease | 2.0 → 1.0 → 0.5 |
| Insufficient rotation | `TOTAL_DURATION_US` | Increase | 2500000 → 3500000 |
| Insufficient rotation | `BASE_KICKSTART_VOLTAGE` | Increase | 5V → 7V → 10V |
| Noisy tracking | `UPDATE_INTERVAL_US` | Increase | 10000 → 20000 |
| Slow response | `UPDATE_INTERVAL_US` | Decrease | 10000 → 5000 |

---

## Expected Performance

### Typical Values (Unloaded Motor)

| Metric | Value | Tolerance |
|--------|-------|-----------|
| Kickstart duration | 2.5 s | Fixed |
| Final rotation | 4.71 rad | ±0.2 rad OK |
| Tracking error | 0.15 rad | <0.3 rad OK |
| Voltage applied | 5-7 V | Varies with error |
| Angle jump at transition | **0.0 rad** | **<0.05 rad acceptable** |
| Transition time | <1 ms | Instantaneous |

### Performance Under Load

| Load Condition | Expected Error | Voltage Applied | Notes |
|----------------|---------------|-----------------|-------|
| No load | 0.1-0.2 rad | 5-6 V | Minimal correction |
| Light (friction) | 0.2-0.3 rad | 6-7 V | Normal operation |
| Moderate | 0.3-0.5 rad | 7-9 V | May need higher Kp |
| Heavy | 0.5+ rad | 9-12 V | May need longer duration |

---

## Contact Points

If the fix doesn't work as expected, check these in order:

1. **Verify `g_cached_encoder_angle` updates during kickstart**
   - Should change every ~25-100μs
   - If frozen: loopFOC() not running

2. **Verify `setPhaseVoltage()` receives encoder angle**
   - Add breakpoint in kickstartMotor()
   - Confirm `actual_angle_elec = g_cached_encoder_angle`

3. **Verify no wraparound issues**
   - Angle error should handle 0/2π crossing
   - Check for sudden 6.28 rad jumps

4. **Verify voltage limits not hit constantly**
   - If `g_kickstart_voltage_applied` always at limit
   - Motor may need higher voltage or longer time

---

## Summary

**Primary Goal:** Eliminate angle discontinuity at kickstart→CL transition

**Key Indicator:** `g_cached_encoder_angle` must be **continuous** (no jump) at t=2.5s

**Success = Smooth PWM + No motor disturbance + No audible click**

**Test Duration:** 5 seconds (kickstart + transition + initial CL ramp)

**Tools:** STM32CubeMonitor + Saleae Logic Analyzer

**Expected Outcome:** Motor operates smoothly from standstill through kickstart and into closed-loop speed control without any perceptible transition event.