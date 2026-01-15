# TEST NOW - Kickstart Transition Fix Checklist

## Quick Reference Card

**Problem Fixed:** Motor stopping at kickstart→CL transition
**Fixes Applied:** 2 (angle continuity + velocity continuity)
**Testing Time:** ~5 minutes
**Success Indicator:** Motor accelerates smoothly through transition

---

## Pre-Test: Build & Flash

```bash
cd C:\_projects\FOC\cube\project3
make clean
make
st-flash write build/project3.bin 0x8000000
```

**Expected:** No compile errors, successful flash

---

## Test Setup: STM32CubeMonitor Variables

### Critical Variables (MUST Monitor)

Add these 6 variables to see if fix worked:

```
1. g_cached_encoder_angle           ← Must be continuous at t=2.5s (NO JUMP!)
2. g_debug_ramped_speed             ← Must be 10.0 after kickstart (not 0.5)
3. m_target                         ← Must be 10.0 after kickstart (not 0.5)
4. g_voltage_q                      ← Must INCREASE after transition (not decrease)
5. g_measured_velocity_for_control  ← Must INCREASE after transition
6. g_kickstart_just_completed       ← Should briefly be true at t=2.5s
```

### Optional (For Detailed Analysis)

```
g_kickstart_target_angle_elec       ← Ramps 0→4.71 during kickstart
g_kickstart_angle_error             ← Should be < 0.5 rad
g_kickstart_voltage_applied         ← Should vary 5-7V
g_velocity_error_for_debug          ← Should be positive (+8 rad/s at transition)
```

---

## Test Execution: What to Watch

### Phase 1: Kickstart (0-2.5 seconds)

**Motor should:**
- ✓ Rotate smoothly
- ✓ Complete ~270° rotation
- ✓ No excessive vibration

**Monitor shows:**
- ✓ `g_kickstart_target_angle_elec`: Ramps from 0 to ~4.71 rad
- ✓ `g_cached_encoder_angle`: Tracks target (lag < 0.3 rad)
- ✓ `g_kickstart_angle_error`: Small (~0.1-0.2 rad)

**If motor doesn't move:**
- Check `g_kickstart_voltage_applied` (should be 5-7V)
- Check motor enable signal
- Check PWM output

---

### Phase 2: TRANSITION (t = 2.5 seconds) ★ CRITICAL ★

**This is where the fix matters!**

**Monitor shows:**
- ✅ `g_cached_encoder_angle`: **CONTINUOUS** (no sudden jump)
- ✅ `g_kickstart_just_completed`: true (briefly)
- ✅ `m_target`: **10.0 rad/s** (not 0.5!)
- ✅ `g_debug_ramped_speed`: **10.0 rad/s** (not 0.5!)
- ✅ `g_velocity_error_for_debug`: **+8 rad/s** (positive = accelerate)
- ✅ `g_voltage_q`: **INCREASING** (not decreasing)

**Motor should:**
- ✅ **NO CLICK** (no audible sound)
- ✅ **NO HESITATION** (continuous motion)
- ✅ **NO STOPPING** (keeps accelerating)

**Saleae (if connected):**
- ✅ PWM signals smooth (no glitches at transition)

---

### Phase 3: Acceleration (2.5-7.5 seconds)

**Motor should:**
- ✓ Accelerate continuously
- ✓ Reach ~20-40 rad/s
- ✓ Smooth operation (no jerking)

**Monitor shows:**
- ✓ `g_debug_ramped_speed`: Ramps up (10→20→30→... rad/s)
- ✓ `g_measured_velocity_for_control`: Follows ramped_speed
- ✓ `g_voltage_q`: Varies 8-15V (based on error)

---

## Pass/Fail Criteria

### ✅ PASS - Fix Successful

**All of these must be true:**

1. ✅ `g_cached_encoder_angle` continuous at t=2.5s (no jump > 0.05 rad)
2. ✅ `m_target` = 10.0 rad/s after kickstart (visible in trace)
3. ✅ `g_debug_ramped_speed` = 10.0 rad/s after kickstart
4. ✅ Motor does NOT stop or decelerate at transition
5. ✅ No audible click or vibration at t=2.5s
6. ✅ Motor accelerates to at least 20 rad/s

**Result:** 🎉 Both fixes working correctly! Motor operates smoothly.

---

### ❌ FAIL - Issue Still Present

**If ANY of these occur:**

1. ❌ `g_cached_encoder_angle` JUMPS at t=2.5s (sudden discontinuity)
   → **Angle fix didn't work** - check kickstart implementation

2. ❌ `m_target` = 0.5 rad/s after kickstart (not 10.0)
   → **Old code still active** - check initFOC() change

3. ❌ `g_debug_ramped_speed` = 0.5 or decreasing after transition
   → **Velocity fix didn't work** - check ramped_speed initialization

4. ❌ Motor STOPS or DECELERATES at transition
   → **Check all of above** - velocity continuity issue

5. ❌ Audible click or motor jerks at t=2.5s
   → **Angle discontinuity still present** - check encoder angle usage

6. ❌ PWM glitches visible in Saleae at transition
   → **Angle discontinuity** - verify setPhaseVoltage receives encoder angle

---

## Troubleshooting

### Issue: Motor Doesn't Move During Kickstart

**Check:**
- `g_kickstart_voltage_applied` < 3V? → Increase BASE_KICKSTART_VOLTAGE
- Motor enable signal active?
- PWM outputs working?

**Fix:** Increase voltage in code (5V → 7V)

---

### Issue: Angle Jumps at Transition (Fix #1 Failed)

**Check:**
- Is `g_cached_encoder_angle` updating during kickstart?
- Does `setPhaseVoltage` receive `g_cached_encoder_angle` in kickstart?

**Verify:** Add breakpoint in kickstartMotor() at setPhaseVoltage call

---

### Issue: Motor Stops at Transition (Fix #2 Failed)

**Check:**
- `m_target` value in debugger after kickstart (should be 10.0)
- `g_kickstart_just_completed` ever becomes true?
- `g_debug_ramped_speed` initialization

**Verify:** 
```
Breakpoint at line 731 (initFOC): m_target should be set to 10.0
Breakpoint at line 2820 (update_speed_closed_loop): ramped_speed should initialize to 10.0
```

---

### Issue: Motor Oscillates During Kickstart

**Check:** `g_kickstart_angle_error` oscillating > ±0.5 rad?

**Fix:** Reduce Kp_position from 2.0 to 1.0 in kickstartMotor()

---

### Issue: Incomplete Rotation (< 270°)

**Check:** `g_kickstart_actual_rotation_elec` < 4.5 rad?

**Fix:** 
- Increase TOTAL_DURATION_US from 2500000 to 3500000 (3.5 seconds)
- OR increase BASE_KICKSTART_VOLTAGE by 20-50%

---

## Quick Diagnosis Flow

```
START
  │
  ├─ Motor moves during kickstart?
  │  NO → Check voltage, enable signal, PWM
  │  YES ↓
  │
  ├─ Angle continuous at t=2.5s?
  │  NO → Fix #1 failed (angle discontinuity)
  │  YES ↓
  │
  ├─ m_target = 10.0 after kickstart?
  │  NO → Fix #2 failed (wrong target)
  │  YES ↓
  │
  ├─ Motor accelerates after transition?
  │  NO → Check g_debug_ramped_speed initialization
  │  YES ↓
  │
  └─ SUCCESS! ✅ Both fixes working
```

---

## Data to Collect

### For Successful Test

**Screenshot/export from STM32CubeMonitor:**
- Full trace showing kickstart through acceleration (0-5s)
- Zoom on transition region (2.4-2.6s)

**Saleae capture:**
- PWM signals at transition point
- Before/after comparison

**Notes:**
- Final speed achieved: _____ rad/s
- Transition smoothness: _____ (smooth/minor glitch/major glitch)
- Any unusual behavior: _____

---

### For Failed Test

**What failed:**
- [ ] Motor doesn't move during kickstart
- [ ] Angle jumps at transition
- [ ] Motor stops at transition
- [ ] Motor oscillates during kickstart
- [ ] Other: _____

**Variable values at failure:**
```
m_target at t=2.5s:                    _____
g_debug_ramped_speed at t=2.5s:        _____
g_cached_encoder_angle jump magnitude: _____
g_kickstart_actual_rotation_elec:      _____
```

---

## Expected Graph Appearance

### Success Pattern

```
g_cached_encoder_angle:
  6 ┤                                    ╭─────
  5 ┤                            ╭───────╯
  4 ┤                    ╭───────╯
  3 ┤            ╭───────╯               ← Smooth continuous curve
  2 ┤    ╭───────╯
  1 ┤────╯
  0 ┼────────────────────────────────────
    0s  1s  2s  2.5s 3s  4s  5s
            ↑
        Transition (no visible discontinuity)

g_debug_ramped_speed:
 20 ┤                                    ╭─
 15 ┤                              ╭─────╯
 10 ┤                        ╭─────╯      ← Jumps to 10, then ramps up
  5 ┤                   ╭────╯
  0 ┼───────────────────╯
    0s  1s  2s  2.5s 3s  4s  5s
            ↑
        Initializes to 10.0 here
```

### Failure Pattern (Angle Jump)

```
g_cached_encoder_angle:
  6 ┤                           ╭─────────
  5 ┤                       ╱───╯
  4 ┤                   ╱───╯            ← Visible jump/discontinuity
  3 ┤               ╱───╯      ↓ JUMP!
  2 ┤           ╱───╯
  1 ┤───────────╯
  0 ┼────────────────────────────────────
    0s  1s  2s  2.5s 3s  4s  5s
            ↑
        Discontinuity visible here
```

### Failure Pattern (Deceleration)

```
g_measured_velocity_for_control:
 10 ┤
  5 ┤            ╭╮
  2 ┤         ╭──╯╰────────────           ← Decelerates and stops
  0 ┼─────────╯          
 -2 ┤
    0s  1s  2s  2.5s 3s  4s  5s
            ↑
        Motor stops here
```

---

## Test Result Summary

**Date/Time:** _____________

**Build:** _____________

**Test Result:** [ ] PASS  [ ] FAIL

**Motor behavior:** _____________________________________________

**Graphs match expected:** [ ] YES  [ ] NO

**Issues found:** ______________________________________________

**Next steps:** ________________________________________________

---

## Success!

If all checks pass:

🎉 **CONGRATULATIONS!** 🎉

Both fixes are working correctly:
- ✅ Angle continuity achieved (closed-loop kickstart)
- ✅ Velocity continuity achieved (proper initialization)
- ✅ Motor operates smoothly through transition
- ✅ PWM signals clean and continuous

**The kickstart transition problem is SOLVED.**

Motor now:
- Starts reliably from rest
- Transitions seamlessly to CL control
- Accelerates smoothly to target speed
- Operates as if kickstart never happened

**You can now proceed with normal motor operation and testing.**

---

## Reference Documents

For detailed information:
- `KICKSTART_COMPLETE_FIX_SUMMARY.md` - Overview of both fixes
- `KICKSTART_TRANSITION_ANGLE_ANALYSIS.md` - Fix #1 (angle) details
- `TRANSITION_DECELERATION_FIX.md` - Fix #2 (velocity) details
- `KICKSTART_MONITORING_GUIDE.md` - Comprehensive monitoring guide
- `TEST_KICKSTART_FIX.md` - Extended test procedures

---

**READY TO TEST? → Follow checklist from top → Report results**