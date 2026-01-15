# Transition Deceleration Bug Fix

## Problem Identified

After implementing the closed-loop kickstart (which successfully eliminated the angle discontinuity), a **new problem emerged**: the motor would **decelerate or stop** immediately after the kickstart→CL transition at t=2.5s.

### Evidence from Traces

**STM32CubeMonitor traces (around 75.35-75.65s):**
- Motor rotates smoothly during kickstart (71-75s)
- At ~75.4s: Sharp drop in target/speed traces (blue line plummets)
- Encoder angle stops increasing
- Motor stalls

**Saleae logic analyzer (around +0.3s):**
- PWM signals continue but show reduced duty cycle
- Abrupt change in PWM pattern at transition
- Motor stops rotating

## Root Cause Analysis

### The Deceleration Logic Chain

**Problem:** Multiple static variables with conflicting initial values caused unintended deceleration.

#### 1. Target Set Too Low After Kickstart

```cpp
// In initFOC() - BEFORE FIX
m_kickstart_active = true;
kickstartMotor();
m_target = 0.5f;  // ← TOO LOW! Motor just moved at ~1.88 rad/s
m_kickstart_active = false;
```

**Issue:** Motor exits kickstart moving at ~1.88 rad/s electrical (from the ramp), but target is set to only 0.5 rad/s. Controller tries to decelerate!

#### 2. Ramped Speed Static Variable

```cpp
// In update_speed_closed_loop() - BEFORE FIX
static float ramped_speed = 0.5;  // Initialized once at startup

// On first call after kickstart:
if (ramped_speed > target_mechanical_rps)  // 0.5 > 0.5? No
{
    ramped_speed -= ACCEL_RATE * delta_seconds;  // Doesn't execute
}
```

**Issue:** `ramped_speed` starts at 0.5 rad/s. Since `m_target = 0.5`, it doesn't ramp up - it stays at 0.5! Motor should be accelerating but instead tries to match the low target.

#### 3. Ramp Function Timing

```cpp
// In update_ramp() - BEFORE FIX
static uint32_t last_ramp_time = 0;  // Initialized to 0

// First call after kickstart:
uint32_t elapsed_us = micros() - last_ramp_time;  // Huge value (micros() - 0)
if (elapsed_us >= 100000)  // Always true on first call
{
    rps += 10.0f;  // Tries to ramp, but...
    stepper.update_target_rad_per_sec(rps);  // Updates from 0.5 -> 10.5
}
```

**Issue:** Even though `update_ramp()` tries to increase speed, the internal `ramped_speed` in the controller is stuck at 0.5 and won't accelerate until target exceeds it significantly.

### Why Motor Stopped

**Sequence of events:**
1. Kickstart ends with motor at ~1.88 rad/s
2. `m_target` set to 0.5 rad/s
3. `update_speed_closed_loop()` sees target=0.5, ramped_speed=0.5
4. Controller calculates voltage for 0.5 rad/s (very low)
5. Motor decelerates from 1.88 → 0.5 → **stops** (insufficient voltage to overcome friction)
6. `update_ramp()` tries to increase target, but too late - motor already stopped
7. Restarting from zero is hard (static friction)

---

## Solution Implemented

### Fix 1: Set Higher Target After Kickstart

**File:** `CM7/Core/Src/motors/StepperMotor.cpp`
**Function:** `initFOC()` (line ~731)

```cpp
// AFTER FIX
m_kickstart_active = true;
kickstartMotor();

// After kickstart, motor is moving at ~1.88 rad/s electrical
// Set target HIGH to allow ramp to continue acceleration
// The update_ramp() function will ramp from 0.5 -> 80 rad/s
m_target = 10.0f;  // Start ramping toward higher speed immediately

m_kickstart_active = false;
```

**Benefit:** Controller sees target=10 rad/s, which is higher than current speed → accelerates instead of decelerating.

### Fix 2: Initialize Ramped Speed After Kickstart

**File:** `CM7/Core/Src/motors/StepperMotor.cpp`
**Function:** `update_speed_closed_loop()` (line ~2816)

```cpp
// Added new static flag and kickstart completion signal
static float ramped_speed = 2.0;  // Changed from 0.5 to 2.0 rad/s
static bool post_kickstart_initialized = false;

extern volatile bool g_kickstart_just_completed;
if (g_kickstart_just_completed && !post_kickstart_initialized)
{
    ramped_speed = target_mechanical_rps;  // Initialize to current target (10.0)
    post_kickstart_initialized = true;
    g_kickstart_just_completed = false;
}
```

**Benefit:** After kickstart, `ramped_speed` immediately jumps to 10.0 rad/s (matching target), allowing smooth acceleration.

### Fix 3: Add Kickstart Completion Flag

**File:** `CM7/Core/Src/motors/StepperMotor.cpp`

**Added global variable:**
```cpp
volatile bool g_kickstart_just_completed(false);
```

**Set at end of kickstartMotor():**
```cpp
// Signal that kickstart just completed (for ramped_speed initialization)
extern volatile bool g_kickstart_just_completed;
g_kickstart_just_completed = true;
```

**Benefit:** Provides explicit signal to controller that kickstart just finished, allowing one-time initialization of ramped_speed.

### Fix 4: Initialize Ramp Timing Properly

**File:** `CM7/Core/Src/main_cpp.cpp`
**Function:** `update_ramp()` (line ~689)

```cpp
static bool ramp_initialized = false;

if (!ramp_initialized)
{
    last_ramp_time = micros();  // Initialize to current time (not 0)
    ramp_initialized = true;
    // Keep rps at initial value (0.5) - will ramp up from there
}
```

**Benefit:** Prevents huge `elapsed_us` on first call, ensuring clean ramp timing.

---

## How the Fix Works

### New Transition Sequence

**Before fix:**
```
t=2.5s: Kickstart ends
        m_target = 0.5 rad/s
        ramped_speed = 0.5 rad/s (static, unchanged)
        Controller: "Target is 0.5, speed is 0.5, maintain"
        Voltage: LOW (for 0.5 rad/s)
        Result: Motor decelerates and stops ❌
```

**After fix:**
```
t=2.5s: Kickstart ends
        m_target = 10.0 rad/s
        g_kickstart_just_completed = true
        ramped_speed = 10.0 rad/s (initialized to target)
        Controller: "Target is 10, speed is 10, but actual velocity ~2 rad/s"
        Velocity error: 10 - 2 = +8 rad/s (positive = need to accelerate)
        Voltage: INCREASES via PI controller
        Result: Motor accelerates smoothly ✅
```

### Key Improvements

1. **Target matches intention:** 10 rad/s instead of 0.5 rad/s
2. **Ramped speed synced:** Starts at target value, not stuck at 0.5
3. **Clear handoff signal:** `g_kickstart_just_completed` flag
4. **Proper timing:** Ramp timing initialized correctly

---

## Variables to Monitor

### Critical for Debugging This Issue

**During transition (t=2.5s):**
```
g_kickstart_just_completed:     true (briefly)
m_target:                        10.0 rad/s ← Should be 10, not 0.5!
g_debug_ramped_speed:           10.0 rad/s ← Should match target
g_debug_actual_target_rps:      10.0 rad/s ← Should be 10
g_measured_velocity_for_control: ~2.0 rad/s (actual motor speed)
g_velocity_error_for_debug:     +8.0 rad/s (positive = accelerate)
g_voltage_q:                     Increasing (5V → 8V → 12V)
```

**After transition (t>2.5s):**
```
g_debug_ramped_speed:           Ramping 10 → 20 → 30 → ... → 80 rad/s
g_measured_velocity_for_control: Following ramped_speed (accelerating)
g_voltage_q:                     Varies based on velocity error (8-15V typical)
Motor behavior:                  Smooth continuous acceleration ✅
```

### Failure Indicators

❌ `m_target = 0.5` after kickstart (old code still active)
❌ `g_debug_ramped_speed` decreasing or staying at 0.5
❌ `g_voltage_q` decreasing instead of increasing
❌ `g_velocity_error_for_debug` negative (trying to slow down)
❌ Motor decelerates after transition

---

## Testing Procedure

### Test 1: Verify Target Initialization

**Monitor:** `m_target`
**Expected:** After kickstart ends (t=2.5s), should jump to 10.0 rad/s
**Pass:** `m_target = 10.0`
**Fail:** `m_target = 0.5` (old code)

### Test 2: Verify Ramped Speed Sync

**Monitor:** `g_debug_ramped_speed`
**Expected:** Immediately after kickstart, should be 10.0 rad/s (matching target)
**Pass:** `ramped_speed = 10.0` at t=2.5s
**Fail:** `ramped_speed = 0.5` or decreasing

### Test 3: Verify Acceleration

**Monitor:** `g_voltage_q` and `g_measured_velocity_for_control`
**Expected:** Both should increase after transition
**Pass:** Voltage increases, velocity increases, motor accelerates
**Fail:** Voltage decreases, velocity decreases, motor stops

### Test 4: Full Run

**Procedure:**
1. Power on system
2. Motor runs through alignment
3. Kickstart executes (0-2.5s)
4. Transition occurs at 2.5s
5. Motor continues accelerating to ~20-40 rad/s

**Expected result:** Smooth continuous acceleration with no deceleration or stopping

---

## Code Changes Summary

### Files Modified

1. **`CM7/Core/Src/motors/StepperMotor.cpp`**
   - Line 107: Added `g_kickstart_just_completed` variable
   - Line 731-738: Changed `m_target` from 0.5 to 10.0
   - Line 1509: Set `g_kickstart_just_completed = true`
   - Line 2816-2829: Added ramped_speed initialization after kickstart

2. **`CM7/Core/Src/main_cpp.cpp`**
   - Line 680: Added `ramp_initialized` flag
   - Line 695-704: Initialize `last_ramp_time` on first call

### New Variables

```cpp
volatile bool g_kickstart_just_completed(false);  // Signals kickstart completion
```

---

## Interaction with Other Fixes

### Works Together With

1. **Closed-Loop Kickstart** (previous fix)
   - Eliminated angle discontinuity
   - This fix eliminates velocity discontinuity
   - Together: complete smooth transition

2. **Kalman Filter**
   - Provides accurate velocity feedback
   - PI controller uses this to calculate correct voltage

3. **Ramp Function**
   - Continues ramping from 10 → 80 rad/s after transition
   - Now properly timed to avoid glitches

---

## Why This Happened

### Design Assumption Violated

**Original design assumed:**
- Motor starts from rest
- `ramped_speed` begins at 0.5 rad/s
- Target ramps up gradually
- Controller smoothly accelerates motor

**Kickstart changed this:**
- Motor doesn't start from rest (moving at ~2 rad/s after kickstart)
- Static variables retained old values
- Target was reset to 0.5 (below actual speed)
- Controller tried to decelerate → motor stopped

**Lesson:** Static variables in control loops need careful initialization when operating mode changes.

---

## Expected Outcome

With both fixes applied:

1. ✅ **Angle continuity** (closed-loop kickstart)
2. ✅ **Velocity continuity** (proper target/ramp initialization)
3. ✅ **Smooth PWM** (no discontinuity)
4. ✅ **Motor acceleration** (no stopping)
5. ✅ **Reaches target speed** (e.g., 40-80 rad/s)

**Result:** Motor executes kickstart, transitions smoothly to CL control, and accelerates continuously to target speed without any interruption.

---

## Status

**Implementation:** ✅ Complete
**Testing:** Ready for validation
**Confidence:** High (addresses both angle and velocity discontinuity)

---

## References

- Original issue: PWM disruption at transition (Saleae plot)
- First fix: Closed-loop kickstart (KICKSTART_TRANSITION_ANGLE_ANALYSIS.md)
- This fix: Velocity/target initialization (prevents deceleration)
- Related: KICKSTART_MONITORING_GUIDE.md (updated with new variables)