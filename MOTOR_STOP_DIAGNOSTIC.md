# Motor Stop Diagnostic - 7 Second Cycle Issue

## Problem Description

The motor exhibits periodic stopping behavior with the following pattern:
- Motor runs for ~7 seconds
- Motor stops for ~7 seconds  
- Pattern repeats indefinitely

Observable in traces D5 and D6 which show gaps every 7 seconds.

---

## Root Cause Analysis

### Suspected Issue #1: `rps` Reset in `foc_iteration()`

**Location:** `project3/CM7/Core/Src/main_cpp.cpp` line 790

**Code:**
```cpp
void foc_iteration(void)
{
    if(is_foc_initialized)
    {
        winding_currents result = update_amperage();
        stepper.loopFOC(result.winding_amperage_a, result.winding_amperage_b);
        update_ramp();
    }
    else
    {
        rps = 0.0f;  // ← THIS RESETS TARGET SPEED TO ZERO
    }
}
```

**Problem:**
- When `is_foc_initialized` becomes `false`, `rps` is reset to 0
- This stops the motor
- Motor remains stopped until something triggers re-initialization

**Fix Applied:**
```cpp
else
{
    // DISABLED: This was causing periodic motor stops
    // rps = 0.0f;  // Commented out
}
```

---

### Suspected Issue #2: `is_foc_initialized` Becoming False

**Possible Causes:**

1. **System reset/watchdog trigger**
   - Check: `g_system_reset_count` or similar
   - Look for watchdog timeout

2. **initFOC() being called repeatedly**
   - Check: Is `cpp_main()` being called multiple times?
   - Look for: Jump to reset vector

3. **initFOC() returning false**
   - Happens when `alignSensor()` fails
   - Encoder error bits set
   - Encoder communication failure

4. **Memory corruption**
   - Static variable `is_foc_initialized` being overwritten
   - Stack overflow
   - DMA buffer overrun

---

## Diagnostic Variables to Monitor

### Critical Variables

| Variable | Location | What to Check |
|----------|----------|---------------|
| `is_foc_initialized` | main_cpp.cpp | Should stay `true`, check if goes to `false` |
| `rps` | main_cpp.cpp | Should ramp 0→80, check if resets to 0 |
| `g_cmd_rps` | main_cpp.cpp | Should match `rps`, check for discontinuities |
| `m_target` | StepperMotor | Should match `rps`, check if goes to 0 |
| `m_enabled` | StepperMotor | Should stay `true`, check if goes to `false` |
| `m_motor_status` | StepperMotor | Should be `READY`, check for `CALIBRATION_FAILED` |

### Timing Variables

| Variable | What It Shows |
|----------|---------------|
| `g_ramp_update_count` | How many times ramp has updated (should increment) |
| `g_ramp_elapsed_us` | Time between ramp updates (should be ~100ms) |
| `g_update_speed_cl_call_count` | How many times control loop called |
| `g_loopfoc_update_count` | How many times FOC loop called |

---

## Step-by-Step Diagnosis

### Step 1: Verify Fix is Applied

Check that line 791 in `main_cpp.cpp` is commented out:
```cpp
// rps = 0.0f;  // Should be commented
```

### Step 2: Monitor `is_foc_initialized`

**How:**
- Add to STM Studio variable list
- Watch during motor operation
- Check if it ever becomes `false` (0)

**Expected:** Should stay `true` (1) continuously

**If it becomes false:**
- Something is calling `initFOC()` again
- Or memory corruption
- Or system reset

### Step 3: Monitor `rps` Variable

**How:**
- Plot `rps` vs time
- Plot `g_cmd_rps` vs time (should match)
- Plot `m_target` vs time (should match)

**Expected:** 
- Ramps from 0.5 → 80 rad/s
- Stays at 80 rad/s
- Never drops to 0 after ramp completes

**If it drops to 0:**
- Check previous step - `is_foc_initialized` became false
- Or `update_ramp()` not being called

### Step 4: Check Encoder Communication

**Variables:**
- `g_as5048_error_count` - Should stay 0 or very low
- `g_as5048_spi_timeout_count` - Should stay 0
- `g_cached_encoder_angle` - Should continuously update

**If errors increasing:**
- Encoder losing communication
- SPI timing issues
- Causes `alignSensor()` to fail
- Makes `initFOC()` return `false`

### Step 5: Check Motor Enable Status

**Variables:**
- `m_enabled` - Should stay `true`
- `m_motor_status` - Should be `READY` (not `CALIBRATION_FAILED`)

**If disabled:**
- Something called `disable()`
- Or `initFOC()` failed and called `disable()`

---

## Timeline Analysis

### 7-Second Cycle Breakdown

```
Hypothesis 1: Motor runs, then is_foc_initialized goes false
  0s : Motor starts
  0-5s : Accumulated angle mode
  5-7s : Blending mode (2 seconds)
  7s : Full encoder mode
  7s : SOMETHING GOES WRONG
       - is_foc_initialized = false
       - rps = 0.0f (was causing this)
       - Motor stops
  7-14s: Motor stopped (7 seconds)
  14s: System somehow recovers
       - initFOC() called again?
       - is_foc_initialized = true again
  14s: Cycle repeats
```

**Why 7 seconds running?**
- 5 seconds accumulated mode
- 2 seconds blending
- Total = 7 seconds until fully in encoder mode
- Then something fails

**Why 7 seconds stopped?**
- Unknown - need to identify what triggers restart

---

## Additional Checks

### Check for System Resets

Monitor these (if available):
- `__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)` - Independent watchdog reset
- `__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)` - Window watchdog reset
- `__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)` - Low power reset
- `__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)` - Software reset

### Check Stack Usage

- Monitor stack pointer
- Check for stack overflow
- Look for memory corruption patterns

### Check DMA Status

- `HAL_DMA_GetState()` for SPI DMA
- Check for DMA errors
- Verify DMA buffers not corrupted

---

## Quick Test Procedures

### Test 1: Verify rps Fix

1. Flash firmware with commented-out `rps = 0.0f`
2. Monitor `rps` variable
3. Run motor for 30 seconds
4. **Expected:** rps ramps up and stays high
5. **If still stops:** Fix didn't work, issue is elsewhere

### Test 2: Force is_foc_initialized True

Add this in `foc_iteration()`:
```cpp
void foc_iteration(void)
{
    is_foc_initialized = true;  // FORCE IT TRUE for testing
    
    if(is_foc_initialized)
    {
        // ... rest of code
    }
}
```

**If this fixes it:** Something is setting `is_foc_initialized` to false

### Test 3: Monitor Call Counts

Watch these counters during stop period:
- `g_update_speed_cl_call_count` - Should keep incrementing
- `g_loopfoc_update_count` - Should keep incrementing
- `g_ramp_update_count` - Should increment every 100ms

**If they stop incrementing:** Functions not being called

**If they keep incrementing:** Functions running but motor still stopped

---

## Expected Results After Fix

### Success Criteria

- ✅ Motor ramps from 0.5 → 80 rad/s continuously
- ✅ No periodic stops
- ✅ `rps` stays at 80 rad/s after ramp completes
- ✅ `is_foc_initialized` stays `true` continuously
- ✅ Encoder blend completes smoothly at 5-7s mark
- ✅ Motor continues running indefinitely

### If Still Not Working

The issue is **NOT** the `rps = 0.0f` line.

**Next steps:**
1. Identify what makes `is_foc_initialized` become `false`
2. Check for system resets
3. Check for memory corruption
4. Check encoder communication
5. Look for exception handlers being triggered

---

## Code Changes Summary

### Change #1: Comment Out rps Reset

**File:** `project3/CM7/Core/Src/main_cpp.cpp`
**Line:** ~790

**Before:**
```cpp
else
{
    rps = 0.0f;
}
```

**After:**
```cpp
else
{
    // DISABLED: This was causing periodic motor stops
    // rps = 0.0f;
}
```

---

## Variables for STM Studio

Add these to monitor the issue:

```
is_foc_initialized
rps
g_cmd_rps
g_ramp_update_count
g_ramp_elapsed_us
m_target
m_enabled
m_motor_status
g_update_speed_cl_call_count
g_loopfoc_update_count
g_as5048_error_count
g_cached_encoder_angle
g_encoder_blend_factor
```

---

## Troubleshooting Decision Tree

```
Motor stops every 7 seconds?
│
├─ Is rps going to 0?
│  ├─ YES → Check is_foc_initialized
│  │        ├─ Stays true → Something else setting rps=0
│  │        └─ Goes false → Find what's setting it false
│  │
│  └─ NO → Check m_enabled
│           ├─ Goes false → Find what's calling disable()
│           └─ Stays true → Check voltage commands
│
├─ Are control functions still being called?
│  ├─ NO → Interrupt stopped or function not called
│  └─ YES → Continue investigating
│
└─ Is encoder working?
   ├─ NO → Fix encoder communication
   └─ YES → Check voltage output to motor
```

---

## Next Actions

1. **Compile and flash** firmware with `rps = 0.0f` commented out
2. **Monitor** `is_foc_initialized` and `rps` continuously
3. **Observe** if 7-second stops still occur
4. **Collect data** during stop period:
   - What changed just before stop?
   - What variables went to unexpected values?
   - Are interrupts still firing?
5. **Report findings** for further diagnosis

---

**Status:** Fix applied (rps reset commented out)
**Next:** Test and verify if issue persists