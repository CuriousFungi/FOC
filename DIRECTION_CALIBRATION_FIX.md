# Direction Calibration Fix - Removing Hard-Coded Inversion

## Problem Statement

The original code had a **hard-coded direction inversion** applied at the lowest level in `AS5048A::fetch_radians()`:

```cpp
// HARD-CODED FIX: Invert encoder direction to match motor rotation
result = TWO_PI - result;
```

This approach had several critical flaws:

1. **Direction was inverted during calibration** - The alignment routine tried to measure encoder direction, but the measurements were already inverted, making the direction detection meaningless.

2. **`m_sensor_direction` was unused** - The code calculated which direction the encoder moved during alignment but never actually used this information.

3. **No dynamic adaptation** - Different motors or encoder mounting orientations require different direction settings, but the hard-coded inversion prevented adaptation.

4. **Calibration logic was backwards** - Direction should be determined FROM calibration, not applied DURING calibration.

## Solution Overview

The fix implements proper direction calibration:

1. **Remove hard-coded inversion** from `fetch_radians()` - returns raw encoder angle
2. **Measure raw encoder behavior** during alignment (without any inversion)
3. **Determine direction dynamically** based on how encoder responds to known stator field rotation
4. **Apply direction correction** in `get_mechanical_phase_angle_radians()` AFTER calibration

## Code Changes

### 1. AS5048A::fetch_radians() - Remove Hard-Coded Inversion

**File:** `CM7\Core\Src\sensors\AS5048A.cpp`

**Before:**
```cpp
result = (static_cast<float>(count_14_bit) * TWO_PI) / AS5048_MAX;

// HARD-CODED FIX: Invert encoder direction to match motor rotation
result = TWO_PI - result;
```

**After:**
```cpp
result = (static_cast<float>(count_14_bit) * TWO_PI) / AS5048_MAX;

// NO INVERSION HERE - return raw encoder angle
// Direction will be determined during calibration and applied at higher level
// in get_mechanical_phase_angle_radians() after calibration is complete
```

### 2. AS5048A::get_mechanical_phase_angle_radians() - Apply Dynamic Direction

**File:** `CM7\Core\Src\sensors\AS5048A.cpp`

**Before:**
```cpp
float AS5048A::get_mechanical_phase_angle_radians()
{
    // Direction inversion is hard-coded in fetch_radians()
    // so we can directly return the result
    float result;
    if(fetch_radians(result))
    {
        return result;
    }
    else
    {
        return 0.0f;
    }
}
```

**After:**
```cpp
float AS5048A::get_mechanical_phase_angle_radians()
{
    // Get raw encoder angle
    float result;
    if(!fetch_radians(result))
    {
        return 0.0f;
    }
    
    // Apply direction inversion if needed (determined during calibration)
    // m_invert_output is set by invert_output() method based on alignment results
    if(m_invert_output)
    {
        result = TWO_PI - result;
    }
    
    return result;
}
```

### 3. StepperMotor::alignSensor() - Dynamic Direction Detection

**File:** `CM7\Core\Src\motors\StepperMotor.cpp`

**Key Changes:**

#### A. Disable direction during calibration:
```cpp
// CRITICAL: Disable direction inversion during calibration
// We need RAW encoder readings to properly determine which direction it moves
m_sensor.invert_output(false);
```

#### B. Measure raw encoder response to known field rotation:
```cpp
// We applied +90° electrical field rotation (positive direction in stator frame)
// The rotor should follow this field rotation
// Measure how the RAW encoder responded (before any inversion)

float mech_delta = mech_angle_90 - mech_angle_0;

// Handle wraparound: if delta is large negative, encoder wrapped through 0
if (mech_delta < -MY_PI)
{
    mech_delta += TWO_PI;
}
else if (mech_delta > MY_PI)
{
    mech_delta -= TWO_PI;
}
```

#### C. Determine if inversion is needed:
```cpp
// FOC convention: positive electrical field rotation → positive mechanical angle increase
// If encoder DECREASED when we advanced field +90°, we need to invert it
bool need_inversion = (mech_delta < 0.0f);
```

#### D. Set direction enum (diagnostic):
```cpp
// Direction::CW (+1):   Encoder naturally increases with positive field rotation
//                       No inversion needed, use encoder values as-is
//
// Direction::CCW (-1):  Encoder naturally decreases with positive field rotation
//                       Inversion needed (TWO_PI - angle) to match FOC convention

if (mech_delta > 0.0f)
{
    m_sensor_direction = Direction::CW;   // Encoder naturally aligned with FOC convention
}
else
{
    m_sensor_direction = Direction::CCW;  // Encoder counts backward, needs inversion
}
```

#### E. Apply direction correction for all future reads:
```cpp
// CRITICAL: Apply direction inversion based on calibration result
// This will be used for ALL subsequent encoder reads (after calibration completes)
// The inversion is applied in get_mechanical_phase_angle_radians()
// From this point forward, all angle reads will be corrected to match FOC convention
m_sensor.invert_output(need_inversion);
```

## Direction Detection Logic

### Physical Interpretation

During alignment, the code performs the following sequence:

1. **Apply electrical field at 0°** → rotor aligns → read mechanical angle = `mech_angle_0`
2. **Apply electrical field at +90°** → rotor rotates forward → read mechanical angle = `mech_angle_90`
3. **Calculate change:** `mech_delta = mech_angle_90 - mech_angle_0`

### Interpreting the Result

| Condition | Physical Meaning | Direction Setting | Inversion Needed? |
|-----------|------------------|-------------------|-------------------|
| `mech_delta > 0` | Encoder **increased** when field rotated forward | `Direction::CW` (+1) | **NO** - encoder naturally matches FOC convention |
| `mech_delta < 0` | Encoder **decreased** when field rotated forward | `Direction::CCW` (-1) | **YES** - apply `TWO_PI - angle` to flip direction |

### Why This Works

- **FOC convention:** Positive electrical angle should produce positive torque in the "forward" direction
- **Encoder consistency:** The encoder must read increasing values for forward rotation
- **If encoder is backwards:** Physical encoder counts down during forward rotation → we invert it → software sees it counting up
- **If encoder is correct:** Physical encoder counts up during forward rotation → no inversion → software sees raw values

## Direction Enum Clarification

```cpp
enum class Direction : int8_t
{
    CW      =  1,  // Encoder naturally increases with positive field rotation (no inversion)
    CCW     = -1,  // Encoder naturally decreases with positive field rotation (needs inversion)
    UNKNOWN =  0   // Not yet calibrated
};
```

**Note:** "CW" and "CCW" here refer to **encoder behavior relative to field rotation**, not absolute motor shaft direction. The motor's actual rotation direction depends on load, mounting, and phase wiring.

## Debug Variables

New debug variable added to monitor calibration:

```cpp
volatile float g_align_direction_inversion;  // 1.0 = inversion applied, 0.0 = no inversion
```

Existing debug variables that help verify correct operation:

- `g_align_mech_angle_0` - Mechanical angle at electrical 0°
- `g_align_mech_angle_90` - Mechanical angle at electrical +90°
- `g_align_mech_delta` - Change in mechanical angle (should be ±0.0314 rad for 50 pole pairs)
- `g_align_mech_zero` - Final mechanical angle used for offset calculation
- `g_align_offset_calculated` - Calculated electrical offset

## Expected Behavior After Fix

### During Alignment (initFOC):

1. **First 5 seconds:** Motor initializes, SPI buffers populate
2. **Alignment sequence:**
   - Apply 0° electrical → wait → read raw encoder
   - Apply +90° electrical → wait → read raw encoder
   - Calculate `mech_delta` to determine direction
   - Set `m_sensor.invert_output()` based on measurement
   - Re-apply 0° electrical → read encoder (now direction-corrected)
   - Calculate offset
3. **Result:** `m_sensor_direction` set, direction inversion flag set, offset calculated

### During Runtime (loopFOC):

1. **Encoder read:** `fetch_radians()` returns raw angle
2. **Direction correction:** `get_mechanical_phase_angle_radians()` applies inversion if needed
3. **Electrical conversion:** `get_electric_angle_radians()` converts to electrical and subtracts offset
4. **Commutation:** Corrected electrical angle used for Park/Inverse Park transforms

## Testing and Verification

### 1. Check Direction Detection

After alignment completes, check these debug variables:

```cpp
g_align_mech_delta       // Should be ±0.0314 rad (±1.8°) for 50 pole pairs
g_align_direction_inversion  // 0.0 or 1.0 - indicates if inversion was applied
```

**Expected values:**
- For 50 pole pairs, +90° electrical = +1.8° mechanical = **0.0314 radians**
- If `mech_delta ≈ +0.031`: encoder correct, no inversion needed
- If `mech_delta ≈ -0.031`: encoder backward, inversion needed

### 2. Manual Rotation Test

With motor disabled, manually rotate shaft and observe:

```cpp
g_encoder_mech_angle_raw  // Should increase smoothly from 0 to 2π for one rotation
```

- **If increases:** Direction correction is working
- **If decreases:** Direction correction is inverted (should not happen after this fix)

### 3. Motor Direction Test

Run motor in velocity control mode and verify:

- **Positive target speed** → motor rotates forward consistently
- **Negative target speed** → motor rotates backward consistently
- **No cogging or fighting** → commutation is synchronized with rotor position

### 4. Current Measurement Verification

During operation, check:

```cpp
g_amperage_q  // Should be smooth and proportional to load
g_amperage_d  // Should be near zero (no field weakening)
```

- **If Iq is noisy or oscillating:** Direction may still be wrong
- **If Id is large:** Commutation angle may be offset by 90°

## Troubleshooting

### Problem: Motor still doesn't commutate correctly

**Check 1: Is calibration completing successfully?**
- Verify `mech_angle_0` and `mech_angle_90` are valid (not 0xDEAD, not stuck at 0)
- Verify `mech_delta` is reasonable magnitude (~0.03 rad for 50 pole pairs)

**Check 2: Is NUM_POLE_PAIRS correct?**
- For a 200-step stepper motor: `NUM_POLE_PAIRS = 50`
- If motor has different step count, adjust accordingly
- Wrong pole pair count causes wrong electrical angle scaling

**Check 3: Is direction detection logic correct?**
- If motor rotates but with cogging: Try **inverting the direction logic**
- Change `bool need_inversion = (mech_delta < 0.0f);` to `bool need_inversion = (mech_delta > 0.0f);`
- This accounts for different interpretations of "forward" rotation

**Check 4: Is offset calculation correct?**
- `g_align_offset_calculated` should be in range [0, 2π × NUM_POLE_PAIRS]
- If offset is outside this range, wraparound may not be working correctly

### Problem: Direction changes randomly between runs

**Cause:** Encoder position during alignment varies
**Solution:** This is expected - direction should be **consistent relative to encoder position**, not absolute motor shaft direction

### Problem: Motor works in one direction but not the other

**Cause:** PWM driver or phase wiring issue, not direction calibration
**Solution:** Check driver enable signals, verify all three phase outputs are working

## Comparison: Before vs After

| Aspect | Before (Hard-Coded) | After (Dynamic Calibration) |
|--------|---------------------|----------------------------|
| **Direction Inversion** | Always applied | Applied only if encoder counts backward |
| **Calibration Logic** | Measured already-inverted angles | Measures raw angles first |
| **`m_sensor_direction`** | Calculated but unused | Properly set and used |
| **Adaptability** | Fixed for one motor/encoder setup | Adapts to any encoder orientation |
| **Debugging** | Hard to tell if inversion is correct | Clear indication via `g_align_direction_inversion` |
| **Motor Compatibility** | May work for some motors, not others | Should work for any motor |

## Future Improvements

1. **Add sanity checks on `mech_delta`:**
   - Verify magnitude is reasonable (should be ~1-3° mechanical)
   - If too small, alignment may have failed
   - If too large, encoder may have multi-turn rollover issue

2. **Store calibration in non-volatile memory:**
   - Save `need_inversion` flag to FLASH
   - Skip alignment on subsequent power-ups
   - Force re-calibration on user command or after timeout

3. **Add visual feedback:**
   - LED blink pattern to indicate direction detected
   - Serial output showing calibration results

4. **Support for absolute encoders:**
   - Multi-turn encoders don't need alignment (know absolute position)
   - Code could detect and skip alignment if encoder is absolute

## Conclusion

This fix removes the problematic hard-coded direction inversion and replaces it with **proper dynamic calibration**. The encoder direction is now:

1. ✅ Measured during alignment (from raw encoder data)
2. ✅ Stored in the sensor object (`m_invert_output` flag)
3. ✅ Applied consistently during runtime (in `get_mechanical_phase_angle_radians()`)
4. ✅ Diagnostic information available (`m_sensor_direction` enum)

The motor should now properly commutate regardless of encoder mounting orientation or motor wiring configuration.