# FOC Motor Calibration Analysis - Executive Summary

## Problem Identified

The FOC motor control system was experiencing **commutation failure** preventing proper torque generation. Root cause analysis revealed a fundamental flaw in how encoder direction was handled during calibration.

## Root Cause

### Hard-Coded Direction Inversion

The original code applied a **hard-coded direction inversion** at the lowest level of encoder reading:

```cpp
// In AS5048A::fetch_radians()
result = TWO_PI - result;  // HARD-CODED inversion applied to ALL reads
```

### Critical Flaw: Inverted During Calibration

This inversion was applied **during the alignment procedure**, causing the calibration logic to fail:

1. **Alignment tries to measure direction** → applies +90° electrical field
2. **Reads encoder change** → but reading is already inverted!
3. **Calculates direction** → but from inverted data (wrong)
4. **Sets `m_sensor_direction`** → but never uses it (ignored)
5. **Assumes inversion is always correct** → motor-specific, not universal

**Result:** The calibration measured inverted angles to determine if inversion was needed - circular logic that couldn't work correctly.

## The Fix

### Remove Hard-Coded Inversion, Add Dynamic Calibration

The fix implements proper direction detection:

### 1. Read Raw Encoder During Calibration
```cpp
// Disable any inversion during calibration
m_sensor.invert_output(false);

// Read RAW encoder angles
fetch_radians(mech_angle_0);   // At 0° electrical
fetch_radians(mech_angle_90);  // At +90° electrical
```

### 2. Measure Natural Encoder Behavior
```cpp
// Calculate how encoder responded to known field rotation
float mech_delta = mech_angle_90 - mech_angle_0;

// Determine if inversion is needed
bool need_inversion = (mech_delta < 0.0f);
```

**Logic:**
- If encoder **increased** when field advanced +90° → encoder correct, no inversion needed
- If encoder **decreased** when field advanced +90° → encoder backward, apply inversion

### 3. Apply Direction Only After Calibration
```cpp
// Set inversion flag based on measurement
m_sensor.invert_output(need_inversion);

// From now on, all encoder reads will be direction-corrected
// in get_mechanical_phase_angle_radians()
```

### 4. Dynamic Direction Application at Runtime
```cpp
float AS5048A::get_mechanical_phase_angle_radians()
{
    float result;
    fetch_radians(result);  // Get raw angle
    
    // Apply inversion if calibration determined it's needed
    if(m_invert_output)
    {
        result = TWO_PI - result;
    }
    
    return result;
}
```

## Technical Details

### Mechanical vs Electrical Angles

- **Mechanical angle:** 0 to 2π per shaft rotation
- **Electrical angle:** 0 to 2π per pole pair
- **Conversion:** `electrical = mechanical × NUM_POLE_PAIRS`

For the stepper motor (50 pole pairs):
- 1 mechanical rotation = 50 electrical cycles
- +90° electrical = +1.8° mechanical = **0.0314 radians**

### Offset Calculation

During alignment:
1. Apply electrical field at 0° → rotor aligns → read position = `mech_zero`
2. Convert to electrical offset: `offset_elec = mech_zero × 50`
3. Store offset for runtime subtraction

During runtime:
1. Read mechanical angle (direction-corrected): `mech_current`
2. Convert to electrical: `elec_current = mech_current × 50`
3. Subtract offset: `elec_final = elec_current - offset_elec`
4. Use for commutation: `setPhaseVoltage(Vq, Vd, elec_final)`

### Direction Enum Clarification

```cpp
enum class Direction : int8_t
{
    CW  =  1,   // Encoder increases with positive field rotation (no inversion)
    CCW = -1,   // Encoder decreases with positive field rotation (needs inversion)
};
```

**Note:** CW/CCW here refers to encoder behavior relative to field rotation, not absolute motor direction.

## Expected Behavior After Fix

### Alignment Sequence (initFOC)

1. **Disable direction inversion** → read raw encoder values
2. **Apply 0° electrical field** → rotor aligns → read `mech_angle_0`
3. **Apply +90° electrical field** → rotor follows → read `mech_angle_90`
4. **Calculate direction:**
   - `mech_delta = mech_angle_90 - mech_angle_0`
   - If `mech_delta > 0`: encoder correct, no inversion
   - If `mech_delta < 0`: encoder backward, apply inversion
5. **Enable direction inversion** if needed
6. **Re-apply 0° electrical field** → read final position (now direction-corrected)
7. **Calculate offset** in electrical domain
8. **Motor ready** for commutation

### Runtime Operation (loopFOC)

1. **Read encoder** → `fetch_radians()` returns raw angle
2. **Apply direction** → `get_mechanical_phase_angle_radians()` inverts if needed
3. **Convert to electrical** → multiply by pole pairs
4. **Subtract offset** → align to rotor position
5. **Commutate** → use corrected angle for Park/Inverse Park transforms

## Verification Steps

### 1. Check Calibration Results

After alignment, verify debug variables:

```
g_align_mech_delta = ±0.0314 rad   // Expected for 50 pole pairs (+90° elec = +1.8° mech)
g_align_direction_inversion = 0 or 1  // Indicates if inversion was applied
g_align_offset_calculated = [valid range]  // Electrical offset
```

### 2. Test Motor Rotation

With motor enabled:
- **Positive target speed** → smooth forward rotation
- **Negative target speed** → smooth reverse rotation
- **No cogging or fighting** → commutation synchronized with rotor

### 3. Monitor Currents

During operation:
- **`g_amperage_q`** → smooth, proportional to load (torque-producing current)
- **`g_amperage_d`** → near zero (magnetizing current, should be minimal)

### 4. Check Angle Consistency

Manually rotate shaft (motor disabled):
- **`g_encoder_mech_angle_raw`** → should increase smoothly 0 → 2π per rotation
- **`g_encoder_elec_angle_raw`** → should cycle 0 → 2π exactly 50 times per rotation

## Troubleshooting

### If Motor Still Doesn't Work

**Option 1: Verify pole pair count**
- For 200-step stepper: `NUM_POLE_PAIRS = 50` ✓
- For 400-step stepper: `NUM_POLE_PAIRS = 100`
- Wrong pole count → wrong electrical angle scaling

**Option 2: Try inverting direction logic**
- If motor cogs or fights, try changing:
  ```cpp
  bool need_inversion = (mech_delta > 0.0f);  // Opposite sense
  ```
- This accounts for different "forward" definitions

**Option 3: Check alignment measurements**
- Verify `mech_angle_0` and `mech_angle_90` are valid (not 0, not stuck)
- Verify `mech_delta` is reasonable (~0.03 rad)
- If values are bad, alignment failed (encoder not reading correctly)

## Benefits of This Fix

| Before | After |
|--------|-------|
| ❌ Hard-coded inversion | ✅ Dynamic calibration |
| ❌ Direction measured from inverted data | ✅ Direction measured from raw data |
| ❌ `m_sensor_direction` unused | ✅ `m_sensor_direction` properly set and used |
| ❌ Only works for one encoder orientation | ✅ Adapts to any encoder mounting |
| ❌ Calibration logic backwards | ✅ Calibration logic correct |
| ❌ Hard to debug | ✅ Clear diagnostic variables |

## Files Modified

1. **`AS5048A.cpp`** - `fetch_radians()`: Removed hard-coded inversion
2. **`AS5048A.cpp`** - `get_mechanical_phase_angle_radians()`: Apply dynamic direction
3. **`StepperMotor.cpp`** - `alignSensor()`: Proper direction detection from raw data
4. **`StepperMotor.cpp`** - Added debug variable `g_align_direction_inversion`

## Conclusion

The commutation problem was caused by applying encoder direction inversion **before determining which direction to apply**. The fix separates these concerns:

1. ✅ **Calibration phase:** Measure raw encoder behavior to determine direction
2. ✅ **Runtime phase:** Apply direction correction based on calibration results

This proper sequencing allows the motor control system to **dynamically adapt** to any encoder orientation or motor wiring configuration, enabling correct commutation and smooth torque generation.

---

**Status:** Fix implemented and ready for testing  
**Expected Result:** Motor should now commutate correctly with proper direction handling  
**Next Steps:** Flash firmware, run alignment, verify motor operates smoothly in both directions