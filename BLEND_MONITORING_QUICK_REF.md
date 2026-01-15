# Angle Blending - Quick Monitoring Reference

## 🎯 Critical Variables to Watch

| Variable | Normal Range | What It Tells You |
|----------|--------------|-------------------|
| `g_encoder_blend_factor` | 0.0 → 1.0 over 2s | Blend progress (0=accumulated, 1=encoder) |
| `g_blended_result` | 0 to 2π | Final angle used for commutation |
| `g_accumulated_angle_pre_blend` | 0 to 2π | Input from accumulated calculation |
| `g_encoder_angle_pre_blend` | 0 to 2π | Input from encoder sensor |
| `g_angle_difference_accum_vs_encoder` | -0.5 to +0.5 rad | Angular error between sources |
| `g_accumulated_angle_source` | 2.0 or 3.0 | Mode: 2=accumulated, 3=encoder |

---

## ⏱️ Expected Timeline

```
Time    | Blend Factor | Mode Source | What's Happening
--------|--------------|-------------|----------------------------------
0.0s    | -            | 2.0         | Motor startup, accumulated mode
1.0s    | -            | 2.0         | Accumulating angle from target
2.0s    | -            | 2.0         | Still in accumulated mode
3.0s    | -            | 2.0         | Motor spinning up
4.0s    | -            | 2.0         | Approaching transition
4.9s    | -            | 2.0         | Last moment of accumulated
5.0s    | 0.0          | 3.0         | 🔄 TRANSITION BEGINS
5.5s    | 0.25         | 3.0         | 25% blended to encoder
6.0s    | 0.50         | 3.0         | 50% blended (halfway)
6.5s    | 0.75         | 3.0         | 75% blended
7.0s    | 1.00         | 3.0         | ✅ FULLY USING ENCODER
8.0s+   | 1.00         | 3.0         | Pure encoder mode
```

---

## 📊 STM Studio Setup

### Variable List (Copy/Paste)
```
g_encoder_blend_factor
g_blended_result
g_accumulated_angle_pre_blend
g_encoder_angle_pre_blend
g_angle_difference_accum_vs_encoder
g_accumulated_angle_source
g_cached_encoder_angle
g_measured_velocity_for_control
```

### Plot Configuration

**Plot 1: Blend Progress**
- X: Time
- Y1: `g_encoder_blend_factor` (should ramp 0→1)
- Y2: `g_accumulated_angle_source` (should jump 2→3 at t=5s)

**Plot 2: Angle Transition**
- X: Time
- Y1: `g_accumulated_angle_pre_blend` (blue)
- Y2: `g_encoder_angle_pre_blend` (red)
- Y3: `g_blended_result` (green - should be smooth curve)

**Plot 3: Angular Error**
- X: Time
- Y: `g_angle_difference_accum_vs_encoder` (should converge to ~0)

---

## ✅ Success Indicators

### Good Transition (Everything Working)
- ✅ `g_encoder_blend_factor` ramps smoothly 0.0 → 1.0
- ✅ `g_blended_result` is continuous (no jumps)
- ✅ `g_angle_difference_accum_vs_encoder` < 0.5 rad at t=5s
- ✅ Motor maintains constant velocity through t=5-7s
- ✅ No increase in `g_velocity_error_for_debug`
- ✅ `g_m_voltage_q_in_controller` remains steady

### Transition Timeline
```
4.99s: accumulated_angle_source = 2.0 (accumulated)
5.00s: accumulated_angle_source = 3.0 (encoder)
5.00s: encoder_blend_factor = 0.00
5.50s: encoder_blend_factor = 0.25
6.00s: encoder_blend_factor = 0.50
6.50s: encoder_blend_factor = 0.75
7.00s: encoder_blend_factor = 1.00
```

---

## ⚠️ Problem Indicators

### PROBLEM: Motor Stalls at t=5s
**Symptoms:**
- Velocity drops suddenly
- Large spike in `g_velocity_error_for_debug`
- `g_m_voltage_q_in_controller` saturates

**Check:**
- `g_angle_difference_accum_vs_encoder` at t=4.99s
  - If > 1.0 rad: Accumulated drifted too much
  - **Action:** Increase SYNC_GAIN in accumulated mode

### PROBLEM: Vibration During Blend (t=5-7s)
**Symptoms:**
- Oscillation in velocity
- `g_blended_result` has small discontinuities
- Motor audibly rough

**Check:**
- `g_blended_result` for jumps > 0.1 rad between samples
  - **Action:** Increase BLEND_DURATION_US to 3000000 (3 seconds)

### PROBLEM: Blend Never Completes
**Symptoms:**
- `g_encoder_blend_factor` stuck at 0.5 or similar
- Never reaches 1.0
- `g_accumulated_angle_source` oscillates between 2.0 and 3.0

**Check:**
- `g_encoder_blend_start_time` value
  - If changing: Mode switching back and forth
  - **Action:** Add hysteresis to mode transition logic

### PROBLEM: Large Angle Jump at t=5s
**Symptoms:**
- `g_blended_result` jumps > 1.0 rad instantly
- Despite blend_factor = 0.0

**Check:**
- Angle unwrapping in blend calculation
  - `angle_diff` should be in range [-π, +π]
  - **Action:** Verify MY_PI and TWO_PI constants

---

## 🔧 Quick Tuning Guide

### Blend Too Fast (Motor Can't Follow)
```cpp
// Line ~3373 in StepperMotor.cpp
const uint32_t BLEND_DURATION_US = 3000000;  // Change from 2s to 3s
```

### Blend Too Slow (Unnecessary Delay)
```cpp
const uint32_t BLEND_DURATION_US = 1500000;  // Change from 2s to 1.5s
```

### Large Angular Errors Before Transition
```cpp
// Line ~3313 in StepperMotor.cpp
const float SYNC_GAIN = 0.005f;  // Increase from 0.002 to 0.005
```

---

## 📈 Benchmark Values

### Typical Good Run (10 rad/s target)
```
Time   | blend_factor | angle_diff | velocity  | voltage_q
-------|--------------|------------|-----------|----------
4.9s   | -            | 0.15 rad   | 9.8 rad/s | 8.2 V
5.0s   | 0.00         | 0.15 rad   | 10.1 rad/s| 8.3 V
5.5s   | 0.25         | 0.11 rad   | 10.0 rad/s| 8.2 V
6.0s   | 0.50         | 0.08 rad   | 9.9 rad/s | 8.3 V
6.5s   | 0.75         | 0.04 rad   | 10.1 rad/s| 8.2 V
7.0s   | 1.00         | 0.02 rad   | 10.0 rad/s| 8.3 V
```

**Key Observations:**
- Velocity stays within ±0.2 rad/s of target
- Angular error decreases during blend
- Voltage remains stable (no saturation)

---

## 🎬 Test Procedure

### 5-Minute Validation Test

**Step 1:** Start motor at 10 rad/s
**Step 2:** Monitor variables for 10 seconds
**Step 3:** Check blend at t=5s:
- [ ] blend_factor ramps smoothly
- [ ] No velocity drop
- [ ] No voltage spike
- [ ] angle_diff decreases

**Step 4:** Stop motor, wait 5s, repeat at 15 rad/s
**Step 5:** Repeat at 20 rad/s
**Step 6:** ✅ If all pass → Blend working correctly

---

## 🐛 Debug Commands

### Via Serial/USB
```
# Check if blend is active
print g_encoder_blend_factor

# Check mode
print g_accumulated_angle_source

# Check angular error
print g_angle_difference_accum_vs_encoder

# Check blend timestamp (should be ~5s after start)
print g_encoder_blend_start_time
```

---

## 📞 Quick Diagnosis Table

| Symptom | Most Likely Cause | Quick Fix |
|---------|-------------------|-----------|
| Stall at t=5s | Large angle jump | Increase BLEND_DURATION_US |
| Vibration t=5-7s | Angle unwrapping issue | Check angle_diff calculation |
| Blend stuck at 0.0 | Timer not starting | Check encoder_mode_start_time init |
| Blend never reaches 1.0 | Mode oscillation | Add transition hysteresis |
| angle_diff > 1 rad | Accumulated drift | Increase SYNC_GAIN |
| Random stalls | Encoder error bits | Check sensor status register |

---

## 💡 Pro Tips

1. **Always monitor blend_factor first** - It tells you if blending is even active
2. **Watch angle_diff at t=4.99s** - Predicts how hard the blend will be
3. **Velocity should stay constant** - If it drops, blend isn't smooth enough
4. **Compare pre-blend angles** - Shows if encoder is noisy or accumulated drifted
5. **Export data to CSV** - Easier to spot subtle issues in post-processing

---

## 📝 Data Collection Template

```
Test Date: __________
Motor Speed: ________ rad/s
Blend Duration: ________ seconds

t=4.9s:
  - angle_diff: ________ rad
  - velocity: ________ rad/s

t=5.0s (blend start):
  - blend_factor: ________ (should be 0.0)
  - velocity: ________ rad/s

t=6.0s (mid-blend):
  - blend_factor: ________ (should be 0.5)
  - velocity: ________ rad/s
  - angle_diff: ________ rad

t=7.0s (blend end):
  - blend_factor: ________ (should be 1.0)
  - velocity: ________ rad/s

Result: [ ] PASS  [ ] FAIL
Notes: ________________________________
```

---

## 🎯 Success Criteria Summary

**PASS if ALL true:**
- ✅ blend_factor ramps 0.0 → 1.0 in 2.0 seconds
- ✅ Motor velocity stable (±5% of target)
- ✅ No audible vibration or roughness
- ✅ angle_diff decreases during blend
- ✅ blended_result has no discontinuities
- ✅ Works at 5, 10, 15, 20 rad/s

**FAIL if ANY true:**
- ❌ Motor stalls or stops
- ❌ Large velocity drop (>20%)
- ❌ Voltage saturates (>18V)
- ❌ Audible grinding or clicking
- ❌ blend_factor doesn't reach 1.0

---

**For detailed troubleshooting, see `ANGLE_BLENDING_IMPLEMENTED.md`**