# Quick Start: Add Blend Monitoring in 5 Minutes

## What You Need to Know

Your motor control system **already has** a blend ratio signal implemented. You just need to **add it to STM32CubeMonitor** to see it!

---

## ⚡ 5-Minute Setup

### Step 1: Open STM32CubeMonitor (30 seconds)

1. Launch STM32CubeMonitor
2. Connect to your STM32H755 board
3. Open your existing monitoring session (or create new one)

### Step 2: Add the Blend Ratio Variable (1 minute)

**The variable you need:**
```
g_hybrid_blend_factor
```

**How to add it:**
1. In STM32CubeMonitor, click **"Add Variable"**
2. Type: `g_hybrid_blend_factor`
3. Set display name: `Blend Ratio`
4. Set Y-axis range: **0.0 to 1.0**
5. Choose a bright color (e.g., yellow or magenta)
6. Make line thick for visibility

### Step 3: Add Supporting Variables (2 minutes)

Add these for complete picture:

| Variable Name | Display Name | Y-Axis | Color |
|---------------|--------------|--------|-------|
| `g_hybrid_measured_speed` | Speed (rad/s) | 0-50 | Green |
| `g_kalman_velocity` | Kalman Vel | 0-50 | Cyan |
| `g_target_rps_to_cl_controller` | Target Speed | 0-50 | White |

### Step 4: Run Motor and Observe (1.5 minutes)

1. Start motor from rest
2. Ramp to >20 rad/s
3. Watch `g_hybrid_blend_factor`:
   - Should be **0.0** at low speed (<10 rad/s)
   - Should smoothly ramp **0.0 → 1.0** as speed increases
   - Should be **1.0** at high speed (>20 rad/s)

---

## 🎯 What You Should See

### Perfect Transition
```
Blend    1.0 |                    ┌─────────────
Factor       |                   /
             |                  /
         0.5 |                 /
             |                /
         0.0 |───────────────┘
             └─────────────────────────────────
             0s   2s   4s   6s   8s   10s
             
             Open  Transition  Closed-Loop
```

### What Each Value Means

| Blend Value | Control Mode | What's Happening |
|-------------|--------------|------------------|
| **0.0** | Pure Open-Loop | Using accumulated angle, ignoring encoder |
| **0.0 - 1.0** | Blended | Smoothly transitioning control authority |
| **1.0** | Pure Closed-Loop | Using encoder angle, full precision control |

---

## ✅ Success Criteria

Your system is working correctly if:

- [x] Blend factor starts at **0.0** during startup
- [x] Blend factor smoothly transitions (no oscillations)
- [x] Blend factor reaches **1.0** and stays there
- [x] Motor runs smoothly throughout transition (no stutter)
- [x] Transition occurs around 10-20 rad/s speed range

---

## 🚨 Common Issues (and Quick Fixes)

### Issue 1: Blend Factor Stuck at 0.0

**Symptom:** Never transitions to closed-loop
**Quick Fix:** Check if motor speed reaches 10 rad/s
- Add `g_kalman_velocity` to verify speed measurement
- If speed is correct but blend stuck, check code at line 2744

### Issue 2: Motor Stutters During Transition

**Symptom:** Rough operation when blend factor changes
**Quick Fix:** Angles may be misaligned
- Add `g_open_loop_elec_before_blend` and `g_closed_loop_elec_before_blend`
- Check if difference is small (<0.5 rad) before transition
- If large difference, recalibrate encoder offset

### Issue 3: Blend Factor Oscillates

**Symptom:** Rapidly switches instead of smooth ramp
**Quick Fix:** Speed estimate may be noisy
- Add `g_kalman_velocity` to see if speed oscillates
- May need to tune Kalman filter or widen thresholds

---

## 📊 Recommended Chart Layout

### Chart 1: Blend Overview (Primary)
**Y-Axis 1 (0-1):**
- `g_hybrid_blend_factor` (thick yellow line)

**Y-Axis 2 (0-50 rad/s):**
- `g_hybrid_measured_speed` (green)
- `g_target_rps_to_cl_controller` (white dashed)

### Chart 2: Angle Details (Optional)
**Y-Axis (0-6.28 rad):**
- `g_open_loop_elec_before_blend` (orange)
- `g_closed_loop_elec_before_blend` (blue)
- `g_blended_angle_before_rate_limit` (green)
- `g_cached_encoder_angle` (gray)

---

## 🔧 Advanced: Tuning the Thresholds

If you want to adjust when transition occurs:

**File:** `project3/CM7/Core/Src/motors/StepperMotor.cpp`  
**Location:** Around line 2744-2746

```cpp
const float SPEED_THRESHOLD_LOW = 10.0f;   // Start transition
const float SPEED_THRESHOLD_HIGH = 20.0f;  // End transition
```

### Suggested Adjustments

**Smoother, slower transition:**
```cpp
const float SPEED_THRESHOLD_LOW = 5.0f;
const float SPEED_THRESHOLD_HIGH = 30.0f;
```

**Faster transition:**
```cpp
const float SPEED_THRESHOLD_LOW = 15.0f;
const float SPEED_THRESHOLD_HIGH = 20.0f;
```

**Stay in open-loop longer (high-speed apps):**
```cpp
const float SPEED_THRESHOLD_LOW = 20.0f;
const float SPEED_THRESHOLD_HIGH = 40.0f;
```

---

## 📖 Additional Variables (If You Want More Detail)

All of these are **already implemented** in your code:

| Variable | Purpose |
|----------|---------|
| `g_hybrid_open_loop_angle` | Open-loop angle value |
| `g_hybrid_closed_loop_angle` | Closed-loop angle value |
| `g_blended_angle_before_rate_limit` | Result of blending |
| `g_blended_angle_after_rate_limit` | After rate limiter |
| `g_angle_change_limited` | Rate limiter delta |
| `g_rate_limiter_active` | Is rate limiter active? |

---

## 💡 Pro Tips

1. **Record from t=0**: Start recording before motor turns on to see complete startup
2. **Use triggers**: Set trigger when `g_hybrid_blend_factor > 0.01` to capture transition
3. **Zoom in**: Transition happens in ~2 seconds, zoom to see details
4. **Compare duty cycles**: Watch duty cycles during transition - should stay smooth
5. **Check encoder**: `g_as5048_angle` should increase continuously (no jumps)

---

## 🎓 Understanding the Blend

### The Formula
```
final_angle = (1 - blend_factor) × open_loop_angle
            +      blend_factor  × closed_loop_angle
```

### Examples

**At startup (blend = 0.0):**
```
final = 1.0 × open_loop + 0.0 × closed_loop
      = open_loop only ✓
```

**During transition (blend = 0.5):**
```
final = 0.5 × open_loop + 0.5 × closed_loop
      = average of both ✓
```

**Running steady (blend = 1.0):**
```
final = 0.0 × open_loop + 1.0 × closed_loop
      = closed_loop only ✓
```

**It's a weighted average that smoothly hands control from open to closed loop!**

---

## ✨ Why This Matters

### Problem Without Blending
- Hard switch from open to closed loop
- Sudden angle jump = torque disturbance
- Motor may stall or vibrate
- Audible "thunk" sound

### Solution With Blending
- ✅ Smooth, gradual transition
- ✅ No torque disturbance
- ✅ Silent operation
- ✅ Robust control at all speeds
- ✅ Best of both worlds!

---

## 📚 Full Documentation

For deeper dive:
- **BLEND_RATIO_SUMMARY.md** - Complete technical reference
- **BLEND_MONITORING_GUIDE.md** - Detailed troubleshooting
- **BLEND_FLOW_DIAGRAM.txt** - Visual flowcharts
- **ANALYZING_YOUR_GRAPHS.md** - How to read your data

---

## 🎯 Your Next Action

**RIGHT NOW:**
1. Open STM32CubeMonitor
2. Add variable: `g_hybrid_blend_factor`
3. Run motor
4. Watch the magic happen! ✨

**The variable exists and is working - you just need to visualize it!**

---

## Summary

✅ **Blend ratio is already implemented**  
✅ **Variable: `g_hybrid_blend_factor`**  
✅ **Range: 0.0 (open-loop) to 1.0 (closed-loop)**  
✅ **Just add to STM32CubeMonitor and observe!**

**Time to complete: 5 minutes**  
**Difficulty: Easy**  
**Reward: Complete visibility into motor control transition!** 🚀