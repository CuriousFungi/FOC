# Analyzing Your Motor Control Graphs - Blend Ratio Perspective

## Current Graph Analysis

Based on your two graphs, here's what we can see and what's missing:

---

## Graph 1: Long-Term View (22.6s - 24.4s)

### What You're Seeing

**Orange Line (`g_as5048_angle`):**
- Starts at ~5.8 rad at t=22.6s
- Climbs steadily to ~2.3 rad at t=24.4s
- This is the **mechanical angle** from the encoder
- Shows continuous rotation - motor is running smoothly
- The "jump" from ~6.3 down to ~0.8 around t=23.5s is the 2π wraparound (normal!)

**Purple Spikes (`g_cached_encoder_angle`):**
- Regular spikes from 0 to ~6.3 rad
- These are the **electrical angle** wrapping every electrical cycle
- With 50 pole pairs, you get 50 electrical cycles per mechanical rotation
- Frequency of spikes = motor speed × pole pairs
- This is CORRECT behavior!

**Other Variables:**
- Duty cycles (cyan, blue, yellow, gray) showing sinusoidal patterns
- All properly synchronized with electrical angle
- Good sign - proper commutation is happening

### What's MISSING - The Blend Ratio!

**Critical variable not shown: `g_hybrid_blend_factor`**

This would tell you:
- Are you in open-loop (0.0), closed-loop (1.0), or transition?
- Based on the graph timeframe (22.6s+), you're likely in **closed-loop mode**
- But we need to add `g_hybrid_blend_factor` to confirm!

---

## Graph 2: Close-Up View (22.47s - 22.64s)

### What You're Seeing

**Duty Cycle Patterns:**
- `g_dutycycle_1A`, `1B`, `2A`, `2b` all showing smooth sinusoidal waves
- Phase-shifted from each other (correct for bipolar stepper)
- Peak values around 0.75-0.80 (75-80% duty cycle)
- Clean waveforms = good electrical commutation

**Purple/Green Spikes:**
- Sharp transitions in encoder angles
- This is the electrical angle wrapping (0 → 2π → 0)
- Happens ~4 times in 170ms window
- Indicates motor is rotating at approximately:
  - 4 cycles / 0.17s = 23.5 electrical cycles/sec
  - Divided by 50 pole pairs = 0.47 mechanical rotations/sec
  - = 0.47 × 2π = **2.95 rad/s mechanical speed**

### Analysis

At 2.95 rad/s:
- Below the 10 rad/s threshold for transition
- **Blend factor should be 0.0** (pure open-loop)
- But without seeing `g_hybrid_blend_factor`, we're guessing!

---

## What You SHOULD Add to Your Monitoring

### Essential Addition: Blend Ratio

Add these variables to a **new Y-axis** (range 0-1):

1. **`g_hybrid_blend_factor`** (PRIMARY - shows blend 0.0 to 1.0)
2. **`g_hybrid_measured_speed`** (scaled to Y-axis, shows speed used for blend)

### Enhanced Angle Monitoring

Add these to see the blending in action:

3. **`g_open_loop_elec_before_blend`** (orange)
4. **`g_closed_loop_elec_before_blend`** (blue)
5. **`g_blended_angle_before_rate_limit`** (green)

This will show you:
- When blend_factor = 0.0, green line should match orange (open-loop wins)
- When blend_factor = 1.0, green line should match blue (closed-loop wins)
- During transition, green line interpolates between orange and blue

---

## Expected Behavior During Complete Startup

### Phase 1: Alignment (t = 0 to ~1s)
```
g_hybrid_blend_factor = 0.0
Motor held at fixed angle
Speed = 0 rad/s
```

### Phase 2: Open-Loop Acceleration (t = 1s to ~3s)
```
g_hybrid_blend_factor = 0.0
Speed ramping up: 0 → 10 rad/s
Using accumulated angle based on target velocity
g_as5048_angle should increase linearly
```

### Phase 3: Transition/Blending (t = ~3s to ~5s)
```
g_hybrid_blend_factor: 0.0 → 1.0 (smooth ramp)
Speed: 10 → 20 rad/s
Blended angle transitions from open-loop to encoder-based
WATCH FOR: Smooth transition, no angle jumps
```

### Phase 4: Closed-Loop Operation (t = 5s+)
```
g_hybrid_blend_factor = 1.0
Speed > 20 rad/s (steady state)
Pure encoder-based control
g_closed_loop_elec_before_blend == g_cached_encoder_angle
```

---

## Diagnostic Checklist

### ✅ Things That Look Good in Your Graphs

- [x] Duty cycles are smooth sinusoids (no jitter)
- [x] Mechanical angle increasing continuously (motor rotating)
- [x] Electrical angle wrapping regularly (proper pole pair handling)
- [x] No obvious discontinuities or glitches
- [x] Waveforms properly phase-shifted

### ❓ Things We Can't Verify Yet (Need Blend Variables)

- [ ] Is motor in open-loop or closed-loop mode?
- [ ] Has transition occurred smoothly?
- [ ] Is speed estimate correct for blend calculation?
- [ ] Are open/closed angles aligned before blending?
- [ ] Is rate limiter active (protecting against voltage spikes)?

---

## Specific Questions Your Graphs Raise

### Question 1: Why Does `g_as5048_angle` Start at 5.8 rad?

**Answer:** This is fine! The encoder measures absolute position. Motor could have:
- Been manually positioned before startup
- Completed alignment phase before this graph window
- Started from arbitrary initial position

What matters is that it **increases continuously** (shows rotation).

### Question 2: What's the "Glitch" Around t=22.8s?

**Looking at first graph around t=22.8s:**
- Orange line (`g_as5048_angle`) shows slight deviation
- Could be:
  - Mechanical load variation (cogging torque)
  - Brief encoder read glitch
  - Rate limiter activation

**To diagnose:**
- Add `g_angle_change_limited` to see if rate limiter activated
- Add `g_kalman_velocity` to see if speed estimate dropped
- Add `g_hybrid_blend_factor` to see if blend ratio changed

### Question 3: Are We Getting Torque?

**Yes!** Evidence:
- Motor is rotating (angle increasing)
- Duty cycles are substantial (75-80%)
- Clean sinusoidal commutation
- Sustained rotation over 2+ seconds

**But to optimize:**
- Check if `g_amperage_q` shows expected torque-producing current
- Verify `g_amperage_d` is near zero (efficient operation)
- Monitor `g_Uq_to_setPhaseVoltage` (Q-axis voltage command)

---

## Recommended Next Steps

### Step 1: Update STM32CubeMonitor Configuration

Add a new chart with these signals:

**Primary Y-Axis (0-1 range):**
- `g_hybrid_blend_factor` (thick line, bright color)

**Secondary Y-Axis (0-50 rad/s range):**
- `g_hybrid_measured_speed` (medium line)
- `g_kalman_velocity` (thin line, for comparison)
- `g_target_rps_to_cl_controller` (dashed line, target reference)

**Angle Chart (0-2π range):**
- `g_open_loop_elec_before_blend` (orange)
- `g_closed_loop_elec_before_blend` (blue)
- `g_blended_angle_before_rate_limit` (green)
- `g_blended_angle_after_rate_limit` (red)
- `g_cached_encoder_angle` (gray, for reference)

### Step 2: Run Complete Startup Sequence

Record from t=0 (power on) through steady-state operation:

1. Start motor from rest
2. Let it accelerate through transition zone (10-20 rad/s)
3. Reach steady state (>20 rad/s)
4. Observe blend_factor: 0.0 → 0.0~1.0 → 1.0

### Step 3: Analyze Transition

Look for:
- **Smooth blend_factor ramp** (no oscillations or steps)
- **No angle discontinuities** during transition
- **Speed estimate matches target** (within ~10%)
- **Rate limiter not constantly active** (occasional use is OK)

### Step 4: Fine-Tune If Needed

If you see issues:

**Issue: Rough transition**
→ Widen threshold range (e.g., 5-30 rad/s instead of 10-20)

**Issue: Too much open-loop time**
→ Lower thresholds (e.g., 5-15 rad/s)

**Issue: Angle jump at transition**
→ Check encoder direction and offset calibration

**Issue: Oscillating blend_factor**
→ Add hysteresis or filter speed measurement

---

## Understanding The Blend Concept

### Why Blend At All?

**Problem with Pure Open-Loop:**
- Accumulated errors over time
- No correction for load disturbances
- Can't hold precise position

**Problem with Pure Closed-Loop at Low Speed:**
- Encoder resolution limits at low speed
- Noise can cause jitter
- Zero-torque at zero electrical angle error

**Solution: Hybrid Approach**
- Start with open-loop (predictable, strong startup torque)
- Gradually transition to closed-loop as speed increases
- Get best of both worlds!

### The Blend Formula

```
final_angle = (1 - blend_factor) × open_loop_angle
            +      blend_factor  × closed_loop_angle
```

**When blend_factor = 0.0:**
```
final_angle = 1.0 × open_loop_angle + 0.0 × closed_loop_angle
            = open_loop_angle
```

**When blend_factor = 0.5:**
```
final_angle = 0.5 × open_loop_angle + 0.5 × closed_loop_angle
            = average of both
```

**When blend_factor = 1.0:**
```
final_angle = 0.0 × open_loop_angle + 1.0 × closed_loop_angle
            = closed_loop_angle
```

It's a weighted average that smoothly transitions control authority!

---

## Summary

### What Your Graphs Show
✅ Motor is running
✅ Electrical commutation is working
✅ Duty cycles are clean
✅ Encoder is reading position

### What's Missing
❌ Blend ratio visibility
❌ Speed estimate visibility
❌ Open vs closed loop angle comparison
❌ Rate limiter activity

### Action Item
**Add `g_hybrid_blend_factor` to your monitoring session!**

This single variable will immediately tell you:
- Current control mode (open/closed/blended)
- When transition occurs
- If transition is smooth
- Whether your speed thresholds are appropriate

Then you'll have complete visibility into your motor control system! 🎯