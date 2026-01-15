# Angle Blending Visual Guide

## 🎨 Visual Representation of the Blend Process

### Timeline View
```
    0s                    5s                    7s                    10s
    |---------------------|---------------------|---------------------|
    |                     |                     |                     |
    | ACCUMULATED MODE    | BLENDING MODE       | ENCODER MODE        |
    |                     |                     |                     |
    | 100% accumulated    | 0% → 100% encoder   | 100% encoder        |
    | 0% encoder          | 100% → 0% accum     | 0% accumulated      |
    |                     |                     |                     |
    |---------------------|---------------------|---------------------|
         Motor spins up        Gradual transition      Steady state
```

---

## 📊 Angle Transition Graphs

### Graph 1: Blend Factor Over Time
```
blend_factor
    1.0 |                        ________________
        |                      /
        |                    /
    0.5 |                  /
        |                /
        |              /
    0.0 |____________/
        +----+----+----+----+----+----+----+----+
        0s   2s   4s   5s   6s   7s   8s   10s
                       ↑              ↑
                   Transition      Blend
                     starts       complete
```

### Graph 2: Angle Sources During Transition
```
Angle (rad)
    6.28|     ╱╲    accumulated ━━━━━━
        |    ╱  ╲                ╲
        |   ╱    ╲                ╲    ┌─ encoder ─ ─ ─
    3.14|  ╱      ╲                ╲  ╱
        | ╱        ╲                ╲╱
        |╱          ╲              ╱ ╲
    0.0 +────────────╲──────────╱────╲────────
        0s   2s   4s  5s   6s   7s   8s   10s
                       └────┬────┘
                         BLEND
                    blended_result ═══════
```

### Graph 3: Angular Error Convergence
```
angle_diff (rad)
    1.0 |     
        |     ●
    0.5 |       ●
        |         ●
    0.2 |           ●●
        |              ●●
    0.0 |                ●●●●●●●●●●●●●●●●
   -0.2 |
        +────+────+────+────+────+────+────+
        0s   2s   4s   5s   6s   7s   8s   10s
                       └──────┬──────┘
                       Error decreases
                       during blend
```

---

## 🔄 State Machine Diagram

```
    ┌─────────────────────────────────────────────────────┐
    │                   SYSTEM START                      │
    └────────────────────┬────────────────────────────────┘
                         │
                         ▼
    ┌─────────────────────────────────────────────────────┐
    │           ACCUMULATED ANGLE MODE                    │
    │                                                     │
    │  • angle = ∫(target_velocity × dt)                 │
    │  • Sync toward encoder with small gain             │
    │  • blend_start_time = 0                            │
    │  • accumulated_angle_source = 2.0                  │
    └────────────────────┬────────────────────────────────┘
                         │
                         │ time > 5s
                         │
                         ▼
    ┌─────────────────────────────────────────────────────┐
    │              BLENDING MODE                          │
    │                                                     │
    │  • Start timer: blend_start_time = micros()        │
    │  • Calculate: blend_factor = time / 2.0s           │
    │  • angle = accum + (blend_factor × diff)           │
    │  • accumulated_angle_source = 3.0                  │
    └────────────────────┬────────────────────────────────┘
                         │
                         │ blend_factor >= 1.0
                         │
                         ▼
    ┌─────────────────────────────────────────────────────┐
    │              ENCODER MODE                           │
    │                                                     │
    │  • angle = encoder_angle (100%)                    │
    │  • blend_factor = 1.0 (clamped)                    │
    │  • Pure closed-loop control                        │
    └─────────────────────────────────────────────────────┘
```

---

## 🎯 Blend Calculation Visualization

### Step-by-Step at t=5.5s (blend_factor=0.25)

```
1. INPUTS:
   ┌──────────────────────┐        ┌──────────────────────┐
   │ accumulated_angle    │        │ encoder_angle        │
   │     4.50 rad         │        │     4.70 rad         │
   └──────────────────────┘        └──────────────────────┘
            │                               │
            └───────────┬───────────────────┘
                        │
                        ▼
2. CALCULATE DIFFERENCE:
   ┌────────────────────────────────────────┐
   │ angle_diff = 4.70 - 4.50 = 0.20 rad   │
   │ (unwrapped, shortest path)             │
   └────────────────────────────────────────┘
                        │
                        ▼
3. APPLY BLEND FACTOR:
   ┌────────────────────────────────────────┐
   │ blend_contribution = 0.25 × 0.20       │
   │                    = 0.05 rad          │
   └────────────────────────────────────────┘
                        │
                        ▼
4. ADD TO ACCUMULATED:
   ┌────────────────────────────────────────┐
   │ result = 4.50 + 0.05 = 4.55 rad       │
   │                                        │
   │ (25% toward encoder, 75% accumulated)  │
   └────────────────────────────────────────┘
                        │
                        ▼
5. OUTPUT:
   ┌──────────────────────┐
   │ blended_result       │
   │     4.55 rad         │
   └──────────────────────┘
```

---

## 📏 Angle Unwrapping Example

### Why Unwrapping is Critical

```
Scenario: Encoder wraps from 6.28 → 0.0 rad

WITHOUT unwrapping:
   accumulated_angle = 6.20 rad
   encoder_angle     = 0.10 rad (wrapped)
   angle_diff        = 0.10 - 6.20 = -6.10 rad  ❌ WRONG!
   → Motor would rotate backward 6.10 rad

WITH unwrapping:
   accumulated_angle = 6.20 rad
   encoder_angle     = 0.10 rad (wrapped)
   raw_diff          = 0.10 - 6.20 = -6.10 rad
   
   if (raw_diff < -π):
       angle_diff = raw_diff + 2π
       angle_diff = -6.10 + 6.28 = 0.18 rad  ✅ CORRECT!
   
   → Motor rotates forward 0.18 rad (shortest path)
```

### Unwrapping Logic Flowchart

```
                    ┌─────────────────────┐
                    │  Calculate raw_diff │
                    │  = enc - accum      │
                    └──────────┬──────────┘
                               │
                    ┌──────────▼──────────┐
                    │ raw_diff > π ?      │
                    └──┬───────────────┬──┘
                  YES  │               │  NO
               ┌───────▼─────┐    ┌────▼────────┐
               │ diff = raw  │    │ raw_diff    │
               │   - 2π      │    │   < -π ?    │
               │             │    └──┬───────┬──┘
               │ (wrapped    │  YES  │       │  NO
               │  forward)   │ ┌─────▼────┐  │
               └─────────────┘ │ diff =   │  │
                               │  raw + 2π│  │
                               │          │  │
                               │ (wrapped │  │
                               │ backward)│  │
                               └──────────┘  │
                                             │
                    ┌────────────────────────▼────┐
                    │ diff = raw_diff             │
                    │ (no wrapping needed)        │
                    └─────────────────────────────┘
                               │
                    ┌──────────▼──────────┐
                    │  diff now in range  │
                    │     [-π, +π]        │
                    └─────────────────────┘
```

---

## 🎬 Transition Animation (ASCII)

### Frame-by-Frame at 0.5s Intervals

```
t=4.5s (Before Transition)
    Angle Sources:
    Accumulated: ████████████████████████ 100%
    Encoder:                              0%
    Result:      ████████████████████████

t=5.0s (Transition Starts)
    Angle Sources:
    Accumulated: ████████████████████████ 100%
    Encoder:                              0%
    Result:      ████████████████████████
    [BLEND TIMER STARTS]

t=5.5s (25% Blended)
    Angle Sources:
    Accumulated: ██████████████████       75%
    Encoder:     ██████                   25%
    Result:      ████████████████████████

t=6.0s (50% Blended)
    Angle Sources:
    Accumulated: ████████████             50%
    Encoder:     ████████████             50%
    Result:      ████████████████████████

t=6.5s (75% Blended)
    Angle Sources:
    Accumulated: ██████                   25%
    Encoder:     ██████████████████       75%
    Result:      ████████████████████████

t=7.0s (Blend Complete)
    Angle Sources:
    Accumulated:                          0%
    Encoder:     ████████████████████████ 100%
    Result:      ████████████████████████
    [BLEND COMPLETE]
```

---

## 🔬 Detailed Variable Flow

### Data Flow During Blend

```
                    loopFOC() @ 10kHz
                          │
                          ▼
    ┌─────────────────────────────────────────┐
    │ Read Encoder (non-blocking)             │
    │ → g_cached_encoder_angle                │
    └──────────────┬──────────────────────────┘
                   │
                   ▼
    ┌─────────────────────────────────────────┐
    │ update_speed_closed_loop()              │
    └──────────────┬──────────────────────────┘
                   │
                   ▼
    ┌─────────────────────────────────────────┐
    │ Check time since startup                │
    │ time < 5s ? → Accumulated Mode          │
    │ time > 5s ? → Encoder Mode (blend)      │
    └──────────────┬──────────────────────────┘
                   │
         ┌─────────┴─────────┐
         │                   │
         ▼                   ▼
    ACCUMULATED         ENCODER MODE
       MODE             (with blending)
         │                   │
         │                   ├─→ Start timer
         │                   ├─→ Calculate blend_factor
         │                   ├─→ Unwrap angle_diff
         │                   ├─→ Blend angles
         │                   └─→ Normalize result
         │                   │
         └───────┬───────────┘
                 │
                 ▼
    ┌─────────────────────────────────────────┐
    │ electrical_angle_radians                │
    └──────────────┬──────────────────────────┘
                   │
                   ▼
    ┌─────────────────────────────────────────┐
    │ setPhaseVoltage(Uq, Ud, angle)          │
    │ → Inverse Park Transform                │
    │ → Space Vector PWM                      │
    │ → Motor phases energized                │
    └─────────────────────────────────────────┘
```

---

## 🎪 Example Scenarios

### Scenario 1: Perfect Transition (Ideal Case)

```
Variables at t=4.99s:
  accumulated_angle = 4.500 rad
  encoder_angle     = 4.520 rad
  angle_diff        = 0.020 rad  ← Very small!
  
Blend Progress:
  t=5.0s: blend=0.00 → angle = 4.500 + (0.00 × 0.020) = 4.500
  t=5.5s: blend=0.25 → angle = 4.500 + (0.25 × 0.020) = 4.505
  t=6.0s: blend=0.50 → angle = 4.500 + (0.50 × 0.020) = 4.510
  t=6.5s: blend=0.75 → angle = 4.500 + (0.75 × 0.020) = 4.515
  t=7.0s: blend=1.00 → angle = 4.500 + (1.00 × 0.020) = 4.520
  
Result: ✅ Smooth transition, motor maintains sync
```

### Scenario 2: Large Angular Error (Challenging Case)

```
Variables at t=4.99s:
  accumulated_angle = 3.800 rad
  encoder_angle     = 4.500 rad
  angle_diff        = 0.700 rad  ← Large difference!
  
Blend Progress:
  t=5.0s: blend=0.00 → angle = 3.800 + (0.00 × 0.700) = 3.800
  t=5.5s: blend=0.25 → angle = 3.800 + (0.25 × 0.700) = 3.975
  t=6.0s: blend=0.50 → angle = 3.800 + (0.50 × 0.700) = 4.150
  t=6.5s: blend=0.75 → angle = 3.800 + (0.75 × 0.700) = 4.325
  t=7.0s: blend=1.00 → angle = 3.800 + (1.00 × 0.700) = 4.500
  
Result: ✅ Gradual correction, no sudden jump
Note: May need to increase blend duration to 3-4 seconds
```

### Scenario 3: Encoder Error During Transition

```
Variables at t=4.99s:
  accumulated_angle = 4.200 rad
  encoder_angle     = 4.250 rad
  angle_diff        = 0.050 rad
  
At t=5.3s, encoder has error bit set:
  encoder_angle jumps to 2.100 rad (ERROR!)
  
Without blending:
  ❌ Motor would jump: 4.2 → 2.1 rad instantly
  ❌ Loss of synchronization
  ❌ Motor stalls
  
With blending:
  t=5.3s: blend=0.15 → angle = 4.200 + (0.15 × error)
  ✅ Error is dampened by blend factor
  ✅ Motor tracks blended angle (more accumulated than encoder)
  ✅ System remains stable
  
Result: ✅ Blend protects against transient encoder errors
```

---

## 📐 Mathematical Representation

### Blend Equation

```
θ_result(t) = θ_accum + k(t) × Δθ

Where:
  θ_result = Final angle used for commutation
  θ_accum  = Accumulated angle (integrated velocity)
  k(t)     = Blend factor (time-dependent)
  Δθ       = Angular difference (unwrapped)

Blend factor:
  k(t) = clamp((t - t_start) / T_blend, 0, 1)
  
  t       = Current time
  t_start = Transition start time (5 seconds)
  T_blend = Blend duration (2 seconds)
```

### Unwrapping Function

```
Δθ_unwrapped = unwrap(θ_encoder - θ_accum)

unwrap(x) = {
    x - 2π,  if x > π
    x + 2π,  if x < -π
    x,       otherwise
}

This ensures Δθ ∈ [-π, +π], taking shortest rotational path
```

---

## 🎯 Performance Metrics

### CPU Timing
```
Function: Blend Calculation
┌──────────────────────────────┐
│ Operations per call:         │
│  - 1× float subtraction      │   ~5 cycles
│  - 2× float comparison       │   ~4 cycles
│  - 1× conditional add/sub    │   ~3 cycles
│  - 1× float multiplication   │   ~6 cycles
│  - 1× float addition         │   ~5 cycles
│  - Function call normalize() │  ~20 cycles
│                              │
│ Total: ~43 CPU cycles        │
│                              │
│ At 480 MHz: 0.09 microsec    │
│ At 10 kHz:  0.9% CPU load    │
└──────────────────────────────┘
```

### Memory Usage
```
Static Variables (in function):
  encoder_mode_start_time  →  4 bytes
  
Global Debug Variables:
  g_encoder_blend_factor         →  4 bytes
  g_encoder_blend_start_time     →  4 bytes
  g_accumulated_angle_pre_blend  →  4 bytes
  g_encoder_angle_pre_blend      →  4 bytes
  g_blended_result               →  4 bytes
  
Total: 24 bytes
```

---

## ✅ Verification Checklist

### Visual Inspection
- [ ] Plot `g_encoder_blend_factor` - should be perfect ramp
- [ ] Plot `g_blended_result` - should be smooth curve
- [ ] Plot `g_angle_difference_accum_vs_encoder` - should decrease
- [ ] No discontinuities in any angle plot
- [ ] Velocity plot stays constant through transition

### Numerical Checks
- [ ] `g_encoder_blend_factor` reaches exactly 1.0
- [ ] Transition occurs at exactly t=5.0s
- [ ] Blend completes at exactly t=7.0s
- [ ] `angle_diff` < 0.5 rad at end of blend
- [ ] No overflow in `encoder_mode_start_time`

### Motor Behavior
- [ ] No audible change at t=5s
- [ ] No vibration during t=5-7s
- [ ] Current draw remains steady
- [ ] Voltage command stable
- [ ] Motor continues accelerating smoothly

---

## 🎓 Summary

The angle blending solution transforms a **hard switch** into a **smooth transition**:

**Before:**
```
t=4.99s: angle = accumulated
t=5.00s: angle = encoder  ← INSTANT JUMP!
```

**After:**
```
t=4.99s: angle = accumulated
t=5.00s: angle = accumulated + 0% × diff
t=5.50s: angle = accumulated + 25% × diff
t=6.00s: angle = accumulated + 50% × diff
t=6.50s: angle = accumulated + 75% × diff
t=7.00s: angle = accumulated + 100% × diff = encoder
```

**Result:** Motor maintains synchronization through transition, eliminating stalls and vibration.

---

**For implementation details, see `ANGLE_BLENDING_IMPLEMENTED.md`**
**For monitoring, see `BLEND_MONITORING_QUICK_REF.md`**