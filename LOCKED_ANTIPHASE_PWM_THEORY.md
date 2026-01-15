# Locked Anti-Phase PWM Theory for Field-Oriented Control

## Document Purpose
This document explains the theory behind locked anti-phase PWM modulation for bi-directional motor control in Field-Oriented Control (FOC) applications.

---

## Table of Contents
1. [The Problem: Bipolar Voltage Control](#the-problem-bipolar-voltage-control)
2. [H-Bridge Fundamentals](#h-bridge-fundamentals)
3. [PWM Modulation Strategies](#pwm-modulation-strategies)
4. [Locked Anti-Phase PWM Theory](#locked-anti-phase-pwm-theory)
5. [Mathematical Derivation](#mathematical-derivation)
6. [FOC Integration](#foc-integration)
7. [Advantages and Disadvantages](#advantages-and-disadvantages)
8. [Implementation Considerations](#implementation-considerations)

---

## The Problem: Bipolar Voltage Control

### FOC Generates Bipolar Voltages

Field-Oriented Control (FOC) uses the inverse Park transform to convert quadrature voltages (Uq, Ud) into phase voltages (U_alpha, U_beta):

```
U_alpha = cos(θ) × Ud - sin(θ) × Uq
U_beta  = sin(θ) × Ud + cos(θ) × Uq
```

For a typical FOC application with Ud = 0 (no field weakening):

```
U_alpha = -sin(θ) × Uq
U_beta  = cos(θ) × Uq
```

As the electrical angle θ rotates from 0 to 2π, both U_alpha and U_beta are **sinusoidal and bipolar**:
- They swing from **-Uq to +Uq**
- They are **90° out of phase** (quadrature)

### The Challenge

A single power transistor (MOSFET/IGBT) can only:
- Conduct current in ONE direction
- Connect a terminal to VCC (high) or GND (low)

**Question:** How do we apply a bipolar voltage (positive AND negative) to a motor winding using unipolar transistors?

**Answer:** Use an H-bridge with proper modulation strategy.

---

## H-Bridge Fundamentals

### Basic H-Bridge Topology

```
                    VCC (+20V)
                      |
         Q1 ----+-----+-----+---- Q2
                |           |
         Phase A|           |Phase B
                |           |
                +--[Motor]--+
                |  Winding  |
         Q3 ----+           +---- Q4
                |           |
                +-----+-----+
                      |
                     GND
```

### Four Transistors, Three States

#### State 1: Positive Voltage Across Winding
```
Q1 = ON,  Q2 = OFF
Q3 = OFF, Q4 = ON
→ Current flows: VCC → Q1 → Phase A → Winding → Phase B → Q4 → GND
→ Voltage = +VCC
```

#### State 2: Negative Voltage Across Winding
```
Q1 = OFF, Q2 = ON
Q3 = ON,  Q4 = OFF
→ Current flows: VCC → Q2 → Phase B → Winding → Phase A → Q3 → GND
→ Voltage = -VCC
```

#### State 3: Zero Voltage (Freewheeling)
```
Q1 = OFF, Q2 = OFF, Q3 = ON, Q4 = ON  (both low sides on)
OR
Q1 = ON,  Q2 = ON,  Q3 = OFF, Q4 = OFF (both high sides on)
→ Current recirculates through winding and body diodes
→ Voltage ≈ 0V
```

### The Forbidden State: Shoot-Through

```
Q1 = ON, Q3 = ON  → Phase A shorted: VCC → Q1 → Q3 → GND
OR
Q2 = ON, Q4 = ON  → Phase B shorted: VCC → Q2 → Q4 → GND

⚠️ SHOOT-THROUGH! Power supply directly shorted through FETs!
Result: Excessive current, damaged FETs, no torque production
```

---

## PWM Modulation Strategies

To control average voltage, we use Pulse Width Modulation (PWM). But HOW we apply PWM to the H-bridge matters!

### Strategy 1: Sign/Magnitude PWM (⚠️ CAUSES SHOOT-THROUGH)

**Concept:**
- One pair of transistors (e.g., Q1/Q4) receives PWM for positive voltage
- Other pair (Q2/Q3) receives PWM for negative voltage
- Switch between pairs based on voltage polarity

**Implementation:**
```cpp
if (U_alpha >= 0) {
    // Positive voltage: PWM on Q1/Q4 pair
    Phase_A_high = PWM(duty_cycle);
    Phase_A_low  = 0;
    Phase_B_high = 0;
    Phase_B_low  = PWM(duty_cycle);
} else {
    // Negative voltage: PWM on Q2/Q3 pair
    Phase_A_high = 0;
    Phase_A_low  = PWM(duty_cycle);
    Phase_B_high = PWM(duty_cycle);
    Phase_B_low  = 0;
}
```

**Problem:**
During transitions from positive to negative (or vice versa), there's a brief moment where:
- Old pair hasn't fully turned off yet (transistor turn-off time)
- New pair starts turning on
- **Both pairs momentarily active → SHOOT-THROUGH!**

Even with dead-time insertion, high-frequency transitions (like in FOC) make this risky.

---

### Strategy 2: Locked Anti-Phase PWM (✅ PREVENTS SHOOT-THROUGH)

**Concept:**
- BOTH pairs of transistors receive PWM at ALL times
- PWM signals are **complementary** around 50% duty cycle
- Net voltage = difference between the two sides

**Key Insight:**
```
If Phase_A has duty D_A and Phase_B has duty D_B:
Average voltage across winding = (D_B - D_A) × V_supply

By constraining: D_A + D_B = 1.0 (complementary)
We get: D_B = 1 - D_A
Therefore: V_avg = (1 - D_A - D_A) × V_supply = (1 - 2×D_A) × V_supply
```

**Implementation:**
```cpp
// Always apply complementary PWM
float normalized = U_alpha / V_supply;  // Range: -1 to +1
float base_duty = 0.5f;                 // Center point

Phase_A_duty = base_duty - normalized × 0.5;  // Range: 0 to 1
Phase_B_duty = base_duty + normalized × 0.5;  // Range: 1 to 0

// Net voltage = (Phase_B_duty - Phase_A_duty) × V_supply = U_alpha
```

**Why No Shoot-Through?**
- Both transistor pairs are ALWAYS switching
- They are always complementary (when one is high, other is low)
- No polarity "transitions" that could cause overlap
- Dead-time in hardware handles the brief switching moments

---

## Locked Anti-Phase PWM Theory

### Mathematical Proof

Given:
- Desired voltage: `U_desired` (can be positive or negative)
- Supply voltage: `V_supply` (always positive)
- Normalized voltage: `U_norm = U_desired / V_supply` (range: -1 to +1)

Define duty cycles:
```
D_A = 0.5 - 0.5 × U_norm
D_B = 0.5 + 0.5 × U_norm
```

Verify complementary constraint:
```
D_A + D_B = (0.5 - 0.5×U_norm) + (0.5 + 0.5×U_norm)
          = 1.0  ✓
```

Calculate net voltage:
```
V_net = (D_B - D_A) × V_supply
      = [(0.5 + 0.5×U_norm) - (0.5 - 0.5×U_norm)] × V_supply
      = [0.5 + 0.5×U_norm - 0.5 + 0.5×U_norm] × V_supply
      = [U_norm] × V_supply
      = U_desired  ✓
```

### Example Calculations

Assume V_supply = 20V

#### Case 1: Zero Voltage (U_desired = 0V)
```
U_norm = 0 / 20 = 0
D_A = 0.5 - 0.5×0 = 0.5 (50%)
D_B = 0.5 + 0.5×0 = 0.5 (50%)
V_net = (0.5 - 0.5) × 20V = 0V ✓
```

#### Case 2: Positive Voltage (U_desired = +10V)
```
U_norm = 10 / 20 = 0.5
D_A = 0.5 - 0.5×0.5 = 0.25 (25%)
D_B = 0.5 + 0.5×0.5 = 0.75 (75%)
V_net = (0.75 - 0.25) × 20V = +10V ✓
```

#### Case 3: Negative Voltage (U_desired = -10V)
```
U_norm = -10 / 20 = -0.5
D_A = 0.5 - 0.5×(-0.5) = 0.75 (75%)
D_B = 0.5 + 0.5×(-0.5) = 0.25 (25%)
V_net = (0.25 - 0.75) × 20V = -10V ✓
```

#### Case 4: Maximum Positive Voltage (U_desired = +20V)
```
U_norm = 20 / 20 = 1.0
D_A = 0.5 - 0.5×1.0 = 0.0 (0%)
D_B = 0.5 + 0.5×1.0 = 1.0 (100%)
V_net = (1.0 - 0.0) × 20V = +20V ✓
```

#### Case 5: Maximum Negative Voltage (U_desired = -20V)
```
U_norm = -20 / 20 = -1.0
D_A = 0.5 - 0.5×(-1.0) = 1.0 (100%)
D_B = 0.5 + 0.5×(-1.0) = 0.0 (0%)
V_net = (0.0 - 1.0) × 20V = -20V ✓
```

---

## FOC Integration

### Inverse Park Transform Output

FOC produces two bipolar voltages for a 2-phase motor (or Alpha-Beta frame):

```
U_alpha = -sin(θ_e) × Uq
U_beta  = +cos(θ_e) × Uq

where:
  θ_e = electrical angle (0 to 2π)
  Uq  = torque-producing voltage (from velocity/torque controller)
```

### Applying Locked Anti-Phase to Both Phases

**Phase 1 (Alpha winding):**
```cpp
float alpha_normalized = U_alpha / V_supply;
duty_cycle_1A = 0.5f - alpha_normalized × 0.5f;
duty_cycle_1B = 0.5f + alpha_normalized × 0.5f;
```

**Phase 2 (Beta winding):**
```cpp
float beta_normalized = U_beta / V_supply;
duty_cycle_2A = 0.5f - beta_normalized × 0.5f;
duty_cycle_2B = 0.5f + beta_normalized × 0.5f;
```

### Verification

At any instant:
```
duty_cycle_1A + duty_cycle_1B = 1.0
duty_cycle_2A + duty_cycle_2B = 1.0

Net voltage on winding 1 = (duty_cycle_1B - duty_cycle_1A) × V_supply = U_alpha
Net voltage on winding 2 = (duty_cycle_2B - duty_cycle_2A) × V_supply = U_beta
```

### Temporal Behavior

As the motor rotates and θ_e increases from 0 to 2π:

**U_alpha varies as:** -sin(θ_e) × Uq
- At θ_e = 0°: U_alpha = 0V → duty_1A = 50%, duty_1B = 50%
- At θ_e = 90°: U_alpha = -Uq → duty_1A increases, duty_1B decreases
- At θ_e = 180°: U_alpha = 0V → duty_1A = 50%, duty_1B = 50%
- At θ_e = 270°: U_alpha = +Uq → duty_1A decreases, duty_1B increases

**U_beta varies as:** +cos(θ_e) × Uq (90° ahead of U_alpha)
- Similar sinusoidal variation but phase-shifted

The result: **Both motor windings receive smooth, sinusoidal voltage waveforms with no shoot-through risk.**

---

## Advantages and Disadvantages

### Advantages of Locked Anti-Phase PWM ✅

1. **No Shoot-Through Risk**
   - Both sides always switching complementary
   - No polarity transition problems
   - Hardware dead-time sufficient for safety

2. **Smooth Control**
   - Linear relationship between command and output voltage
   - No discontinuities at zero-crossing
   - Better low-speed performance

3. **Simpler Software**
   - No need to track polarity
   - No conditional logic for switching modes
   - Easier to debug

4. **Full Voltage Range**
   - Can utilize full ±V_supply range
   - Better torque at high speeds

### Disadvantages ⚠️

1. **Continuous Switching**
   - Both transistors always switching (even at zero voltage)
   - Higher switching losses
   - More EMI

2. **No Zero-Voltage State**
   - Can't truly "turn off" the H-bridge
   - Always some ripple current
   - May hear audible noise at 50% duty (zero voltage)

3. **Requires Matched Transistors**
   - Assumes both sides have equal ON-resistance
   - Timing mismatches can cause DC offset

### Comparison with Sign/Magnitude

| Feature | Sign/Magnitude | Locked Anti-Phase |
|---------|----------------|-------------------|
| Shoot-through risk | HIGH | NONE |
| Switching losses | Lower (one side off) | Higher (both sides switch) |
| Zero-crossing behavior | Discontinuous | Smooth |
| Low-speed control | Poor | Excellent |
| Implementation complexity | Medium | Low |
| FOC compatibility | Problematic | Ideal |

---

## Implementation Considerations

### 1. Supply Voltage and Duty Limits

Ensure normalized voltage stays within [-1, +1]:
```cpp
float normalized = symetric_clamp(U_alpha / V_supply, 1.0f);
```

This ensures duty cycles stay within [0, 1]:
```cpp
duty_A = 0.5f - normalized × 0.5f;  // Range: [0.0, 1.0]
duty_B = 0.5f + normalized × 0.5f;  // Range: [0.0, 1.0]
```

### 2. Dead-Time Insertion

Even with locked anti-phase, hardware dead-time is essential:
```cpp
// STM32 Timer Configuration
sBreakDeadTimeConfig.DeadTime = 200;  // 200 timer ticks
```

This prevents brief shoot-through during switching transitions.

### 3. PWM Frequency Selection

Typical FOC applications use:
- **PWM frequency:** 10-40 kHz
- **Higher frequency:** Less audible noise, more switching losses
- **Lower frequency:** More ripple, less switching losses

For STM32H7 @ 200 MHz with ARR=10000:
```
PWM_freq = Timer_clock / (Prescaler × ARR)
         = 200 MHz / (1 × 10000)
         = 20 kHz
```

### 4. Current Ripple

With locked anti-phase, current ripple depends on:
```
ΔI = (V_supply × duty_cycle × (1 - duty_cycle)) / (L × f_PWM)
```

Maximum ripple occurs at 50% duty (zero voltage command):
```
ΔI_max = V_supply / (4 × L × f_PWM)
```

For your system:
- V_supply = 20V
- L = 3.2 mH
- f_PWM = 20 kHz

```
ΔI_max = 20 / (4 × 0.0032 × 20000)
       = 20 / 256
       ≈ 78 mA ripple
```

This is acceptable for most applications.

### 5. Back-EMF Consideration

At high speeds, back-EMF reduces available voltage:
```
V_available = V_supply - V_bemf
```

Account for this in FOC controller:
```cpp
float back_emf = omega × (L × I_q + flux_linkage);
float voltage_limit = V_supply - back_emf;
```

---

## Advanced Topics

### Space Vector Modulation (SVM)

For 3-phase motors, locked anti-phase can be extended to SVM:
- Map (Uα, Uβ) to sector and vector
- Calculate dwell times
- Generate complementary PWM for all three phases

### Overmodulation

When commanded voltage exceeds V_supply:
```
U_desired > V_supply
```

Options:
1. Clamp to V_supply (lose linearity)
2. Use overmodulation algorithms (SVM)
3. Increase V_supply or reduce speed

---

## Summary

**Locked Anti-Phase PWM** is the optimal modulation strategy for FOC motor control because:

1. **Prevents shoot-through** by maintaining complementary duty cycles
2. **Smooth operation** with no zero-crossing discontinuities
3. **Simple implementation** with linear voltage control
4. **FOC compatible** with sinusoidal bipolar voltages

The trade-off of higher switching losses is acceptable given the reliability and control quality benefits.

**Mathematical core:**
```
duty_A = 0.5 - (U / V_supply) × 0.5
duty_B = 0.5 + (U / V_supply) × 0.5
V_net = (duty_B - duty_A) × V_supply = U
```

This elegant solution transforms the challenging problem of bipolar voltage generation into a straightforward complementary PWM implementation.

---

## References

1. **"Field Oriented Control of Permanent Magnet Motors"** - Texas Instruments Application Note
2. **"H-Bridge PWM Techniques for Motor Control"** - STMicroelectronics
3. **"Understanding Motor Driver Dead Time"** - Infineon Application Note
4. **SimpleFOC Library Documentation** - Open-source FOC implementation

---

**Document Version:** 1.0  
**Date:** January 2025  
**Author:** FOC Motor Control Development Team