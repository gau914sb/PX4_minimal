# Altitude Controller Tuning Plan
**Date**: 2025-12-23
**Status**: CRITICAL - Controller Unstable
**Target**: Stable hover at 3m altitude

---

## 🔴 Current Problem Summary

### Observed Behavior
- **Violent oscillations**: 6+ crash cycles in 70 seconds
- **Altitude range**: -1.26m to 12.79m (target: 3.0m)
- **Max descent rate**: -11.1 m/s (DANGEROUS)
- **Thrust saturation**: 20.5% of time at limits
- **Convergence**: NEVER achieved stable hover

### Root Causes
1. **D gain too high** (0.15-0.18) → Overcorrection on velocity changes
2. **P gain too weak** (0.04-0.06) → Can't pull drone back to target
3. **I term ineffective** → Anti-windup clamping prevents correction
4. **Thrust limits too restrictive** → Saturates during corrections

---

## 📋 Tuning Methodology - Ziegler-Nichols Inspired

We'll use a **systematic step-by-step approach**:

### Phase 1: Find Baseline (P-only control)
- Disable I and D completely
- Find minimum P gain that lifts drone
- Increase P until oscillations start
- Record critical gain (Kc) and oscillation period (Tc)

### Phase 2: Calculate Initial PID Values
Using Ziegler-Nichols formulas:
- **Kp = 0.6 × Kc**
- **Ki = 1.2 × Kc / Tc**
- **Kd = 0.075 × Kc × Tc**

### Phase 3: Fine-Tune Response
- Reduce D gain first (prevent overshooting)
- Adjust P gain (faster response vs stability)
- Add I gain gradually (eliminate steady-state error)

### Phase 4: Validate
- Test stability under disturbances
- Verify setpoint changes
- Confirm no limit saturation

---

## 🗺️ Step-by-Step Roadmap

### **STEP 1: Pure P Controller (Baseline)**
**Goal**: Find how much proportional gain we need without oscillations

**Parameters**:
```python
HOVER_THRUST = 0.65      # Start higher than minimum
KP_ALT = 0.08            # Start conservative
KI_ALT = 0.0             # DISABLED
KD_ALT = 0.0             # DISABLED
THRUST_LIMITS = [0.30, 0.90]  # Wide limits
```

**Expected Behavior**:
- Slow rise to target
- Possible steady-state error (hovering below 3m)
- NO oscillations

**Success Criteria**:
- ✓ Drone lifts off smoothly
- ✓ No crashes
- ✓ Settles within ±1m of target (even if not exactly 3m)

**If it fails**: Reduce KP by 50%, retry

---

### **STEP 2: Increase P Until Oscillation**
**Goal**: Find critical gain (Kc) where oscillations start

**Parameters**:
```python
HOVER_THRUST = 0.65
KP_ALT = 0.08 → 0.12 → 0.16 → 0.20  # Increase gradually
KI_ALT = 0.0
KD_ALT = 0.0
```

**Testing Protocol**:
1. Start at 0.08, test for 30 seconds
2. If stable, increase by 0.04
3. Repeat until you see sustained oscillations
4. **Record**: Kc (gain where oscillations start), Tc (oscillation period)

**Expected Behavior**:
- Higher P → faster response
- At Kc → constant amplitude oscillations
- Above Kc → growing oscillations (STOP!)

**Success Criteria**:
- ✓ Found Kc where oscillations are sustained but not growing
- ✓ Measured Tc (time for one complete cycle)

---

### **STEP 3: Apply Ziegler-Nichols PID**
**Goal**: Use calculated values as starting point

**Parameters** (example if Kc=0.16, Tc=4.0s):
```python
HOVER_THRUST = 0.65
KP_ALT = 0.6 × 0.16 = 0.096   ≈ 0.10
KI_ALT = 1.2 × 0.16 / 4.0 = 0.048  ≈ 0.05
KD_ALT = 0.075 × 0.16 × 4.0 = 0.048  ≈ 0.05
```

**Expected Behavior**:
- Quick rise to target (P term)
- Damped oscillations (D term)
- Settles at exact target (I term)
- ~25% overshoot typical

**Success Criteria**:
- ✓ Reaches 3m ± 0.5m
- ✓ Oscillations die out within 15 seconds
- ✓ No crashes

**If it oscillates**: Reduce D gain by 30%, retest

---

### **STEP 4: Reduce Overshoot (Tune D)**
**Goal**: Minimize overshoot while maintaining speed

**Parameters**:
```python
KP_ALT = 0.10  (from Step 3)
KI_ALT = 0.05  (from Step 3)
KD_ALT = 0.05 → 0.03 → 0.02  # Reduce if overshooting
       = 0.05 → 0.07 → 0.09  # Increase if too slow
```

**Tuning Rules**:
- **Too much overshoot** (>1m) → Reduce D by 20%
- **Too slow rise** (>20s to target) → Reduce D by 30%
- **Oscillations** → Reduce D by 50%

**Expected Behavior**:
- Smooth approach to 3m
- <10% overshoot (3.3m max)
- Settles in 10-15 seconds

**Success Criteria**:
- ✓ Overshoot < 0.5m
- ✓ Rise time < 15s
- ✓ No oscillations

---

### **STEP 5: Eliminate Steady-State Error (Tune I)**
**Goal**: Ensure drone holds exactly 3m, not 2.8m or 3.2m

**Parameters**:
```python
KP_ALT = (from Step 4)
KI_ALT = 0.05 → 0.03 → 0.01  # Reduce if oscillating
       = 0.05 → 0.07 → 0.10  # Increase if steady error persists
KD_ALT = (from Step 4)
```

**Tuning Rules**:
- **Steady error >0.2m** → Increase I by 50%
- **Slow oscillations** → Reduce I by 50%
- **Overshoot increased** → Reduce I by 30%

**Expected Behavior**:
- Gradual correction of any offset
- Final altitude = 3.00m ± 0.05m
- No oscillations

**Success Criteria**:
- ✓ Steady-state error < 0.1m after 20s
- ✓ No new oscillations introduced
- ✓ Stable for 30+ seconds

---

### **STEP 6: Stress Test**
**Goal**: Verify robustness under realistic conditions

**Test Cases**:
1. **Setpoint change**: 3m → 2m → 4m → 3m
2. **Manual disturbance**: Push drone down/up in simulation
3. **Long duration**: 2+ minutes hover
4. **Different targets**: 1m, 5m, 10m

**Parameters**: (Final values from Step 5)

**Expected Behavior**:
- Adapts to setpoint changes smoothly
- Recovers from disturbances
- No drift over time
- Works at different altitudes

**Success Criteria**:
- ✓ All tests pass without crashes
- ✓ Overshoot < 15% on setpoint changes
- ✓ Recovery time < 10s from disturbance
- ✓ No altitude drift > 0.2m in 60s

---

## 🎯 Target Performance Specifications

| Metric | Target | Acceptable | Current |
|--------|--------|------------|---------|
| **Overshoot** | <10% (<0.3m) | <20% (<0.6m) | 327% (9.8m) ❌ |
| **Rise time** | 5-10s | <15s | 3-5s ⚠️ |
| **Settling time** | <15s | <25s | NEVER ❌ |
| **Steady-state error** | <0.05m | <0.15m | 4.8m ❌ |
| **Max descent rate** | <3 m/s | <5 m/s | 11.1 m/s ❌ |
| **Oscillations** | None | Damped <3 cycles | Continuous ❌ |

---

## 📝 Testing Protocol

### Pre-Flight Checklist
- [ ] Log file naming working (timestamp)
- [ ] PX4 + Gazebo running cleanly
- [ ] Drone armed and in OFFBOARD mode
- [ ] Terminal ready to Ctrl+C for emergency stop

### During Flight
- [ ] Monitor console output every 0.5s
- [ ] Watch for altitude exceeding 12m (abort!)
- [ ] Watch for descent rate > 8 m/s (abort!)
- [ ] Record observations in notes

### Post-Flight Analysis
- [ ] Check log file for completeness
- [ ] Calculate: max altitude, min altitude, overshoot, settling time
- [ ] Plot altitude vs time (optional but helpful)
- [ ] Compare against target specs
- [ ] Document parameter changes

### Emergency Abort Criteria
- **Altitude > 15m** → Ctrl+C immediately
- **Descent > 10 m/s** → Ctrl+C immediately
- **Ground collision** → Ctrl+C, restart PX4
- **Continuous oscillations > 30s** → Ctrl+C, reduce D gain

---

## 🔧 Recommended Starting Values (Conservative)

Based on analysis and typical quadcopter values:

```python
# STEP 1 Starting Point (safest)
TARGET_ALTITUDE = 3.0
HOVER_THRUST = 0.65        # Known to generate lift
TAKEOFF_THRUST = 0.67      # Slightly higher for initial lift

# Pure P controller
KP_ALT = 0.08              # Conservative, proven to work in logs
KI_ALT = 0.0               # Disabled
KD_ALT = 0.0               # Disabled
MAX_INTEGRAL = 0.20        # Increased headroom
THRUST_LIMITS = [0.30, 0.90]  # Wide range to prevent saturation
```

### Expected Evolution
```
STEP 1: P=0.08, I=0.00, D=0.00 → Slow rise, settles ~2.7m
STEP 2: P=0.12, I=0.00, D=0.00 → Faster, oscillates slightly → Kc≈0.12
STEP 3: P=0.07, I=0.04, D=0.04 → Good rise, some overshoot
STEP 4: P=0.07, I=0.04, D=0.02 → Reduced overshoot to <0.5m
STEP 5: P=0.07, I=0.06, D=0.02 → Perfect 3.00m hold
STEP 6: VALIDATE → All tests pass ✓
```

---

## 📊 Data Analysis Tools

### Quick Analysis Commands
```bash
# Extract altitude column
grep -v "^#" flight_log_*.txt | cut -d',' -f3 | sort -n

# Max altitude
grep -v "^#" flight_log_*.txt | cut -d',' -f3 | sort -rn | head -1

# Min altitude
grep -v "^#" flight_log_*.txt | cut -d',' -f3 | sort -n | head -1

# Count saturation events
grep -v "^#" flight_log_*.txt | awk -F',' '{if($6<=0.31 || $6>=0.89) print}' | wc -l
```

### Python Analysis Script (Optional)
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load log
df = pd.read_csv('flight_log_20251223_165109.txt', comment='#',
                 names=['time', 'phase', 'alt', 'alt_err', 'vz', 'thrust',
                        'p', 'i', 'd', 'int_err', 'roll', 'pitch', 'yaw', 'yaw_rate'])

# Plot
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

ax1.plot(df['time'], df['alt'], label='Altitude')
ax1.axhline(3.0, color='r', linestyle='--', label='Target')
ax1.set_ylabel('Altitude (m)')
ax1.legend()
ax1.grid(True)

ax2.plot(df['time'], df['vz'], label='Vertical Velocity', color='orange')
ax2.set_ylabel('Vz (m/s)')
ax2.legend()
ax2.grid(True)

ax3.plot(df['time'], df['thrust'], label='Thrust', color='green')
ax3.set_ylabel('Thrust')
ax3.set_xlabel('Time (s)')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.savefig('altitude_analysis.png')
print(f"Max altitude: {df['alt'].max():.2f}m")
print(f"Min altitude: {df['alt'].min():.2f}m")
print(f"Overshoot: {(df['alt'].max() - 3.0):.2f}m")
```

---

## ⚠️ Safety Notes

1. **Always test in simulation first** - Never fly real hardware with untested gains
2. **Keep emergency stop ready** - Ctrl+C should be muscle memory
3. **Monitor altitude limits** - Abort if >15m or <-1m
4. **One parameter at a time** - Don't change multiple gains simultaneously
5. **Log everything** - Every test generates a timestamped log
6. **Take breaks** - Tuning takes patience, don't rush

---

## 📈 Progress Tracking

| Date | Step | Parameters | Result | Notes |
|------|------|------------|--------|-------|
| 2025-12-23 | Initial | P=0.04, I=0.008, D=0.15 | FAIL | Violent oscillations, 12.8m overshoot |
| | | | | |
| | | | | |

---

## 🎓 PID Tuning Quick Reference

### Effect of Each Term

**P (Proportional)**:
- ↑ P → Faster response, more overshoot
- ↓ P → Slower response, less overshoot
- Too high → Oscillations
- Too low → Never reaches target

**I (Integral)**:
- ↑ I → Eliminates steady-state error, adds overshoot
- ↓ I → Allows steady-state error, more stable
- Too high → Slow oscillations, windup
- Too low → Persistent offset from target

**D (Derivative)**:
- ↑ D → Reduced overshoot, damped response
- ↓ D → More overshoot, faster response
- Too high → Oscillations from noise, sluggish
- Too low → Overshoots target

### Common Issues & Solutions

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| Oscillates forever | D too high or P too high | Reduce D by 50% first |
| Never reaches target | P too low or I=0 | Increase P or add small I |
| Overshoots badly | D too low | Increase D by 50% |
| Slow to respond | P too low, D too high | Increase P, decrease D |
| Drifts over time | I too low or I=0 | Increase I gradually |
| Slow oscillations | I too high | Reduce I by 50% |

---

## ✅ Next Steps

1. **Implement Step 1** - Pure P controller with conservative values
2. **Run test** - Collect 30-60s of data
3. **Analyze log** - Check for stability, measure steady-state error
4. **Proceed to Step 2** - Increase P to find critical gain
5. **Document results** - Update progress tracking table
6. **Iterate** - Follow roadmap until Step 6 complete

**Estimated Time**: 2-3 hours for complete tuning process

---

**Good luck! Take it slow and systematic. The data will tell you what to do next.** 🚁
