# Parameter Dependency Analysis: MC Controllers

## Root Cause of Parameter Issues

### The Problem
When you enable `mc_att_control` and `mc_rate_control` but keep `mc_pos_control` **disabled**, you get parameter errors because:

**mc_att_control depends on MPC_* parameters that are defined in mc_pos_control module!**

## Parameter Source Mapping

### Module: `mc_pos_control` (Position Controller)
**Status in PX4_minimal**: ❌ **DISABLED**

This module contains **10 parameter files** that define ALL MPC_* (Multicopter Position Control) parameters:

| Parameter File | Parameters Defined | Used By |
|---------------|-------------------|---------|
| `multicopter_stabilized_mode_params.c` | MPC_MAN_TILT_MAX, MPC_MANTHR_MIN, MPC_MAN_Y_MAX, MPC_MAN_R_MAX, MPC_MAN_P_MAX, MPC_YAW_EXPO, MPC_THR_CURVE | mc_att_control (manual/stabilized mode) |
| `multicopter_position_mode_params.c` | MPC_HOLD_DZ, MPC_HOLD_MAX_XY, MPC_HOLD_MAX_Z | mc_att_control (stick deadzone) |
| `multicopter_altitude_mode_params.c` | MPC_MAN_Y_TAU, MPC_Z_* params | StickYaw library, altitude control |
| `multicopter_position_control_limits_params.c` | MPC_THR_MAX, MPC_THR_MIN, MPC_TILTMAX_AIR, MPC_TILTMAX_LND | mc_att_control (thrust limits) |
| `multicopter_position_control_params.c` | MPC_THR_HOVER, MPC_XY_VEL_* | Hover thrust estimation |
| `multicopter_position_control_gain_params.c` | MPC_XY_P, MPC_Z_P, MPC_XY_VEL_P_ACC, etc. | Position PID gains |
| `multicopter_autonomous_params.c` | MPC_ACC_HOR_MAX, MPC_ACC_DOWN_MAX, MPC_JERK_MAX, etc. | Autonomous flight |
| `multicopter_takeoff_land_params.c` | MPC_TKO_SPEED, MPC_LAND_SPEED, etc. | Takeoff/landing |
| `multicopter_nudging_params.c` | MPC_NUDGE_SPEED, MPC_NUDGE_ACC | Mission nudging |
| `multicopter_responsiveness_params.c` | MPC_*_VEL_I_ACC, MPC_*_VEL_D_ACC | Velocity control tuning |

### Module: `mc_att_control` (Attitude Controller)
**Status in PX4_minimal**: ✅ **ENABLED**

Parameter file: `mc_att_control_params.c`

Defines **MC_*** parameters (Multicopter Attitude Control):
- MC_ROLL_P, MC_PITCH_P, MC_YAW_P (attitude P gains)
- MC_ROLLRATE_MAX, MC_PITCHRATE_MAX, MC_YAWRATE_MAX (rate limits)
- MC_AIRMODE, MC_MAN_TILT_TAU, MC_YAW_WEIGHT

**BUT REQUIRES** these MPC_* parameters from mc_pos_control:
- MPC_MAN_TILT_MAX
- MPC_MANTHR_MIN
- MPC_THR_MAX
- MPC_THR_HOVER
- MPC_THR_CURVE
- MPC_YAW_EXPO
- MPC_HOLD_DZ

### Module: `mc_rate_control` (Rate Controller)
**Status in PX4_minimal**: ✅ **ENABLED**

Parameter file: `mc_rate_control_params.c`

Defines **MC_*RATE*** parameters (Rate PID gains):
- MC_ROLLRATE_P/I/D/K/FF
- MC_PITCHRATE_P/I/D/K/FF
- MC_YAWRATE_P/I/D/K/FF
- MC_*R_INT_LIM (integrator limits)
- MC_ACRO_* (acro mode parameters)

**Does NOT require** MPC_* parameters (self-contained).

### Library: `StickYaw`
**Status**: ✅ **ENABLED** (dependency of mc_att_control)

**REQUIRES**:
- MPC_MAN_Y_MAX (from multicopter_altitude_mode_params.c)
- MPC_MAN_Y_TAU (from multicopter_altitude_mode_params.c)

## The Dependency Chain

```
mc_att_control (ENABLED)
    ├── Requires: AttitudeControl library ✅
    ├── Requires: StickYaw library ✅
    │   └── Requires: MPC_MAN_Y_MAX, MPC_MAN_Y_TAU ❌ (from mc_pos_control)
    └── Requires MPC parameters ❌ (from mc_pos_control):
        ├── MPC_MAN_TILT_MAX
        ├── MPC_MANTHR_MIN
        ├── MPC_THR_MAX
        ├── MPC_THR_HOVER
        ├── MPC_THR_CURVE
        ├── MPC_YAW_EXPO
        └── MPC_HOLD_DZ

mc_rate_control (ENABLED)
    ├── Requires: RateControl library ✅
    └── Self-contained (no MPC dependencies) ✅
```

## Solutions

### Option 1: Enable mc_pos_control (Full Solution)
**In `boards/px4/sitl/default.px4board`:**
```cmake
CONFIG_MODULES_MC_POS_CONTROL=y  # ENABLE - provides all MPC_* parameters
```

**Pros:**
- ✅ All parameters automatically available
- ✅ No manual parameter copying needed
- ✅ Future-proof (parameters updated automatically)

**Cons:**
- ❌ Adds ~15,000 lines of position control code
- ❌ Defeats the purpose of "minimal" build
- ❌ Brings in more dependencies

### Option 2: Copy Required Parameter Files (Minimal Solution) ⭐ **RECOMMENDED**
Copy ONLY the parameter files needed by mc_att_control to minimal_commander:

**Files to copy:**
1. `multicopter_stabilized_mode_params.c` → Already copied ✅
2. `multicopter_altitude_mode_params.c` → Need to copy
3. `multicopter_position_mode_params.c` → Need to copy
4. `multicopter_position_control_limits_params.c` → Need to copy
5. `multicopter_position_control_params.c` → Need to copy (for MPC_THR_HOVER)

**Update `minimal_commander/CMakeLists.txt`:**
```cmake
SRCS
    minimal_commander.cpp
    minimal_safety_checks.cpp
    minimal_commander_params.c
    mpc_stabilized_params.c           # Already added
    mpc_altitude_params.c             # Add
    mpc_position_mode_params.c        # Add
    mpc_limits_params.c               # Add
    mpc_control_params.c              # Add
```

**Pros:**
- ✅ Minimal code addition (~500 lines of parameters)
- ✅ No position control logic added
- ✅ All required parameters available

**Cons:**
- ⚠️ Manual maintenance (parameters won't auto-update)
- ⚠️ Need to copy 4-5 more files

### Option 3: Define Minimal MPC Parameters (Already Attempted)
Manually define only the parameters needed.

**Status**: ⚠️ **Incomplete** - Still missing parameters

**Current progress:**
- ✅ MPC_MAN_Y_MAX, MPC_MAN_R_MAX, MPC_MAN_P_MAX
- ✅ MPC_MAN_Y_TAU
- ✅ MPC_HOLD_DZ
- ✅ MPC_MAN_TILT_MAX
- ✅ MPC_MANTHR_MIN
- ✅ MPC_THR_HOVER
- ✅ MPC_THR_CURVE
- ✅ MPC_YAW_EXPO
- ✅ MPC_THR_MAX (just added)
- ❌ Still getting build errors

## Recommendation

**Use Option 2**: Copy the 4-5 parameter files from mc_pos_control to minimal_commander.

This provides a true "minimal" build that:
- Has working attitude and rate controllers
- Avoids 15,000 lines of position control code
- Is maintainable and understandable

## Quick Fix Command

```bash
cd /Users/gauravsinghbhati/Documents/PX4_minimal/src/modules/minimal_commander

# Copy required parameter files
cp /Users/gauravsinghbhati/Documents/PX4-Autopilot/src/modules/mc_pos_control/multicopter_altitude_mode_params.c ./mpc_altitude_params.c
cp /Users/gauravsinghbhati/Documents/PX4-Autopilot/src/modules/mc_pos_control/multicopter_position_mode_params.c ./mpc_position_mode_params.c
cp /Users/gauravsinghbhati/Documents/PX4-Autopilot/src/modules/mc_pos_control/multicopter_position_control_limits_params.c ./mpc_limits_params.c
cp /Users/gauravsinghbhati/Documents/PX4-Autopilot/src/modules/mc_pos_control/multicopter_position_control_params.c ./mpc_control_params.c
```

Then update CMakeLists.txt to include all 5 parameter files.
