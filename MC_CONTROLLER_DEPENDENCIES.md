# MC Rate and Attitude Controller Dependencies

## Summary
To enable `mc_att_control` and `mc_rate_control` in PX4_minimal, the following dependencies are required:

## CMake Dependencies

### mc_att_control
```cmake
DEPENDS
    AttitudeControl         # Attitude control library
    mathlib                 # Math utilities
    px4_work_queue         # Work queue for scheduling
    StickYaw               # Yaw stick input processing
```

### mc_rate_control
```cmake
DEPENDS
    circuit_breaker        # Circuit breaker functionality
    mathlib                # Math utilities
    RateControl            # Rate control library
    px4_work_queue         # Work queue for scheduling
```

## Required Parameters

### COM_* Parameters (Commander)
- `COM_SPOOLUP_TIME` - Motor spoolup time (s)

### MC_* Parameters (Multicopter Attitude/Rate Control)

**Attitude Control:**
- `MC_AIRMODE` - Airmode enable
- `MC_MAN_TILT_TAU` - Manual tilt time constant
- `MC_PITCHRATE_MAX` - Maximum pitch rate (deg/s)
- `MC_PITCH_P` - Pitch P gain
- `MC_ROLLRATE_MAX` - Maximum roll rate (deg/s)
- `MC_ROLL_P` - Roll P gain
- `MC_YAWRATE_MAX` - Maximum yaw rate (deg/s)
- `MC_YAW_P` - Yaw P gain
- `MC_YAW_WEIGHT` - Yaw weight

**Rate Control:**
- `MC_ACRO_EXPO` - Acro mode expo (roll/pitch)
- `MC_ACRO_EXPO_Y` - Acro mode expo (yaw)
- `MC_ACRO_P_MAX` - Acro max pitch rate
- `MC_ACRO_R_MAX` - Acro max roll rate
- `MC_ACRO_SUPEXPO` - Acro super expo (roll/pitch)
- `MC_ACRO_SUPEXPOY` - Acro super expo (yaw)
- `MC_ACRO_Y_MAX` - Acro max yaw rate
- `MC_BAT_SCALE_EN` - Battery scaling enable
- `MC_PITCHRATE_D` - Pitch rate D gain
- `MC_PITCHRATE_FF` - Pitch rate feedforward
- `MC_PITCHRATE_I` - Pitch rate I gain
- `MC_PITCHRATE_K` - Pitch rate controller gain
- `MC_PITCHRATE_P` - Pitch rate P gain
- `MC_PR_INT_LIM` - Pitch rate integrator limit
- `MC_ROLLRATE_D` - Roll rate D gain
- `MC_ROLLRATE_FF` - Roll rate feedforward
- `MC_ROLLRATE_I` - Roll rate I gain
- `MC_ROLLRATE_K` - Roll rate controller gain
- `MC_ROLLRATE_P` - Roll rate P gain
- `MC_RR_INT_LIM` - Roll rate integrator limit
- `MC_YAWRATE_D` - Yaw rate D gain
- `MC_YAWRATE_FF` - Yaw rate feedforward
- `MC_YAWRATE_I` - Yaw rate I gain
- `MC_YAWRATE_K` - Yaw rate controller gain
- `MC_YAWRATE_P` - Yaw rate P gain
- `MC_YAW_TQ_CUTOFF` - Yaw torque cutoff frequency
- `MC_YR_INT_LIM` - Yaw rate integrator limit

### MPC_* Parameters (Multicopter Position Control - used by attitude controller)
- `MPC_HOLD_DZ` - Manual stick deadzone
- `MPC_MANTHR_MIN` - Minimum manual thrust
- `MPC_MAN_TILT_MAX` - Maximum tilt angle (deg)
- `MPC_MAN_Y_MAX` - Maximum yaw rate (deg/s)
- `MPC_MAN_Y_TAU` - Yaw rate filter time constant (s)
- `MPC_THR_CURVE` - Thrust curve mode (0=linear, 1=rescale)
- `MPC_THR_HOVER` - Hover thrust
- `MPC_THR_MAX` - Maximum thrust
- `MPC_YAW_EXPO` - Yaw expo factor

## Parameter Files Needed

All MC_* parameters are already defined in:
- `/src/modules/mc_att_control/mc_att_control_params.c`
- `/src/modules/mc_rate_control/mc_rate_control_params.c`

The MPC_* parameters need to be added to minimal_commander:
- `MPC_MAN_Y_MAX`, `MPC_MAN_Y_TAU` - For StickYaw library
- `MPC_HOLD_DZ`, `MPC_MANTHR_MIN`, `MPC_MAN_TILT_MAX` - For attitude controller
- `MPC_THR_CURVE`, `MPC_THR_HOVER`, `MPC_THR_MAX`, `MPC_YAW_EXPO` - For thrust/yaw control

## Current Status in PX4_minimal

### ✅ Already Enabled
- `mathlib` - Core math library
- `px4_work_queue` - Scheduling
- `RateControl` - Rate control algorithms
- `AttitudeControl` - Attitude control algorithms

### ⚠️ Need to Verify
- `circuit_breaker` - Check if enabled in build
- `StickYaw` - Yaw stick processing library

### 📝 Parameter Status
Created files:
- `mpc_stabilized_params.c` - Contains most MPC_* parameters
- `minimal_commander_params.c` - Contains COM_SPOOLUP_TIME

Missing parameters that need to be added:
- Check if all MC_* parameters are available (they should be from mc_att_control and mc_rate_control modules)

## Build Configuration

In `boards/px4/sitl/default.px4board`:
```
CONFIG_MODULES_MC_ATT_CONTROL=y
CONFIG_MODULES_MC_RATE_CONTROL=y
```

## Testing Approach

Once all parameters are defined and build succeeds:

1. **Start PX4**: `./build/px4_sitl_default/bin/px4`
2. **ARM**: `pxh> minimal_commander arm`
3. **Enable Offboard**: `pxh> minimal_commander takeoff`
4. **Send Attitude Commands** via MAVLink:
   - SET_ATTITUDE_TARGET (message 82)
   - MANUAL_CONTROL (message 69)
5. **Verify**: Drone attitude should respond to commands

Expected result: Yaw angle changes when sending yaw rate commands.
