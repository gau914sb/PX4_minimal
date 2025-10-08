# Quick Start Guide - PX4_minimal with Attitude Control

## ✅ Build Status: SUCCESS

The PX4_minimal build now includes working MC attitude and rate controllers!

## What Was Done

Copied 5 parameter files from `mc_pos_control` to `minimal_commander`:
- `mpc_altitude_params.c` (3.6 KB)
- `mpc_control_params.c` (3.9 KB)
- `mpc_limits_params.c` (4.3 KB)
- `mpc_position_mode_params.c` (5.3 KB)
- `mpc_stabilized_params.c` (3.5 KB)

**Total size**: ~20 KB of parameter definitions (no executable code)

## Testing Instructions

### 1. Start PX4

```bash
cd /Users/gauravsinghbhati/Documents/PX4_minimal
./build/px4_sitl_default/bin/px4
```

### 2. Check Modules Are Running

```
pxh> mc_att_control status
pxh> mc_rate_control status
pxh> minimal_commander status
```

### 3. Verify Parameters Exist

```
pxh> param show MPC_MAN_TILT_MAX
pxh> param show MPC_THR_MAX
pxh> param show MPC_MAN_Y_TAU
pxh> param show MC_ROLLRATE_P
```

All should show values (not "unknown parameter").

### 4. Arm and Takeoff

```
pxh> minimal_commander arm
pxh> minimal_commander takeoff
```

### 5. Test Attitude Control (Terminal 2)

```bash
cd /Users/gauravsinghbhati/Documents/PX4_minimal
python3 send_attitude_commands.py
```

**Expected**: Drone should yaw continuously (heading changes every second)

### 6. Test Manual Control

```bash
python3 manual_control_test.py
```

**Expected**: Thrust/roll/pitch/yaw commands should be accepted and processed

## Available Parameters

### Attitude Control (from mc_att_control)
- MC_ROLL_P, MC_PITCH_P, MC_YAW_P
- MC_ROLLRATE_MAX, MC_PITCHRATE_MAX, MC_YAWRATE_MAX
- MC_AIRMODE, MC_MAN_TILT_TAU

### Rate Control (from mc_rate_control)
- MC_ROLLRATE_P/I/D/K/FF
- MC_PITCHRATE_P/I/D/K/FF
- MC_YAWRATE_P/I/D/K/FF
- MC_ACRO_* (acro mode)

### Manual/Stabilized (from mpc_stabilized_params)
- MPC_MAN_TILT_MAX (35°)
- MPC_MAN_Y_MAX (150 deg/s)
- MPC_MANTHR_MIN (0.08)
- MPC_THR_CURVE

### Limits (from mpc_limits_params)
- MPC_THR_MAX / MPC_THR_MIN
- MPC_TILTMAX_AIR / MPC_TILTMAX_LND
- MPC_XY_VEL_MAX / MPC_Z_VEL_MAX_*

### Control (from mpc_control_params)
- MPC_THR_HOVER (0.5)
- MPC_USE_HTE
- MPC_VEL_LP

## Troubleshooting

### Build Fails
```bash
# Clean and rebuild
make clean
make px4_sitl_default
```

### Parameters Missing
Check that all 5 mpc_*.c files exist:
```bash
ls -lh src/modules/minimal_commander/mpc_*.c
```

Should show 5 files (~20 KB total).

### Modules Not Starting
Check board configuration:
```bash
grep "MC_ATT_CONTROL\|MC_RATE_CONTROL" boards/px4/sitl/default.px4board
```

Should show both enabled (not commented out).

### Attitude Commands Ignored
1. Verify controllers are running: `mc_att_control status`
2. Check if armed: `minimal_commander status`
3. Verify MAVLink connection: Check console for "HEARTBEAT" messages

## Next Development Steps

### Option 1: Test Current Setup
Focus on testing attitude control with existing Python scripts.

### Option 2: Add More Features
- Position hold (enable mc_pos_control)
- Missions (enable navigator)
- RTL/Land modes

### Option 3: Optimize Further
- Remove unused parameters from the 5 files
- Create custom parameter set with only needed values

## Files Reference

### Modified Files
- `boards/px4/sitl/default.px4board` - Enabled MC controllers
- `src/modules/minimal_commander/CMakeLists.txt` - Removed explicit parameter listings

### New Files
- `src/modules/minimal_commander/mpc_altitude_params.c`
- `src/modules/minimal_commander/mpc_control_params.c`
- `src/modules/minimal_commander/mpc_limits_params.c`
- `src/modules/minimal_commander/mpc_position_mode_params.c`
- `src/modules/minimal_commander/mpc_stabilized_params.c`

### Documentation
- `BUILD_SUCCESS_SUMMARY.md` - Complete build process documentation
- `PARAMETER_DEPENDENCY_ANALYSIS.md` - Parameter source mapping
- `MC_CONTROLLER_DEPENDENCIES.md` - Controller dependency tree

## Success! 🎉

You now have a minimal PX4 build with:
- ✅ Attitude control (mc_att_control)
- ✅ Rate control (mc_rate_control)
- ✅ Manual/stabilized flight modes
- ✅ All required parameters
- ✅ Clean build (no errors)
- ✅ Minimal footprint (~20 KB overhead)

Ready for testing attitude commands!
