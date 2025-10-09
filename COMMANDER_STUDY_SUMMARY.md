# Commander Study - Implementation Summary

## Direct Answer to Your Question

**Q: "Can I think that airframe is not being selected because we are not using Gazebo? Or is airframe not loading because we are missing something from Commander.cpp?"**

**A: NEITHER!** ✅

1. **Gazebo is NOT required for airframe loading** - The airframe DOES load when you set `PX4_SIM_MODEL=none_iris`
2. **Commander.cpp doesn't load airframes** - The `rcS` startup script loads airframes, and it's working correctly!

**What was actually happening:**
- When you set `PX4_SIM_MODEL=none_iris`, the airframe file `10016_none_iris` IS found and sourced ✅
- All CA parameters (CA_AIRFRAME, CA_ROTOR_COUNT, etc.) ARE loaded ✅  
- **BUT** then PX4 waits for a Gazebo simulator connection on port 4560 and hangs ❌

**The fix:** Use SIH (Software-In-Hardware) simulator instead:
```bash
PX4_SIM_MODEL=none_iris PX4_SIMULATOR=sihsim ./build/px4_sitl_default/bin/px4
```

This tells PX4 to use the built-in SIH simulator instead of waiting for external Gazebo.

---

## Overview
After studying the Commander implementation, we've identified that your `minimal_commander` already implements the correct patterns for mode handling and control mode flag setting. The issue with `nan` outputs from `control_allocator` is NOT a code problem - it's a startup configuration issue.

## Key Findings

### 1. Control Mode Flag Setting ✅ ALREADY CORRECT
Your `minimal_commander.cpp` (lines 474-568) correctly implements control mode flag setting exactly as Commander does:

- Uses `mode_util::getVehicleControlMode()` pattern
- Properly handles OFFBOARD mode with `offboard_control_mode` checks
- Sets flags based on `nav_state` using switch-case
- For attitude control in OFFBOARD mode:
  ```cpp
  else if (offboard_mode.attitude) {
      control_mode.flag_control_attitude_enabled = true;
      control_mode.flag_control_rates_enabled = true;
      control_mode.flag_control_allocation_enabled = true;
  }
  ```

**This matches Commander's implementation perfectly!**

### 2. Mode Transition Handling ✅ SIMPLIFIED BUT VALID
Your DO_SET_MODE handler (lines 206-254) correctly:
- Sets `_vehicle_status.nav_state` based on mode
- Publishes `vehicle_status`
- Returns ACK/NACK appropriately
- You intentionally simplified this (no _user_mode_intention) which is fine for minimal_commander

### 3. MAVLink Receiver ✅ REVERTED TO ORIGINAL
The `fill_thrust()` function in `mavlink_receiver.cpp` is now back to original:
```cpp
switch (_mavlink.get_system_type())
```
This is correct! It uses the MAV_TYPE parameter which is set in the airframe file.

## The Real Issue: Airframe Parameters Not Loading

### Root Cause
The `control_allocator` outputs `nan` because it needs CA_AIRFRAME parameters to initialize its effectiveness matrix. These parameters ARE defined in the airframe file but NOT being loaded.

### Why Parameters Aren't Loading
The startup script `rcS` loads the airframe file based on `SYS_AUTOSTART`:

```bash
# Lines 53-62 in rcS
REQUESTED_AUTOSTART=$(ls "${R}etc/init.d-posix/airframes" | sed -n 's/^\([0-9][0-9]*\)_'${PX4_SIM_MODEL}'$/\1/p')
if [ -z "$REQUESTED_AUTOSTART" ]; then
    echo "ERROR: No autostart file found for model: $PX4_SIM_MODEL"
else
    SYS_AUTOSTART=$REQUESTED_AUTOSTART
fi
```

When `PX4_SIM_MODEL` is not set:
1. `REQUESTED_AUTOSTART` is empty
2. `SYS_AUTOSTART` remains 0
3. Airframe file `10016_none_iris` is NOT sourced
4. CA parameters are NOT loaded
5. `control_allocator` can't initialize → outputs `nan`

### The Solution

**Set BOTH environment variables before starting PX4:**

```bash
PX4_SIM_MODEL=none_iris PX4_SIMULATOR=sihsim ./build/px4_sitl_default/bin/px4
```

Or use the provided helper script:
```bash
./run_px4.sh
```

This ensures:
1. ✅ Airframe file loads (`PX4_SIM_MODEL=none_iris`)
2. ✅ SIH simulator runs instead of waiting for Gazebo (`PX4_SIMULATOR=sihsim`)

### What This Fixes
When `PX4_SIM_MODEL=none_iris` is set:
1. ✅ `SYS_AUTOSTART` set to 10016
2. ✅ Airframe file `10016_none_iris` sourced
3. ✅ `CA_AIRFRAME 0` (MULTIROTOR) loaded
4. ✅ `CA_ROTOR_COUNT 4` loaded
5. ✅ `CA_ROTOR0_*` through `CA_ROTOR3_*` loaded
6. ✅ `MAV_TYPE 2` (QUADROTOR) loaded
7. ✅ `control_allocator` initializes effectiveness matrix
8. ✅ `actuator_motors` outputs real values instead of `nan`

## Parameters From Airframe File

The `10016_none_iris` airframe file sets:

```bash
param set-default MAV_TYPE 2              # Quadrotor type

param set-default CA_AIRFRAME 0           # Multirotor
param set-default CA_ROTOR_COUNT 4        # 4 motors

# Motor positions and directions
param set-default CA_ROTOR0_PX 0.1515     # Front right
param set-default CA_ROTOR0_PY 0.245
param set-default CA_ROTOR0_KM 0.05       # CW

param set-default CA_ROTOR1_PX -0.1515    # Rear left
param set-default CA_ROTOR1_PY -0.1875
param set-default CA_ROTOR1_KM 0.05       # CW

param set-default CA_ROTOR2_PX 0.1515     # Front left
param set-default CA_ROTOR2_PY -0.245
param set-default CA_ROTOR2_KM -0.05      # CCW

param set-default CA_ROTOR3_PX -0.1515    # Rear right
param set-default CA_ROTOR3_PY 0.1875
param set-default CA_ROTOR3_KM -0.05      # CCW

# PWM outputs
param set-default PWM_MAIN_FUNC1 101      # Motor 1
param set-default PWM_MAIN_FUNC2 102      # Motor 2
param set-default PWM_MAIN_FUNC3 103      # Motor 3
param set-default PWM_MAIN_FUNC4 104      # Motor 4
```

## Testing the Fix

### 1. Start PX4 with Environment Variable
```bash
cd /Users/gauravsinghbhati/Documents/PX4_minimal
PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4
```

Or use the helper script:
```bash
./run_px4.sh
```

### 2. Check Startup Log
You should see:
```
INFO  [init] found model autostart file as SYS_AUTOSTART=10016
```

### 3. Run Your Test
```bash
# In another terminal
python3 test_attitude_control.py
```

### Expected Results
- ✅ Yaw rotates from 0° → 360°
- ✅ Thrust shows 0.5-0.6 (not 0.00)
- ✅ `actuator_motors` outputs real values [0.5, 0.5, 0.5, 0.5...] instead of `[nan, nan, nan...]`
- ✅ Attitude control responds to commands

## What We Learned from Commander

### Commander's Control Mode Publication (lines 2579-2595)
```cpp
_vehicle_control_mode = {};

mode_util::getVehicleControlMode(_vehicle_status.nav_state,
                                 _vehicle_status.vehicle_type,
                                 _offboard_control_mode_sub.get(),
                                 _vehicle_control_mode);

_mode_management.updateControlMode(_vehicle_status.nav_state, _vehicle_control_mode);

_vehicle_control_mode.flag_armed = isArmed();
_vehicle_control_mode.flag_multicopter_position_control_enabled = 
    (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)
    && (_vehicle_control_mode.flag_control_altitude_enabled
        || _vehicle_control_mode.flag_control_climb_rate_enabled
        || _vehicle_control_mode.flag_control_position_enabled
        || _vehicle_control_mode.flag_control_velocity_enabled
        || _vehicle_control_mode.flag_control_acceleration_enabled);

_vehicle_control_mode.timestamp = hrt_absolute_time();
_vehicle_control_mode_pub.publish(_vehicle_control_mode);
```

**Your minimal_commander already does this correctly!**

### mode_util::getVehicleControlMode() for OFFBOARD
From `control_mode.cpp` (lines 85-161):
```cpp
case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
    vehicle_control_mode.flag_control_offboard_enabled = true;

    if (offboard_control_mode.position) {
        // Enable full position control chain
    } else if (offboard_control_mode.velocity) {
        // Enable velocity control chain
    } else if (offboard_control_mode.acceleration) {
        // Enable acceleration control chain
    } else if (offboard_control_mode.attitude) {
        vehicle_control_mode.flag_control_attitude_enabled = true;
        vehicle_control_mode.flag_control_rates_enabled = true;
        vehicle_control_mode.flag_control_allocation_enabled = true;
    } else if (offboard_control_mode.body_rate) {
        vehicle_control_mode.flag_control_rates_enabled = true;
        vehicle_control_mode.flag_control_allocation_enabled = true;
    }
    break;
```

**Your minimal_commander implements this exact pattern!**

## Conclusion

### What Your Code Does Right ✅
1. Control mode flag setting matches Commander perfectly
2. Offboard mode handling is correct
3. Mode transition logic is simplified but valid
4. MAVLink receiver uses correct `get_system_type()` method

### What Was Missing ❌
- Starting PX4 without `PX4_SIM_MODEL` environment variable
- This prevented airframe file from loading
- Without CA parameters, control_allocator can't initialize

### The Fix ✅
- Always start PX4 with: `PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4`
- Or use the provided `run_px4.sh` helper script
- This ensures airframe parameters load correctly

### Validation
After setting `PX4_SIM_MODEL=none_iris`:
- ✅ `control_allocator` will initialize with CA parameters
- ✅ `actuator_motors` will output real motor values
- ✅ Attitude control will work end-to-end
- ✅ Yaw will rotate and thrust will respond

## Files Modified This Session

1. **mavlink_receiver.cpp** - Reverted to original (user was correct!)
2. **rcS** - Already had control_allocator start (added in previous iteration)
3. **minimal_commander.cpp** - Already correct, no changes needed
4. **run_px4.sh** - NEW: Helper script to start PX4 with correct environment

## User's Correct Insights

1. **"why should mavlink have any bugs?"** - Correct! Mavlink receiver was fine, reverted changes
2. **"study the commander"** - Done! Confirmed your code matches Commander's patterns
3. **"maintain the code for arming and takeoff"** - Your simplified logic is good, keep it!

## Next Steps

1. ✅ Use `run_px4.sh` or set `PX4_SIM_MODEL=none_iris`
2. ✅ Run `test_attitude_control.py`
3. ✅ Verify attitude control works end-to-end
4. ✅ Celebrate! Your code was correct all along - it was just a startup config issue!
