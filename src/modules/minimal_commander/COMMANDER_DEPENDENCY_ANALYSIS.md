# Commander Parameter Dependencies Analysis

**Date**: October 8, 2025
**Goal**: Identify all parameters from `commander` that other modules depend on, to enable building without the full commander module.

---

## Problem Statement

The minimal_commander cannot be used standalone because other essential PX4 modules (like `manual_control` and `land_detector`) depend on parameters defined in the full `commander` module's parameter file (`commander_params.c`).

**Build Error Example:**
```
fatal error: no member named 'COM_RC_IN_MODE' in 'px4::params'
```

---

## Parameters Required by Other Modules

### 1. **manual_control Module** (CRITICAL - Cannot disable)

**Required Parameters from commander_params.c:**

| Parameter | Type | Default | Purpose |
|-----------|------|---------|---------|
| `COM_RC_IN_MODE` | INT32 | 3 | RC input mode selection |
| `COM_RC_LOSS_T` | FLOAT | 0.5s | RC loss timeout |
| `COM_RC_STICK_OV` | FLOAT | 30% | Stick override threshold |
| `COM_RC_ARM_HYST` | INT32 | 1000ms | RC arm/disarm duration |
| `COM_ARM_SWISBTN` | BOOL | false | Arm with switch/button |
| `COM_FLTMODE1` through `COM_FLTMODE6` | INT32 | varies | Flight mode mappings |

**Usage:**
```cpp
// From ManualControl.hpp
DEFINE_PARAMETERS(
    (ParamInt<px4::params::COM_RC_IN_MODE>) _param_com_rc_in_mode,
    (ParamFloat<px4::params::COM_RC_LOSS_T>) _param_com_rc_loss_t,
    (ParamFloat<px4::params::COM_RC_STICK_OV>) _param_com_rc_stick_ov,
    (ParamBool<px4::params::MAN_ARM_GESTURE>) _param_man_arm_gesture,
    (ParamFloat<px4::params::MAN_KILL_GEST_T>) _param_man_kill_gest_t,
    (ParamInt<px4::params::COM_RC_ARM_HYST>) _param_com_rc_arm_hyst,
    (ParamBool<px4::params::COM_ARM_SWISBTN>) _param_com_arm_swisbtn,
    (ParamInt<px4::params::COM_FLTMODE1>) _param_fltmode_1,
    (ParamInt<px4::params::COM_FLTMODE2>) _param_fltmode_2,
    (ParamInt<px4::params::COM_FLTMODE3>) _param_fltmode_3,
    (ParamInt<px4::params::COM_FLTMODE4>) _param_fltmode_4,
    (ParamInt<px4::params::COM_FLTMODE5>) _param_fltmode_5,
    (ParamInt<px4::params::COM_FLTMODE6>) _param_fltmode_6
)
```

**COM_RC_IN_MODE Values:**
- 0: RC Transmitter only
- 1: Joystick only
- 2: RC and Joystick with fallback
- 3: RC or Joystick keep first (DEFAULT)
- 4: Stick input disabled
- 5: RC priority, Joystick fallback
- 6: Joystick priority, RC fallback

---

### 2. **land_detector Module** (For Rover only)

**Required Parameters from navigator module:**

| Parameter | Type | Default | Purpose |
|-----------|------|---------|---------|
| `NAV_ACC_RAD` | FLOAT | 10.0m | Navigation acceptance radius |

**Usage:**
```cpp
// From RoverLandDetector.h
DEFINE_PARAMETERS(
    (ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad
)

// Used to detect if rover reached waypoint
return distance_to_curr_wp < _param_nav_acc_rad.get() && !position_setpoint_triplet.next.valid;
```

**Note:** This is only used in `RoverLandDetector`, not in `MulticopterLandDetector`. Since we're focused on multicopters, this is less critical.

---

## Solution Approaches

### **Approach 1: Create minimal_commander_params.c** ✅ RECOMMENDED

Extract only the necessary parameters from commander into a new parameter file for minimal_commander.

**Steps:**
1. Create `src/modules/minimal_commander/minimal_commander_params.c`
2. Copy the 13 COM_* parameters listed above
3. Modify `CMakeLists.txt` to include the parameter file
4. Rebuild - other modules will now find these parameters

**Advantages:**
- ✅ Clean separation
- ✅ Minimal code duplication (~200 lines vs 1052 lines in full commander_params.c)
- ✅ Documents exactly what minimal_commander provides
- ✅ No need to keep full commander module

**File Structure:**
```
src/modules/minimal_commander/
├── CMakeLists.txt
├── minimal_commander.cpp
├── minimal_commander.hpp
├── minimal_commander_params.c  ← NEW FILE
├── minimal_safety_checks.cpp
├── minimal_safety_checks.hpp
└── Kconfig
```

---

### **Approach 2: Keep Commander for Parameters Only** (Current)

Keep the full commander module compiled but only use it as a parameter provider.

**Advantages:**
- ✅ No code changes needed
- ✅ Guaranteed parameter compatibility

**Disadvantages:**
- ❌ Still compiling ~20,000 lines of unused commander code
- ❌ Binary size not reduced
- ❌ Build time not improved
- ❌ Doesn't achieve goal of minimal build

---

### **Approach 3: Disable manual_control Module**

Remove dependency by not using manual_control.

**Disadvantages:**
- ❌ Loses RC input processing
- ❌ Loses joystick support
- ❌ Loses flight mode switching
- ❌ Makes system unusable for manual flight

**Verdict:** Not viable for research purposes.

---

## Recommended Implementation Plan

### **Phase 1: Extract Core Parameters** (30 minutes)

1. Create `src/modules/minimal_commander/minimal_commander_params.c`
2. Copy these parameters from `src/modules/commander/commander_params.c`:
   - COM_RC_IN_MODE
   - COM_RC_LOSS_T
   - COM_RC_STICK_OV
   - COM_RC_ARM_HYST
   - COM_ARM_SWISBTN
   - COM_FLTMODE1 through COM_FLTMODE6
   - COM_LOW_BAT_ACT (already used in minimal_commander)
   - BAT_LOW_THR (already used in minimal_commander)

3. Update `CMakeLists.txt`:
```cmake
px4_add_module(
    MODULE modules__minimal_commander
    MAIN minimal_commander
    SRCS
        minimal_commander.cpp
        minimal_safety_checks.cpp
        minimal_commander_params.c  # Add this line
    DEPENDS
        battery
)
```

### **Phase 2: Test Build** (10 minutes)

1. Disable full commander in `default.px4board`:
```cmake
# CONFIG_MODULES_COMMANDER=y  # Disabled - using minimal_commander_params instead
CONFIG_MODULES_MINIMAL_COMMANDER=y
```

2. Clean build:
```bash
make distclean
make px4_sitl_default
```

3. Verify compilation succeeds

### **Phase 3: Runtime Testing** (20 minutes)

1. Launch SITL:
```bash
make px4_sitl_default gazebo
```

2. Test parameter access:
```bash
param show COM_RC_IN_MODE
param show COM_RC_LOSS_T
param show COM_FLTMODE1
```

3. Test RC input via manual_control module
4. Test arming/disarming

---

## Parameter Details Reference

### **COM_RC_IN_MODE** (Critical)
```c
/**
 * RC input mode
 *
 * The default value of 3 will enable RC transmitter and Joystick, respecting
 * whichever input comes first.
 *
 * @group Commander
 * @min 0
 * @max 6
 * @value 0 RC Transmitter only
 * @value 1 Joystick only
 * @value 2 RC and Joystick with fallback
 * @value 3 RC or Joystick keep first
 * @value 4 Stick input disabled
 * @value 5 RC priority, Joystick fallback
 * @value 6 Joystick priority, RC fallback
 */
PARAM_DEFINE_INT32(COM_RC_IN_MODE, 3);
```

### **COM_RC_LOSS_T**
```c
/**
 * Manual control loss timeout
 *
 * The time in seconds without a new setpoint from RC or Joystick, after which the connection is considered lost.
 * This must be kept short as the vehicle will use the last supplied setpoint until the timeout triggers.
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 35
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_RC_LOSS_T, 0.5f);
```

### **COM_RC_STICK_OV**
```c
/**
 * RC stick override threshold
 *
 * If an RC stick is moved more than by this amount the system will interpret this as
 * override request by the pilot.
 *
 * @group Commander
 * @unit %
 * @min 5
 * @max 80
 * @decimal 0
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_RC_STICK_OV, 30.0f);
```

### **COM_RC_ARM_HYST**
```c
/**
 * RC input arm/disarm command duration
 *
 * The default value of 1000 requires the stick to be held in the arm or disarm position for 1 second.
 *
 * @group Commander
 * @min 100
 * @max 1500
 * @unit ms
 */
PARAM_DEFINE_INT32(COM_RC_ARM_HYST, 1000);
```

### **COM_ARM_SWISBTN**
```c
/**
 * Arm switch is a button
 *
 * 0: Arming/disarming triggers on switch transition.
 * 1: Arming/disarming triggers on button press.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_SWISBTN, 0);
```

### **COM_FLTMODE1-6**
```c
/**
 * Flight mode 1-6
 *
 * The main features of flight modes are
 * the automatic heading control and throttle handling.
 *
 * @group Commander
 * @min -1
 * @max 10
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Orbit
 * @value 10 Takeoff
 */
PARAM_DEFINE_INT32(COM_FLTMODE1, -1);
PARAM_DEFINE_INT32(COM_FLTMODE2, -1);
PARAM_DEFINE_INT32(COM_FLTMODE3, -1);
PARAM_DEFINE_INT32(COM_FLTMODE4, -1);
PARAM_DEFINE_INT32(COM_FLTMODE5, -1);
PARAM_DEFINE_INT32(COM_FLTMODE6, -1);
```

---

## Summary

**Total Parameters to Extract:** 13
- 1 × COM_RC_IN_MODE
- 1 × COM_RC_LOSS_T
- 1 × COM_RC_STICK_OV
- 1 × COM_RC_ARM_HYST
- 1 × COM_ARM_SWISBTN
- 6 × COM_FLTMODE1-6
- 2 × Battery parameters (already in minimal_commander)

**Estimated Code:** ~200 lines (vs 1052 lines in full commander_params.c)

**Result:** After this extraction, the full commander module can be completely disabled, achieving a true minimal build.

---

## Next Steps

1. ✅ Create `minimal_commander_params.c` with extracted parameters
2. ✅ Update `CMakeLists.txt`
3. ✅ Disable commander in `default.px4board`
4. ✅ Build and test
5. ✅ Document in COMPILATION_REPORT.md
6. ✅ Update MINIMAL_BUILD_CONFIGURATION.md

**Expected Outcome:** Successfully build PX4 SITL without the full commander module, using only minimal_commander!

---

**Created**: October 8, 2025
**Status**: Ready for Implementation
