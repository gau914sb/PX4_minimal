# Minimal Commander Operation Flow

## Table of Contents
1. [Class Structure](#class-structure)
2. [Lifecycle Overview](#lifecycle-overview)
3. [Initialization Flow](#initialization-flow)
4. [Main Loop (Run())](#main-loop-run)
5. [Command Processing](#command-processing)
6. [State Transitions](#state-transitions)
7. [Safety Checks](#safety-checks)
8. [Detailed Function Flow](#detailed-function-flow)

---

## Class Structure

### Main Classes

```cpp
┌─────────────────────────────────────────┐
│       MinimalCommander                  │
│  (Main State Machine Controller)        │
├─────────────────────────────────────────┤
│ Inherits from:                          │
│  - ModuleParams (parameter handling)    │
│  - ScheduledWorkItem (work queue)       │
└─────────────────────────────────────────┘
           │
           │ Uses
           ▼
┌─────────────────────────────────────────┐
│    MinimalStateMachine                  │
│   (Static Helper Class)                 │
├─────────────────────────────────────────┤
│ + can_transition()                      │
│ + state_to_string()                     │
│ + is_armed()                            │
│ + requires_disarm()                     │
└─────────────────────────────────────────┘
           │
           │ Uses
           ▼
┌─────────────────────────────────────────┐
│    MinimalSafetyChecks                  │
│   (Safety Validation)                   │
├─────────────────────────────────────────┤
│ + checkAndUpdateArmingState()           │
│   - Battery check (≥20%, ≥10V)         │
│   - Power check (5V ≥4.5V)             │
│   - Emergency stop check                │
└─────────────────────────────────────────┘
```

---

## Lifecycle Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     MODULE LIFECYCLE                             │
└─────────────────────────────────────────────────────────────────┘

1. SPAWNING PHASE
   └─> task_spawn()
       └─> new MinimalCommander()  ← Constructor called
           └─> init()
               └─> ScheduleOnInterval(100ms)  ← Schedule 10Hz loop

2. RUNTIME PHASE (Continuous Loop at 10 Hz)
   └─> Run()  ← Called every 100ms by work queue
       ├─> parameter_update_s check
       ├─> process_commands()
       ├─> check_offboard_timeout()
       ├─> check_battery_status()
       └─> publish_status()

3. SHUTDOWN PHASE
   └─> should_exit() returns true
       └─> exit_and_cleanup()
           └─> ~MinimalCommander()  ← Destructor called
               └─> perf_free(_loop_perf)
```

---

## Initialization Flow

### Constructor: `MinimalCommander()`

```
START: Constructor MinimalCommander()
│
├─> Initialize base classes:
│   ├─> ModuleParams(nullptr)         [Parameter handling]
│   └─> ScheduledWorkItem(...)        [Work queue setup]
│
├─> Initialize member variables:
│   ├─> _state = INIT
│   ├─> _armed_timestamp = 0
│   ├─> _last_offboard_timestamp = 0
│   ├─> _arm_disarm_cycles = 0
│   ├─> _emergency_stops = 0
│   ├─> _battery_warning = 0
│   └─> _low_battery_disarm_enabled = true
│
├─> Initialize subscribers (uORB):
│   ├─> _vehicle_command_sub
│   ├─> _battery_status_sub
│   ├─> _parameter_update_sub
│   ├─> _offboard_control_mode_sub
│   └─> _manual_control_setpoint_sub
│
├─> Initialize publishers (uORB):
│   ├─> _vehicle_status_pub
│   ├─> _actuator_armed_pub
│   ├─> _vehicle_control_mode_pub
│   └─> _vehicle_command_ack_pub
│
├─> Create performance counter:
│   └─> _loop_perf = perf_alloc(...)
│
└─> Constructor COMPLETE
    └─> Object ready for init()
```

### Initialization: `init()`

```
START: init()
│
├─> Check current state
│   └─> if (_state == INIT)
│       └─> _state = DISARMED
│           └─> PX4_INFO("Minimal Commander initialized - State: DISARMED")
│
├─> Initialize vehicle_status structure:
│   ├─> timestamp = hrt_absolute_time()
│   ├─> system_type = VEHICLE_TYPE_ROTARY_WING
│   ├─> system_id = 1
│   ├─> component_id = 1
│   ├─> arming_state = ARMING_STATE_DISARMED
│   └─> nav_state = NAVIGATION_STATE_MANUAL
│
├─> Schedule periodic execution:
│   └─> ScheduleOnInterval(100ms)  ← 10 Hz loop
│       └─> Work queue will call Run() every 100ms
│
└─> return PX4_OK
```

### Destructor: `~MinimalCommander()`

```
START: Destructor ~MinimalCommander()
│
├─> Free performance counter:
│   └─> perf_free(_loop_perf)
│       └─> Releases performance monitoring resources
│
└─> Destructor COMPLETE
    └─> All other members auto-destructed by C++
        ├─> Subscribers automatically unsubscribe
        ├─> Publishers automatically unpublish
        └─> Base classes auto-destructed
```

---

## Main Loop (Run())

**Execution Frequency:** 10 Hz (every 100ms)
**Triggered by:** Work queue scheduler
**Performance Monitored:** Yes (via perf counters)

```
┌────────────────────────────────────────────┐
│  Run() - Main Loop (Called at 10 Hz)      │
└────────────────────────────────────────────┘
         │
         ▼
    ┌─────────────┐
    │should_exit()?│ ───YES──> ScheduleClear() ──> exit_and_cleanup() ──> RETURN
    └─────────────┘
         │ NO
         ▼
    ┌──────────────────┐
    │ perf_begin()     │  ← Start performance measurement
    └──────────────────┘
         │
         ▼
    ┌──────────────────────────────┐
    │ Check parameter updates      │
    │ _parameter_update_sub.update()│
    └──────────────────────────────┘
         │
         ▼
    ┌──────────────────────────────┐
    │ STEP 1: process_commands()   │ ← Process ARM/DISARM/MODE commands
    └──────────────────────────────┘
         │
         ▼
    ┌──────────────────────────────┐
    │ STEP 2: check_offboard       │ ← Monitor offboard control timeout
    │         _timeout()            │
    └──────────────────────────────┘
         │
         ▼
    ┌──────────────────────────────┐
    │ STEP 3: check_battery        │ ← Monitor battery status
    │         _status()             │
    └──────────────────────────────┘
         │
         ▼
    ┌──────────────────────────────┐
    │ STEP 4: publish_status()     │ ← Publish vehicle_status, actuator_armed,
    └──────────────────────────────┘   vehicle_control_mode
         │
         ▼
    ┌──────────────────┐
    │ perf_end()       │  ← End performance measurement
    └──────────────────┘
         │
         ▼
      RETURN (wait for next 100ms cycle)
```

---

## Command Processing

### `process_commands()` - Detailed Flow

```
┌──────────────────────────────────────────────────────────────┐
│              process_commands()                               │
│         (Called every 100ms from Run())                       │
└──────────────────────────────────────────────────────────────┘
         │
         ├──> LOG: Every 5 seconds "process_commands() alive..."
         │
         ▼
    ┌────────────────────────────────┐
    │ Check for new vehicle_command  │
    │ _vehicle_command_sub.update()   │
    └────────────────────────────────┘
         │
         ├─── NO NEW COMMAND ──> Continue to next section
         │
         └─── COMMAND RECEIVED ──┐
                                 ▼
         ┌────────────────────────────────────────────┐
         │  LOG: Command details (cmd, params)        │
         └────────────────────────────────────────────┘
                                 │
                                 ▼
         ┌────────────────────────────────────────────┐
         │         Switch on cmd.command              │
         └────────────────────────────────────────────┘
                │
                ├──> VEHICLE_CMD_COMPONENT_ARM_DISARM
                │    └──> [Go to ARM/DISARM Flow]
                │
                ├──> VEHICLE_CMD_DO_FLIGHTTERMINATION
                │    └──> _state = EMERGENCY
                │         _emergency_stops++
                │         LOG: "EMERGENCY STOP activated"
                │
                ├──> VEHICLE_CMD_DO_SET_MODE
                │    └──> [Go to MODE CHANGE Flow]
                │
                ├──> VEHICLE_CMD_NAV_TAKEOFF
                │    └──> process_takeoff_land_commands()
                │
                ├──> VEHICLE_CMD_NAV_LAND
                │    └──> process_takeoff_land_commands()
                │
                └──> DEFAULT: Ignore other commands
         │
         ▼
    ┌────────────────────────────────────────────┐
    │ Check offboard_control_mode updates        │
    │ _offboard_control_mode_sub.update()        │
    └────────────────────────────────────────────┘
         │
         └─── OFFBOARD ACTIVE ──> Update _last_offboard_timestamp
              │
              └─── if (DISARMED && (attitude || body_rate))
                   └──> Attempt AUTO-ARM
                        └──> if (safety_checks.pass())
                             └──> _state = ARMED
                                  LOG: "ARMED via offboard control"
         │
         ▼
    ┌────────────────────────────────────────────┐
    │ Check manual_control (RC stick arming)     │
    │ _manual_control_setpoint_sub.update()      │
    └────────────────────────────────────────────┘
         │
         └─── RC STICK ARMING DETECTED
              │ (throttle < 0.1 && yaw > 0.8)
              │
              └─── if (DISARMED)
                   └──> if (safety_checks.pass())
                        └──> _state = ARMED
                             LOG: "ARMED via RC sticks"
              │
              └─── RC STICK DISARMING DETECTED
                   │ (throttle < 0.1 && yaw < -0.8)
                   │
                   └─── if (ARMED)
                        └──> _state = DISARMED
                             LOG: "DISARMED via RC sticks"
         │
         ▼
      RETURN
```

---

## State Transitions

### ARM/DISARM Command Flow

```
┌──────────────────────────────────────────────────────────────┐
│         VEHICLE_CMD_COMPONENT_ARM_DISARM                      │
└──────────────────────────────────────────────────────────────┘
         │
         ├──> Extract: arming_action = (int8_t)lroundf(cmd.param1)
         │
         ▼
    ┌─────────────────┐
    │ arming_action?  │
    └─────────────────┘
         │
         ├──> ARMING_ACTION_ARM (1)
         │    │
         │    ▼
         │    ┌──────────────────────────┐
         │    │ Current state?           │
         │    └──────────────────────────┘
         │         │
         │         ├─── DISARMED? ──┐
         │         │                │
         │         │                ▼
         │         │    ┌────────────────────────────────┐
         │         │    │ can_transition(DISARMED, ARMED)?│
         │         │    └────────────────────────────────┘
         │         │                │
         │         │                ├─── YES ──┐
         │         │                │          │
         │         │                │          ▼
         │         │                │    ┌──────────────────────────────┐
         │         │                │    │ safety_checks.               │
         │         │                │    │ checkAndUpdateArmingState()? │
         │         │                │    └──────────────────────────────┘
         │         │                │          │
         │         │                │          ├─── PASS ──┐
         │         │                │          │           │
         │         │                │          │           ▼
         │         │                │          │    ┌──────────────────┐
         │         │                │          │    │ _state = ARMED   │
         │         │                │          │    │ _armed_timestamp │
         │         │                │          │    │ _arm_disarm_cycles++│
         │         │                │          │    │ LOG: "ARMED"     │
         │         │                │          │    │ ACK: ACCEPTED    │
         │         │                │          │    └──────────────────┘
         │         │                │          │
         │         │                │          └─── FAIL ──┐
         │         │                │                      │
         │         │                │                      ▼
         │         │                │              ┌──────────────────────┐
         │         │                │              │ WARN: "Safety failed"│
         │         │                │              │ ACK: TEMP_REJECTED   │
         │         │                │              └──────────────────────┘
         │         │                │
         │         │                └─── NO ──┐
         │         │                          │
         │         │                          ▼
         │         │                  ┌────────────────────────┐
         │         │                  │ WARN: "Invalid state"  │
         │         │                  │ ACK: DENIED            │
         │         │                  └────────────────────────┘
         │         │
         │         └─── NOT DISARMED ──┐
         │                              │
         │                              ▼
         │                      ┌────────────────────────┐
         │                      │ WARN: "Already armed"  │
         │                      │ ACK: DENIED            │
         │                      └────────────────────────┘
         │
         └──> ARMING_ACTION_DISARM (0)
              │
              ▼
              ┌──────────────────────────────┐
              │ requires_disarm(_state)?     │
              │ (ARMED or EMERGENCY?)        │
              └──────────────────────────────┘
                   │
                   ├─── YES ──┐
                   │          │
                   │          ▼
                   │    ┌──────────────────┐
                   │    │ _state = DISARMED│
                   │    │ LOG: "DISARMED"  │
                   │    │ ACK: ACCEPTED    │
                   │    └──────────────────┘
                   │
                   └─── NO ──┐
                             │
                             ▼
                     ┌──────────────────────────┐
                     │ WARN: "Already disarmed" │
                     │ ACK: DENIED              │
                     └──────────────────────────┘
```

### Mode Change Flow

```
┌──────────────────────────────────────────────────────────────┐
│              VEHICLE_CMD_DO_SET_MODE                          │
└──────────────────────────────────────────────────────────────┘
         │
         ├──> Extract: base_mode = (uint8_t)cmd.param1
         │            custom_mode = (uint32_t)cmd.param2
         │
         ▼
    ┌─────────────────────────────────────┐
    │ base_mode has                       │
    │ VEHICLE_MODE_FLAG_CUSTOM_MODE_      │
    │ ENABLED?                            │
    └─────────────────────────────────────┘
         │
         ├─── YES ──┐
         │          │
         │          ▼
         │    ┌────────────────────────┐
         │    │ Switch on custom_mode  │
         │    └────────────────────────┘
         │          │
         │          ├──> PX4_CUSTOM_MAIN_MODE_OFFBOARD (6)
         │          │    └──> _vehicle_status.nav_state = NAVIGATION_STATE_OFFBOARD
         │          │         LOG: "Switched to OFFBOARD mode"
         │          │         ACK: ACCEPTED
         │          │
         │          ├──> PX4_CUSTOM_MAIN_MODE_MANUAL (1)
         │          │    └──> _vehicle_status.nav_state = NAVIGATION_STATE_MANUAL
         │          │         LOG: "Switched to MANUAL mode"
         │          │         ACK: ACCEPTED
         │          │
         │          ├──> PX4_CUSTOM_MAIN_MODE_STABILIZED (7)
         │          │    └──> _vehicle_status.nav_state = NAVIGATION_STATE_STAB
         │          │         LOG: "Switched to STABILIZED mode"
         │          │         ACK: ACCEPTED
         │          │
         │          └──> DEFAULT (Other modes)
         │               └──> _vehicle_status.nav_state = NAVIGATION_STATE_MANUAL
         │                    LOG: "Unsupported mode, defaulting to MANUAL"
         │                    ACK: ACCEPTED
         │
         └─── NO ──┐
                   │
                   ▼
            ┌─────────────────────────┐
            │ WARN: "Non-custom mode  │
            │        not supported"   │
            │ ACK: UNSUPPORTED        │
            └─────────────────────────┘
```

---

## Safety Checks

### MinimalSafetyChecks::checkAndUpdateArmingState()

```
┌──────────────────────────────────────────────────────────────┐
│      checkAndUpdateArmingState() - Essential Checks           │
│                  (3 checks only)                              │
└──────────────────────────────────────────────────────────────┘
         │
         ▼
    ┌────────────────────────────────┐
    │ CHECK 1: Battery Status        │
    │ _battery_status_sub.copy()     │
    └────────────────────────────────┘
         │
         ├──> Check: battery.remaining >= 0.20 (20%)
         │    AND battery.voltage_v >= 10.0V
         │    │
         │    ├─── PASS ──> Continue
         │    │
         │    └─── FAIL ──> LOG: "Battery too low"
         │                  return false (ARMING BLOCKED)
         │
         ▼
    ┌────────────────────────────────┐
    │ CHECK 2: Power System          │
    │ _system_power_sub.copy()       │
    └────────────────────────────────┘
         │
         ├──> Check: system_power.voltage5v_v >= 4.5V
         │    │
         │    ├─── PASS ──> Continue
         │    │
         │    └─── FAIL ──> LOG: "5V power too low"
         │                  return false (ARMING BLOCKED)
         │
         ▼
    ┌────────────────────────────────┐
    │ CHECK 3: Emergency Stop        │
    │ (Not implemented in minimal)   │
    └────────────────────────────────┘
         │
         └──> (Future: Check safety switch)
              │
              ▼
         ┌────────────────────┐
         │ ALL CHECKS PASSED  │
         │ return true        │
         │ (ARMING ALLOWED)   │
         └────────────────────┘
```

---

## Detailed Function Flow

### 1. check_offboard_timeout()

**Purpose:** Prevent flyaways by disarming if offboard control stops
**Timeout:** 500ms
**Frequency:** Called at 10 Hz (every 100ms)

```
┌────────────────────────────────────────┐
│   check_offboard_timeout()             │
└────────────────────────────────────────┘
         │
         ▼
    ┌─────────────────────┐
    │ Is vehicle ARMED?   │ ───NO──> RETURN (nothing to check)
    └─────────────────────┘
         │ YES
         ▼
    ┌──────────────────────────────────┐
    │ Has offboard control been used?  │
    │ (_last_offboard_timestamp > 0)   │ ───NO──> RETURN (not in offboard mode)
    └──────────────────────────────────┘
         │ YES
         ▼
    ┌──────────────────────────────────┐
    │ Calculate time since last        │
    │ offboard command                 │
    │ elapsed = now - _last_offboard   │
    └──────────────────────────────────┘
         │
         ▼
    ┌─────────────────────┐
    │ elapsed > 500ms?    │ ───NO──> RETURN (still receiving commands)
    └─────────────────────┘
         │ YES (TIMEOUT!)
         ▼
    ┌──────────────────────────────────┐
    │ EMERGENCY DISARM                 │
    │ _state = DISARMED                │
    │ LOG: "Offboard timeout (>500ms)" │
    │ _last_offboard_timestamp = 0     │
    └──────────────────────────────────┘
         │
         ▼
      RETURN
```

### 2. check_battery_status()

**Purpose:** Monitor battery and auto-disarm on low/critical battery
**Frequency:** Called at 10 Hz (every 100ms)
**Logging:** Every 5 seconds

```
┌────────────────────────────────────────┐
│   check_battery_status()               │
└────────────────────────────────────────┘
         │
         ▼
    ┌─────────────────────────────┐
    │ Update battery_status       │
    │ _battery_status_sub.update()│ ───NO UPDATE──> RETURN
    └─────────────────────────────┘
         │ NEW DATA
         ▼
    ┌─────────────────────────────┐
    │ Update battery warning level│
    │ _battery_warning =          │
    │   battery_status.warning    │
    └─────────────────────────────┘
         │
         ▼
    ┌──────────────────────────────┐
    │ Is vehicle ARMED?            │ ───NO──> Skip to logging
    └──────────────────────────────┘
         │ YES
         ▼
    ┌──────────────────────────────┐
    │ Is low_battery_disarm        │
    │ enabled?                     │ ───NO──> Skip to logging
    └──────────────────────────────┘
         │ YES
         ▼
    ┌────────────────────────────────────┐
    │ Check warning level:               │
    │ battery_status.warning             │
    └────────────────────────────────────┘
         │
         ├──> WARNING_CRITICAL (highest)
         │    │
         │    ▼
         │    ┌──────────────────────────┐
         │    │ EMERGENCY DISARM         │
         │    │ _state = EMERGENCY       │
         │    │ _emergency_stops++       │
         │    │ LOG: "CRITICAL BATTERY!" │
         │    └──────────────────────────┘
         │
         ├──> WARNING_LOW
         │    │
         │    ▼
         │    ┌──────────────────────────┐
         │    │ AUTO DISARM              │
         │    │ _state = DISARMED        │
         │    │ LOG: "LOW BATTERY!"      │
         │    └──────────────────────────┘
         │
         └──> WARNING_NONE or WARNING_REMAINING
              └──> Continue (battery OK)
         │
         ▼
    ┌────────────────────────────────────┐
    │ LOGGING (every 5 seconds)          │
    │ LOG: "Battery: %.2fV (%.1f%%)"     │
    └────────────────────────────────────┘
         │
         ▼
      RETURN
```

### 3. publish_status()

**Purpose:** Publish vehicle state to other modules via uORB
**Frequency:** Called at 10 Hz (every 100ms)
**Topics Published:** 3 topics

```
┌────────────────────────────────────────┐
│   publish_status()                     │
└────────────────────────────────────────┘
         │
         ▼
    ┌────────────────────────────────────┐
    │ Get current timestamp              │
    │ now = hrt_absolute_time()          │
    └────────────────────────────────────┘
         │
         ▼
    ┌─────────────────────────────────────────────────┐
    │ PUBLISH 1: vehicle_status                       │
    ├─────────────────────────────────────────────────┤
    │ status.timestamp = now                          │
    │ status.nav_state = _vehicle_status.nav_state    │
    │ status.arming_state = is_armed() ? ARMED :      │
    │                                     DISARMED     │
    │ status.system_type = VEHICLE_TYPE_ROTARY_WING   │
    │ status.is_vtol = false  ← CRITICAL              │
    │ status.failsafe = (battery_warning >= LOW)      │
    │                                                  │
    │ _vehicle_status_pub.publish(status)             │
    └─────────────────────────────────────────────────┘
         │
         ▼
    ┌─────────────────────────────────────────────────┐
    │ PUBLISH 2: actuator_armed                       │
    ├─────────────────────────────────────────────────┤
    │ armed.timestamp = now                           │
    │ armed.armed = is_armed()                        │
    │ armed.prearmed = false                          │
    │ armed.ready_to_arm = (state == DISARMED)        │
    │ armed.lockdown = (state == EMERGENCY)           │
    │                                                  │
    │ _actuator_armed_pub.publish(armed)              │
    └─────────────────────────────────────────────────┘
         │
         ▼
    ┌─────────────────────────────────────────────────┐
    │ PUBLISH 3: vehicle_control_mode                 │
    ├─────────────────────────────────────────────────┤
    │ control_mode.timestamp = now                    │
    │ control_mode.flag_armed = is_armed()            │
    │                                                  │
    │ Switch on nav_state:                            │
    │  ┌─> MANUAL:                                    │
    │  │   - flag_control_manual_enabled = true       │
    │  │   - flag_control_attitude_enabled = true     │
    │  │   - flag_control_rates_enabled = true        │
    │  │                                               │
    │  ┌─> STABILIZED:                                │
    │  │   - Same as MANUAL                           │
    │  │                                               │
    │  ┌─> OFFBOARD:                                  │
    │  │   - flag_control_offboard_enabled = true     │
    │  │   - Check offboard_control_mode to set:      │
    │  │     • position flags                         │
    │  │     • velocity flags                         │
    │  │     • acceleration flags                     │
    │  │     • attitude flags                         │
    │  │     • body_rate flags                        │
    │  │                                               │
    │  └─> ACRO:                                      │
    │      - flag_control_manual_enabled = true       │
    │      - flag_control_rates_enabled = true        │
    │                                                  │
    │ _vehicle_control_mode_pub.publish(control_mode) │
    └─────────────────────────────────────────────────┘
         │
         ▼
      RETURN
```

---

## State Machine Diagram

```
┌──────────────────────────────────────────────────────────────┐
│              MINIMAL COMMANDER STATE MACHINE                  │
│                    (4 States Only)                            │
└──────────────────────────────────────────────────────────────┘

         ┌──────────┐
         │   INIT   │  ← Constructor creates object in INIT state
         └────┬─────┘
              │
              │ init() called
              │ Auto-transition
              ▼
         ┌──────────┐
    ┌────┤ DISARMED ├────┐
    │    └──────────┘    │
    │         ▲          │
    │         │          │
    │         │          │ ARM commands:
    │         │          │ - vehicle_command (MAVLink)
    │         │          │ - offboard_control_mode (auto-arm)
    │         │          │ - RC sticks (throttle low + yaw right)
    │         │          │ - console: minimal_commander arm
    │         │          │
    │         │          │ (If safety checks pass)
    │         │          │
    │         │          ▼
    │         │     ┌───────┐
    │         │     │ ARMED │
    │         │     └───┬───┘
    │         │         │
    │         │         │ DISARM commands:
    │         │         │ - vehicle_command (MAVLink)
    │         │         │ - RC sticks (throttle low + yaw left)
    │         │         │ - offboard timeout (>500ms)
    │         │         │ - console: minimal_commander disarm
    │         │         │ - low battery (auto-disarm)
    │         │         │
    │         └─────────┘
    │                   │
    │                   │ EMERGENCY events:
    │                   │ - VEHICLE_CMD_DO_FLIGHTTERMINATION
    │                   │ - Critical battery (<3.0V per cell)
    │                   │
    │                   ▼
    │            ┌────────────┐
    └────────────┤ EMERGENCY  │
                 └────────────┘
                       │
                       │ Manual recovery:
                       │ - console: minimal_commander disarm
                       │
                       ▼
                  (back to DISARMED)


STATE TRANSITION RULES (enforced by MinimalStateMachine):

INIT       → DISARMED     ✓ (auto-transition on init)
DISARMED   → ARMED        ✓ (if safety checks pass)
DISARMED   → EMERGENCY    ✓ (emergency command)
ARMED      → DISARMED     ✓ (normal disarm)
ARMED      → EMERGENCY    ✓ (emergency command, critical battery)
EMERGENCY  → DISARMED     ✓ (manual recovery)

All other transitions: ✗ BLOCKED
```

---

## Summary: Key Operational Points

### 1. **Threading Model**
- **Single-threaded**: Runs in work queue context
- **No locks needed**: Sequential execution guarantees
- **Scheduled**: Called every 100ms (10 Hz) by work queue

### 2. **Execution Order** (every 100ms)
```
1. Parameter updates (if any)
2. process_commands()         ← ARM/DISARM logic
3. check_offboard_timeout()   ← Safety timeout
4. check_battery_status()     ← Battery monitoring
5. publish_status()           ← State broadcast
```

### 3. **Arming Methods** (4 ways)
1. **MAVLink command** (`VEHICLE_CMD_COMPONENT_ARM_DISARM`)
2. **Offboard control** (auto-arm on offboard commands)
3. **RC sticks** (throttle low + yaw right)
4. **Console** (`minimal_commander arm`)

All methods require passing **3 essential safety checks**

### 4. **Safety Checks** (3 only)
1. Battery ≥ 20% AND ≥ 10.0V
2. 5V power rail ≥ 4.5V
3. Emergency stop not active

### 5. **Auto-Disarm Triggers**
- Offboard timeout (>500ms)
- Low battery (auto-disarm enabled)
- Critical battery (emergency)
- RC stick disarm (throttle low + yaw left)
- Manual disarm command

### 6. **Performance**
- Loop time: 100ms (10 Hz)
- Offboard timeout: 500ms
- Battery check: Every cycle
- Battery logging: Every 5 seconds

---

## Comparison: Full vs Minimal

| Feature | Full Commander | Minimal Commander |
|---------|---------------|-------------------|
| **States** | 31+ navigation states | 4 states |
| **Safety Checks** | 20+ checks | 3 checks |
| **Execution** | ~50 Hz | 10 Hz |
| **Lines of Code** | ~3,000+ | ~717 |
| **Calibration** | 9 sensor types | 0 |
| **Failsafes** | 8+ modes | Battery only |
| **Dependencies** | 50+ modules | 5 modules |

---

**End of Operation Flow Documentation**
