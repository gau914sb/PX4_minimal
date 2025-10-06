# Complete Roadmap: Writing a Minimal PX4 Commander from Scratch

## Overview

This guide provides a step-by-step roadmap to create a minimal PX4 commander that bypasses the complex arming logic and safety systems, enabling direct rate-based control for custom applications like ROS2 integration.

## Table of Contents
1. [Understanding the Existing Commander](#1-understanding-the-existing-commander)
2. [Design Goals for Minimal Commander](#2-design-goals-for-minimal-commander)
3. [Architecture Planning](#3-architecture-planning)
4. [Implementation Roadmap](#4-implementation-roadmap)
5. [File Structure Setup](#5-file-structure-setup)
6. [Core Components Implementation](#6-core-components-implementation)
7. [Integration with PX4 Build System](#7-integration-with-px4-build-system)
8. [Testing and Validation](#8-testing-and-validation)
9. [Advanced Features](#9-advanced-features)
10. [Troubleshooting Guide](#10-troubleshooting-guide)

---

## 1. Understanding the Existing Commander

### 1.1 Current Commander Complexity
The existing PX4 Commander (`src/modules/commander/Commander.cpp`) handles:

```cpp
// Complex systems the standard commander manages:
- Flight mode management (MANUAL, STABILIZED, OFFBOARD, etc.)
- Pre-flight safety checks (GPS, magnetometer, barometer)
- Arming authorization (ArmAuthorization.cpp)
- Failsafe handling (RC loss, GPS loss, battery low)
- Geofencing validation
- Mission planning integration
- Land detector integration
- VTOL transition management
- Multi-vehicle coordination
- Telemetry logging and status reporting
```

### 1.2 What We Actually Need for Minimal Flight
```cpp
// Essential functions for basic flight:
- Basic arming/disarming logic
- Vehicle status publication (armed/disarmed state)
- Control mode publication (rate control enabled)
- Safety bypass (no pre-flight checks)
- Work queue integration for real-time execution
```

### 1.3 Analysis of Current Commander Dependencies
```bash
# Dependencies we can eliminate:
src/modules/commander/Arming/ArmAuthorization/   # Complex authorization
src/modules/commander/HealthAndArmingChecks/     # Pre-flight checks
src/modules/commander/failure_detector/          # Failure detection
src/modules/commander/land_detector/             # Landing detection
src/modules/navigator/                           # Mission planning

# Dependencies we must keep:
src/lib/work_queue/                              # Real-time scheduling
src/modules/sensors/                             # Basic IMU data
src/modules/mc_rate_control/                     # Rate controller
uORB messaging system                            # Inter-module communication
```

---

## 2. Design Goals for Minimal Commander

### 2.1 Primary Objectives
- **Simplicity**: Remove all unnecessary complexity
- **Real-time Performance**: Maintain deterministic timing
- **External Control Ready**: Easy integration with ROS2/MAVLink
- **Safety Bypass**: Skip all pre-flight checks for development
- **Rate Control Focus**: Only enable rate-based control mode

### 2.2 Functional Requirements
```cpp
// Core Functions:
1. Initialize vehicle systems
2. Handle arm/disarm requests (no safety checks)
3. Publish vehicle_status (armed state)
4. Publish actuator_armed (motor enable)
5. Publish vehicle_control_mode (rate control only)
6. Accept external rate commands
7. Emergency disarm capability
```

### 2.3 Non-Requirements (Deliberately Excluded)
```cpp
// Features we intentionally remove:
- GPS integration
- Magnetometer calibration
- Barometer validation
- RC transmitter requirements
- Mission planning
- Geofencing
- Return-to-launch (RTL)
- Landing detection
- Battery monitoring
- Telemetry logging
```

---

## 3. Architecture Planning

### 3.1 System Architecture Diagram
```
┌─────────────────────────────────────────────────────────────────┐
│                    Minimal Commander Architecture               │
├─────────────────────────────────────────────────────────────────┤
│  External Control (ROS2/MAVLink/RC)                            │
│         │                                                       │
│         ▼                                                       │
│  ┌─────────────────┐    uORB Publications                      │
│  │ Minimal         │────────────────────┐                      │
│  │ Commander       │                    │                      │
│  │                 │                    ▼                      │
│  │ • Arming Logic  │         ┌─────────────────────┐           │
│  │ • State Machine │         │ uORB Topics:        │           │
│  │ • Work Queue    │         │ • vehicle_status    │           │
│  │   Integration   │         │ • actuator_armed    │           │
│  └─────────────────┘         │ • vehicle_control_  │           │
│                               │   mode              │           │
│                               └─────────────────────┘           │
│                                         │                      │
│                                         ▼                      │
│                               ┌─────────────────────┐           │
│                               │ Rate Controller     │           │
│                               │ (mc_rate_control)   │           │
│                               └─────────────────────┘           │
│                                         │                      │
│                                         ▼                      │
│                               ┌─────────────────────┐           │
│                               │ Motor Controllers   │           │
│                               │ (PWM Output)        │           │
│                               └─────────────────────┘           │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 State Machine Design
```cpp
enum MinimalCommanderState {
    INIT,           // Initializing systems
    DISARMED,       // Safe, motors disabled
    ARMED,          // Ready for flight, motors enabled
    EMERGENCY       // Emergency stop
};

// State Transitions:
INIT → DISARMED (automatic after initialization)
DISARMED → ARMED (on arm command, no safety checks)
ARMED → DISARMED (on disarm command)
ANY → EMERGENCY (on emergency stop)
EMERGENCY → DISARMED (after manual reset)
```

### 3.3 Work Queue Integration Strategy
```cpp
// Following PX4's ScheduledWorkItem pattern:
class MinimalCommander : public px4::ScheduledWorkItem {
    // Runs at 20Hz (50ms intervals) on high-priority work queue
    void Run() override {
        // 1. Process external commands
        // 2. Update state machine
        // 3. Publish uORB messages
        // 4. Handle emergency conditions
    }
};
```

---

## 4. Implementation Roadmap

### Phase 1: Basic Structure (Week 1)
```cpp
// Milestone 1.1: Create module skeleton
- [ ] Create directory structure
- [ ] Write basic CMakeLists.txt
- [ ] Implement ModuleBase inheritance
- [ ] Add to PX4 build system
- [ ] Verify compilation

// Milestone 1.2: Work queue integration
- [ ] Inherit from ScheduledWorkItem
- [ ] Implement Run() method stub
- [ ] Test work queue execution
- [ ] Add performance monitoring

// Milestone 1.3: Basic uORB integration
- [ ] Add required uORB topic includes
- [ ] Create publication objects
- [ ] Test message publishing
```

### Phase 2: Core Functionality (Week 2)
```cpp
// Milestone 2.1: State machine implementation
- [ ] Define commander states
- [ ] Implement state transitions
- [ ] Add state persistence
- [ ] Test state changes

// Milestone 2.2: Arming logic
- [ ] Implement simple arming mechanism
- [ ] Add disarming capability
- [ ] Test arm/disarm cycle
- [ ] Add emergency stop

// Milestone 2.3: uORB message publishing
- [ ] Publish vehicle_status
- [ ] Publish actuator_armed
- [ ] Publish vehicle_control_mode
- [ ] Verify message reception
```

### Phase 3: External Control Interface (Week 3)
```cpp
// Milestone 3.1: Command processing
- [ ] Subscribe to vehicle_command
- [ ] Process ARM/DISARM commands
- [ ] Handle offboard mode requests
- [ ] Add manual control support

// Milestone 3.2: Safety bypass implementation
- [ ] Disable pre-flight checks
- [ ] Skip sensor validation
- [ ] Override safety switches
- [ ] Test immediate arming

// Milestone 3.3: Rate control integration
- [ ] Configure rate-only control mode
- [ ] Test with rate controller
- [ ] Verify motor output
- [ ] Add thrust control
```

### Phase 4: Integration and Testing (Week 4)
```cpp
// Milestone 4.1: Board configuration
- [ ] Create custom board config
- [ ] Disable standard commander
- [ ] Enable minimal commander
- [ ] Test build variants

// Milestone 4.2: External interface testing
- [ ] Test MAVLink commands
- [ ] Verify ROS2 integration
- [ ] Test RC input (optional)
- [ ] Performance benchmarking

// Milestone 4.3: Documentation and cleanup
- [ ] Code documentation
- [ ] User guide creation
- [ ] Parameter documentation
- [ ] Troubleshooting guide
```

---

## 5. File Structure Setup

### 5.1 Directory Structure
```bash
src/modules/minimal_commander/
├── CMakeLists.txt                 # Build configuration
├── minimal_commander.cpp          # Main implementation
├── minimal_commander.hpp          # Header file
├── minimal_state_machine.hpp      # State management
├── minimal_parameters.hpp         # Parameter definitions
└── README.md                      # Module documentation
```

### 5.2 File Creation Checklist
```bash
# Step 1: Create directory
mkdir -p src/modules/minimal_commander

# Step 2: Create files
touch src/modules/minimal_commander/CMakeLists.txt
touch src/modules/minimal_commander/minimal_commander.hpp
touch src/modules/minimal_commander/minimal_commander.cpp
touch src/modules/minimal_commander/minimal_state_machine.hpp
touch src/modules/minimal_commander/minimal_parameters.hpp
touch src/modules/minimal_commander/README.md
```

---

## 6. Core Components Implementation

### 6.1 Header File Implementation
```cpp
// File: src/modules/minimal_commander/minimal_commander.hpp

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB includes
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

// State machine include
#include "minimal_state_machine.hpp"

using namespace time_literals;

class MinimalCommander : public ModuleBase<MinimalCommander>,
                        public ModuleParams,
                        public px4::ScheduledWorkItem
{
public:
    MinimalCommander();
    ~MinimalCommander() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;

    // Core functions
    void process_commands();
    void update_state_machine();
    void publish_status();
    void handle_emergency();

    // State management
    MinimalCommanderState _state{MinimalCommanderState::INIT};
    MinimalCommanderState _prev_state{MinimalCommanderState::INIT};

    // Timing
    hrt_abstime _last_status_update{0};
    hrt_abstime _boot_timestamp{0};
    hrt_abstime _armed_timestamp{0};

    // Publications
    uORB::Publication<vehicle_status_s>         _vehicle_status_pub{ORB_ID(vehicle_status)};
    uORB::Publication<actuator_armed_s>         _actuator_armed_pub{ORB_ID(actuator_armed)};
    uORB::Publication<vehicle_control_mode_s>   _vehicle_control_mode_pub{ORB_ID(vehicle_control_mode)};

    // Subscriptions
    uORB::SubscriptionCallback<vehicle_command_s>           _vehicle_command_sub{this, ORB_ID(vehicle_command)};
    uORB::Subscription<offboard_control_mode_s>             _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
    uORB::Subscription<manual_control_setpoint_s>           _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

    // Parameters (minimal set)
    DEFINE_PARAMETERS(
        (ParamBool<px4::params::COM_ARM_WO_GPS>) _param_com_arm_wo_gps,
        (ParamFloat<px4::params::COM_DISARM_LAND>) _param_com_disarm_land
    )

    // Statistics
    uint32_t _arm_disarm_cycles{0};
    uint32_t _emergency_stops{0};

    // Performance monitoring
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
```

### 6.2 State Machine Implementation
```cpp
// File: src/modules/minimal_commander/minimal_state_machine.hpp

#pragma once

#include <stdint.h>
#include <px4_platform_common/log.h>

enum class MinimalCommanderState : uint8_t {
    INIT = 0,
    DISARMED,
    ARMED,
    EMERGENCY
};

class MinimalStateMachine {
public:
    static bool can_transition(MinimalCommanderState from, MinimalCommanderState to) {
        switch (from) {
        case MinimalCommanderState::INIT:
            return to == MinimalCommanderState::DISARMED;

        case MinimalCommanderState::DISARMED:
            return to == MinimalCommanderState::ARMED ||
                   to == MinimalCommanderState::EMERGENCY;

        case MinimalCommanderState::ARMED:
            return to == MinimalCommanderState::DISARMED ||
                   to == MinimalCommanderState::EMERGENCY;

        case MinimalCommanderState::EMERGENCY:
            return to == MinimalCommanderState::DISARMED;

        default:
            return false;
        }
    }

    static const char* state_to_string(MinimalCommanderState state) {
        switch (state) {
        case MinimalCommanderState::INIT: return "INIT";
        case MinimalCommanderState::DISARMED: return "DISARMED";
        case MinimalCommanderState::ARMED: return "ARMED";
        case MinimalCommanderState::EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
        }
    }

    static bool is_armed(MinimalCommanderState state) {
        return state == MinimalCommanderState::ARMED;
    }

    static bool requires_disarm(MinimalCommanderState state) {
        return state == MinimalCommanderState::ARMED ||
               state == MinimalCommanderState::EMERGENCY;
    }
};
```

### 6.3 Main Implementation Structure
```cpp
// File: src/modules/minimal_commander/minimal_commander.cpp

#include "minimal_commander.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

MinimalCommander::MinimalCommander() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, wq_configurations::hp_default)
{
    _boot_timestamp = hrt_absolute_time();
}

MinimalCommander::~MinimalCommander()
{
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

bool MinimalCommander::init()
{
    // Initialize publications
    if (!_vehicle_status_pub.advertise()) {
        PX4_ERR("vehicle_status advertise failed");
        return false;
    }

    if (!_actuator_armed_pub.advertise()) {
        PX4_ERR("actuator_armed advertise failed");
        return false;
    }

    if (!_vehicle_control_mode_pub.advertise()) {
        PX4_ERR("vehicle_control_mode advertise failed");
        return false;
    }

    // Initial state transition
    _state = MinimalCommanderState::DISARMED;

    // Schedule periodic execution
    ScheduleOnInterval(50_ms); // 20 Hz

    PX4_INFO("Minimal Commander initialized");
    return true;
}

void MinimalCommander::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);

    // Core execution cycle
    process_commands();
    update_state_machine();
    publish_status();
    handle_emergency();

    perf_end(_loop_perf);
}
```

### 6.4 Command Processing Implementation
```cpp
void MinimalCommander::process_commands()
{
    // Process vehicle commands (ARM/DISARM via MAVLink)
    vehicle_command_s cmd;
    if (_vehicle_command_sub.update(&cmd)) {

        switch (cmd.command) {
        case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if (cmd.param1 > 0.5f) {
                // ARM command
                if (_state == MinimalCommanderState::DISARMED) {
                    _state = MinimalCommanderState::ARMED;
                    _armed_timestamp = hrt_absolute_time();
                    _arm_disarm_cycles++;
                    PX4_INFO("ARMED via command");
                }
            } else {
                // DISARM command
                if (MinimalStateMachine::requires_disarm(_state)) {
                    _state = MinimalCommanderState::DISARMED;
                    PX4_INFO("DISARMED via command");
                }
            }
            break;

        case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION:
            _state = MinimalCommanderState::EMERGENCY;
            _emergency_stops++;
            PX4_WARN("EMERGENCY STOP activated");
            break;

        default:
            // Ignore other commands
            break;
        }
    }

    // Process offboard control mode (auto-arm on offboard commands)
    offboard_control_mode_s offboard_mode;
    if (_offboard_control_mode_sub.update(&offboard_mode)) {
        if (_state == MinimalCommanderState::DISARMED &&
            (offboard_mode.attitude || offboard_mode.rates)) {
            _state = MinimalCommanderState::ARMED;
            _armed_timestamp = hrt_absolute_time();
            _arm_disarm_cycles++;
            PX4_INFO("ARMED via offboard control");
        }
    }

    // Process manual control (RC stick arming - optional)
    manual_control_setpoint_s manual_control;
    if (_manual_control_setpoint_sub.update(&manual_control)) {
        // Classic stick arming: throttle low + yaw right
        if (_state == MinimalCommanderState::DISARMED &&
            manual_control.z < 0.1f && manual_control.r > 0.8f) {
            _state = MinimalCommanderState::ARMED;
            _armed_timestamp = hrt_absolute_time();
            _arm_disarm_cycles++;
            PX4_INFO("ARMED via RC sticks");
        }
        // Stick disarming: throttle low + yaw left
        else if (MinimalStateMachine::is_armed(_state) &&
                 manual_control.z < 0.1f && manual_control.r < -0.8f) {
            _state = MinimalCommanderState::DISARMED;
            PX4_INFO("DISARMED via RC sticks");
        }
    }
}
```

### 6.5 Status Publishing Implementation
```cpp
void MinimalCommander::publish_status()
{
    hrt_abstime now = hrt_absolute_time();

    // Publish vehicle status
    vehicle_status_s status{};
    status.timestamp = now;
    status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
    status.arming_state = MinimalStateMachine::is_armed(_state) ?
                         vehicle_status_s::ARMING_STATE_ARMED :
                         vehicle_status_s::ARMING_STATE_STANDBY;
    status.system_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
    status.system_id = 1;
    status.component_id = 1;
    status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

    // Bypass all safety checks
    status.pre_flight_checks_pass = true;
    status.hil_state = vehicle_status_s::HIL_STATE_OFF;
    status.failsafe = false;
    status.failsafe_and_user_took_over = false;

    _vehicle_status_pub.publish(status);

    // Publish actuator armed status
    actuator_armed_s armed{};
    armed.timestamp = now;
    armed.armed = MinimalStateMachine::is_armed(_state);
    armed.ready_to_arm = (_state != MinimalCommanderState::EMERGENCY);
    armed.lockdown = (_state == MinimalCommanderState::EMERGENCY);
    armed.manual_lockdown = false;
    armed.force_failsafe = false;
    armed.in_esc_calibration_mode = false;

    _actuator_armed_pub.publish(armed);

    // Publish control mode
    vehicle_control_mode_s control_mode{};
    control_mode.timestamp = now;
    control_mode.flag_armed = MinimalStateMachine::is_armed(_state);
    control_mode.flag_external_manual_override_ok = true;
    control_mode.flag_control_manual_enabled = true;
    control_mode.flag_control_rates_enabled = true;

    // Disable higher-level controllers
    control_mode.flag_control_attitude_enabled = false;
    control_mode.flag_control_velocity_enabled = false;
    control_mode.flag_control_position_enabled = false;
    control_mode.flag_control_altitude_enabled = false;
    control_mode.flag_control_climb_rate_enabled = false;
    control_mode.flag_control_termination_enabled = false;

    _vehicle_control_mode_pub.publish(control_mode);

    _last_status_update = now;
}
```

---

## 7. Integration with PX4 Build System

### 7.1 CMakeLists.txt Configuration
```cmake
# File: src/modules/minimal_commander/CMakeLists.txt

px4_add_module(
    MODULE modules__minimal_commander
    MAIN minimal_commander
    COMPILE_FLAGS
        -Wno-cast-align # TODO: fix and enable
    SRCS
        minimal_commander.cpp
    DEPENDS
        px4_work_queue
        parameters
        perf
    )
```

### 7.2 Board Configuration
```cmake
# Create: boards/px4/custom-minimal/default.px4board

CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"

# Disable standard commander
CONFIG_MODULES_COMMANDER=n

# Enable minimal commander
CONFIG_MODULES_MINIMAL_COMMANDER=y

# Essential flight modules
CONFIG_MODULES_MC_RATE_CONTROL=y
CONFIG_MODULES_SENSORS=y
CONFIG_MODULES_EKF2=y
CONFIG_MODULES_PWM_OUT=y
CONFIG_MODULES_MAVLINK=y

# Optional for development
CONFIG_MODULES_LOGGER=y
CONFIG_SYSTEMCMDS_PARAM=y
CONFIG_SYSTEMCMDS_PERF=y

# Remove unnecessary modules
CONFIG_MODULES_MC_ATT_CONTROL=n
CONFIG_MODULES_MC_POS_CONTROL=n
CONFIG_MODULES_NAVIGATOR=n
CONFIG_MODULES_FLIGHT_MODE_MANAGER=n
CONFIG_MODULES_LAND_DETECTOR=n
CONFIG_MODULES_MANUAL_CONTROL=n
```

### 7.3 Startup Script Modification
```bash
# Create: ROMFS/px4fmu_common/init.d/rc.minimal_commander

#!/bin/sh
#
# Minimal Commander startup script
#

# Start minimal commander instead of standard commander
if param compare SYS_AUTOSTART 4001
then
    minimal_commander start
else
    echo "Minimal commander not configured for this airframe"
fi
```

### 7.4 Build Commands
```bash
# Build for custom minimal board
make px4_custom-minimal_default

# Build for existing hardware with minimal commander
make px4_fmu-v5_default CONFIG_MODULES_COMMANDER=n CONFIG_MODULES_MINIMAL_COMMANDER=y

# Clean build
make clean
make px4_custom-minimal_default
```

---

## 8. Testing and Validation

### 8.1 Unit Testing Strategy
```cpp
// File: src/modules/minimal_commander/test/test_minimal_commander.cpp

#include <gtest/gtest.h>
#include "../minimal_commander.hpp"
#include "../minimal_state_machine.hpp"

class MinimalCommanderTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Test setup
    }

    void TearDown() override {
        // Test cleanup
    }
};

TEST_F(MinimalCommanderTest, StateTransitions) {
    // Test valid state transitions
    EXPECT_TRUE(MinimalStateMachine::can_transition(
        MinimalCommanderState::DISARMED,
        MinimalCommanderState::ARMED));

    // Test invalid state transitions
    EXPECT_FALSE(MinimalStateMachine::can_transition(
        MinimalCommanderState::INIT,
        MinimalCommanderState::ARMED));
}

TEST_F(MinimalCommanderTest, ArmingLogic) {
    // Test arming conditions
    // Test disarming conditions
    // Test emergency stop
}
```

### 8.2 Integration Testing
```bash
# Test sequence for hardware validation

# 1. Build and flash
make px4_custom-minimal_default upload

# 2. Connect to console
minicom -D /dev/ttyACM0

# 3. Test basic functionality
minimal_commander start
minimal_commander status

# 4. Test arming
minimal_commander arm
# Verify: motor outputs should be enabled

# 5. Test disarming
minimal_commander disarm
# Verify: motor outputs should be disabled

# 6. Test MAVLink interface
# Use QGroundControl or pymavlink to send ARM/DISARM commands

# 7. Test rate control
# Send rate setpoint commands via MAVLink
```

### 8.3 Performance Validation
```bash
# Monitor performance on hardware
perf reset
perf top

# Check work queue timing
work_queue status

# Monitor uORB message rates
uorb top
```

### 8.4 Safety Testing Checklist
```bash
# Critical safety tests:
- [ ] Emergency stop works in all states
- [ ] Disarm command always works
- [ ] Motor outputs disabled when disarmed
- [ ] No uncommanded arming
- [ ] Proper state transitions
- [ ] uORB message integrity
- [ ] Work queue performance
- [ ] Memory usage within limits
```

---

## 9. Advanced Features

### 9.1 Parameter System Integration
```cpp
// Advanced parameter support
DEFINE_PARAMETERS(
    (ParamBool<px4::params::MC_MIN_AUTO_ARM>) _param_auto_arm,
    (ParamFloat<px4::params::MC_MIN_ARM_TIMEOUT>) _param_arm_timeout,
    (ParamInt<px4::params::MC_MIN_ARM_METHOD>) _param_arm_method
)

void MinimalCommander::parameters_update() {
    // Update parameters from storage
    updateParams();

    // Apply parameter changes
    if (_param_auto_arm.get()) {
        // Enable automatic arming on offboard commands
    }
}
```

### 9.2 Logging Integration
```cpp
// Add structured logging
#include <uORB/topics/log_message.h>

uORB::Publication<log_message_s> _log_pub{ORB_ID(log_message)};

void MinimalCommander::log_info(const char* message) {
    log_message_s log{};
    log.timestamp = hrt_absolute_time();
    log.severity = log_message_s::INFO;
    strncpy((char*)log.text, message, sizeof(log.text) - 1);
    _log_pub.publish(log);
}
```

### 9.3 Health Monitoring
```cpp
// Basic health monitoring without complex checks
void MinimalCommander::check_system_health() {
    // Monitor critical systems only
    if (!_rate_controller_active) {
        PX4_ERR("Rate controller not active - forcing disarm");
        _state = MinimalCommanderState::EMERGENCY;
    }

    if (hrt_elapsed_time(&_last_imu_update) > 100_ms) {
        PX4_WARN("IMU data stale");
        // Don't disarm, just warn
    }
}
```

### 9.4 External API Extension
```cpp
// Add custom MAVLink message support
void MinimalCommander::handle_custom_command(const vehicle_command_s &cmd) {
    switch (cmd.command) {
    case VEHICLE_CMD_USER_1: // Custom command 1
        // Handle custom functionality
        break;

    case VEHICLE_CMD_USER_2: // Custom command 2
        // Handle another custom function
        break;
    }
}
```

---

## 10. Troubleshooting Guide

### 10.1 Common Build Issues
```bash
# Issue: Module not found during build
# Solution: Check CMakeLists.txt and board configuration

# Issue: Compilation errors
# Check: Include paths, uORB topic definitions

# Issue: Linking errors
# Check: Dependencies in CMakeLists.txt

# Issue: Upload fails
# Check: Board configuration matches hardware
```

### 10.2 Runtime Issues
```bash
# Issue: Commander doesn't start
# Debug: Check work queue initialization
minimal_commander status

# Issue: Arming fails
# Debug: Check state machine and command processing
uorb print vehicle_status

# Issue: No motor output
# Debug: Check actuator_armed publication
uorb print actuator_armed

# Issue: Performance problems
# Debug: Check work queue timing
perf top
work_queue status
```

### 10.3 Integration Issues
```bash
# Issue: MAVLink commands not working
# Debug: Check vehicle_command subscription
uorb print vehicle_command

# Issue: ROS2 bridge not working
# Debug: Check uXRCE-DDS configuration
uxrce_dds_client status

# Issue: Rate controller not responding
# Debug: Check vehicle_control_mode publication
uorb print vehicle_control_mode
```

### 10.4 Debug Tools and Commands
```bash
# Essential debug commands:
minimal_commander status           # Check commander state
uorb top                          # Monitor message rates
perf top                          # Check performance
work_queue status                 # Check scheduling
listener vehicle_status           # Monitor status messages
logger start -t -b 200            # Start logging for analysis
```

---

## Conclusion

This roadmap provides a comprehensive guide to implementing a minimal PX4 commander from scratch. The key principles are:

1. **Simplicity First**: Remove all unnecessary complexity
2. **Safety Through Design**: Maintain essential safety features
3. **Performance Focus**: Ensure real-time execution
4. **Integration Ready**: Easy external control interface
5. **Maintainable Code**: Clean, documented implementation

Following this roadmap will result in a lightweight, responsive flight controller perfect for research, development, and custom applications requiring direct rate control.

### Next Steps
1. Start with Phase 1 implementation
2. Test each milestone thoroughly
3. Iterate based on your specific requirements
4. Extend with custom features as needed

The minimal commander approach provides a solid foundation for advanced flight control research while maintaining the reliability and real-time performance required for safe drone operation.

---

## Additional Resources

### Documentation References
- [PX4 Developer Guide](https://dev.px4.io/)
- [uORB Messaging](https://dev.px4.io/main/en/middleware/uorb.html)
- [Work Queue Documentation](https://dev.px4.io/main/en/concept/architecture.html#runtime-environment)
- [PX4 Module Template](https://dev.px4.io/main/en/modules/module_template.html)

### Code Examples
- Standard Commander: `src/modules/commander/Commander.cpp`
- Rate Controller: `src/modules/mc_rate_control/MulticopterRateControl.cpp`
- Work Queue Examples: `src/modules/sensors/`

### Testing Tools
- [MAVLink Tester](https://github.com/mavlink/mavlink-testing-suite)
- [PX4 SITL](https://dev.px4.io/main/en/simulation/)
- [QGroundControl](http://qgroundcontrol.com/)

This comprehensive guide should provide everything needed to successfully implement a minimal commander for PX4!
