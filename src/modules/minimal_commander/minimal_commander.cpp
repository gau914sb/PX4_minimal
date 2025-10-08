/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the     _vehicle_status.arming_state = (MinimalStateMachine::is_armed(_state)) ?
                         vehicle_status_s::ARMING_STATE_ARMED :
                         vehicle_status_s::ARMING_STATE_DISARMED;ve copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file minimal_commander.cpp
 * Minimal PX4 Commander - Simplified state machine for external control
 *
 * This minimal commander bypasses complex validation (GPS, magnetometer,
 * attitude estimators) to enable rapid development and testing with
 * external control systems (ROS2, MAVLink).
 */

#include "minimal_commander.hpp"

#include <px4_platform_common/events.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

using namespace time_literals;

// Static state machine helper class
class MinimalStateMachine {
public:
    static bool can_transition(MinimalCommander::MinimalCommanderState from,
                              MinimalCommander::MinimalCommanderState to) {
        switch (from) {
        case MinimalCommander::MinimalCommanderState::INIT:
            return to == MinimalCommander::MinimalCommanderState::DISARMED;

        case MinimalCommander::MinimalCommanderState::DISARMED:
            return to == MinimalCommander::MinimalCommanderState::ARMED ||
                   to == MinimalCommander::MinimalCommanderState::EMERGENCY;

        case MinimalCommander::MinimalCommanderState::ARMED:
            return to == MinimalCommander::MinimalCommanderState::DISARMED ||
                   to == MinimalCommander::MinimalCommanderState::EMERGENCY;

        case MinimalCommander::MinimalCommanderState::EMERGENCY:
            return to == MinimalCommander::MinimalCommanderState::DISARMED;

        default:
            return false;
        }
    }

    static const char* state_to_string(MinimalCommander::MinimalCommanderState state) {
        switch (state) {
        case MinimalCommander::MinimalCommanderState::INIT: return "INIT";
        case MinimalCommander::MinimalCommanderState::DISARMED: return "DISARMED";
        case MinimalCommander::MinimalCommanderState::ARMED: return "ARMED";
        case MinimalCommander::MinimalCommanderState::EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
        }
    }

    static bool is_armed(MinimalCommander::MinimalCommanderState state) {
        return state == MinimalCommander::MinimalCommanderState::ARMED;
    }

    static bool requires_disarm(MinimalCommander::MinimalCommanderState state) {
        return state == MinimalCommander::MinimalCommanderState::ARMED ||
               state == MinimalCommander::MinimalCommanderState::EMERGENCY;
    }
};

MinimalCommander::MinimalCommander() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

MinimalCommander::~MinimalCommander()
{
    perf_free(_loop_perf);
}

int MinimalCommander::init()
{
    // Transition to DISARMED state
    if (_state == MinimalCommanderState::INIT) {
        _state = MinimalCommanderState::DISARMED;
        PX4_INFO("Minimal Commander initialized - State: DISARMED");
    }

    // Initialize vehicle status
    _vehicle_status.timestamp = hrt_absolute_time();
    _vehicle_status.system_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
    _vehicle_status.system_id = 1;
    _vehicle_status.component_id = 1;
    _vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
    _vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    _vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;

    // Schedule the first run
    ScheduleOnInterval(100_ms); // Run at 10 Hz

    return PX4_OK;
}

void MinimalCommander::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // Update parameters if changed
    parameter_update_s param_update;
    if (_parameter_update_sub.update(&param_update)) {
        updateParams();
    }

    // Core processing functions
    process_commands();
    check_offboard_timeout();
    check_battery_status();
    publish_status();

    perf_end(_loop_perf);
}

void MinimalCommander::process_commands()
{
    // Process vehicle commands (ARM/DISARM via MAVLink)
    static uint64_t last_log_time = 0;
    static int command_count = 0;

    // Log every 5 seconds to confirm this function is being called
    uint64_t now = hrt_absolute_time();
    if (now - last_log_time > 5_s) {
        PX4_INFO("process_commands() alive - checking for commands...");
        last_log_time = now;
    }

    vehicle_command_s cmd;
    if (_vehicle_command_sub.update(&cmd)) {
        command_count++;
        PX4_INFO(">>> COMMAND RECEIVED #%d: cmd=%d, target_sys=%d, target_comp=%d, param1=%.2f <<<",
                 command_count, cmd.command, cmd.target_system, cmd.target_component, (double)cmd.param1);

        switch (cmd.command) {
        case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM: {
            const int8_t arming_action = static_cast<int8_t>(lroundf(cmd.param1));
            PX4_INFO("ARM/DISARM command: action=%d, current_state=%d", arming_action, (int)_state);

            if (arming_action == vehicle_command_s::ARMING_ACTION_ARM) {
                // ARM command - Check state transition validity AND minimal safety
                if (_state == MinimalCommanderState::DISARMED) {
                    if (MinimalStateMachine::can_transition(_state, MinimalCommanderState::ARMED)) {
                        if (_safety_checks.checkAndUpdateArmingState()) {
                            _state = MinimalCommanderState::ARMED;
                            _armed_timestamp = hrt_absolute_time();
                            _arm_disarm_cycles++;
                            PX4_INFO("ARMED via command (minimal safety OK)");
                            answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
                        } else {
                            PX4_WARN("Arming BLOCKED - Essential safety check failed");
                            answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
                        }
                    } else {
                        PX4_WARN("Invalid state transition");
                        answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
                    }
                } else {
                    PX4_WARN("Arming BLOCKED - Already armed or not in DISARMED state");
                    answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
                }
            } else if (arming_action == vehicle_command_s::ARMING_ACTION_DISARM) {
                // DISARM command
                if (MinimalStateMachine::requires_disarm(_state)) {
                    _state = MinimalCommanderState::DISARMED;
                    PX4_INFO("DISARMED via command");
                    answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
                } else {
                    PX4_WARN("DISARM command ignored - already disarmed");
                    answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
                }
            } else {
                PX4_WARN("Invalid arming action: %d", arming_action);
                answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
            }
            break;
        }

        case vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION:
            _state = MinimalCommanderState::EMERGENCY;
            _emergency_stops++;
            PX4_WARN("EMERGENCY STOP activated");
            break;

        case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
        case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
            process_takeoff_land_commands(cmd);
            break;

        default:
            // Ignore other commands
            break;
        }
    }

    // Process offboard control mode (auto-arm on offboard commands)
    offboard_control_mode_s offboard_mode;
    if (_offboard_control_mode_sub.update(&offboard_mode)) {
        _last_offboard_timestamp = hrt_absolute_time();  // Track offboard activity
        if (_state == MinimalCommanderState::DISARMED &&
            (offboard_mode.attitude || offboard_mode.body_rate)) {
            if (_safety_checks.checkAndUpdateArmingState()) {
                _state = MinimalCommanderState::ARMED;
                _armed_timestamp = hrt_absolute_time();
                _arm_disarm_cycles++;
                PX4_INFO("ARMED via offboard control (minimal safety OK)");
            } else {
                PX4_WARN("Offboard arming BLOCKED - Essential safety check failed");
            }
        }
    }

    // Process manual control (RC stick arming - optional)
    manual_control_setpoint_s manual_control;
    if (_manual_control_setpoint_sub.update(&manual_control)) {
        // Classic stick arming: throttle low + yaw right
        if (_state == MinimalCommanderState::DISARMED &&
            manual_control.throttle < 0.1f && manual_control.yaw > 0.8f) {
            if (_safety_checks.checkAndUpdateArmingState()) {
                _state = MinimalCommanderState::ARMED;
                _armed_timestamp = hrt_absolute_time();
                _arm_disarm_cycles++;
                PX4_INFO("ARMED via RC sticks (minimal safety OK)");
            } else {
                PX4_WARN("RC arming BLOCKED - Essential safety check failed");
            }
        }
        // Stick disarming: throttle low + yaw left
        else if (MinimalStateMachine::is_armed(_state) &&
                 manual_control.throttle < 0.1f && manual_control.yaw < -0.8f) {
            _state = MinimalCommanderState::DISARMED;
            PX4_INFO("DISARMED via RC sticks");
        }
    }
}

void MinimalCommander::process_takeoff_land_commands(const vehicle_command_s &cmd)
{
    switch (cmd.command) {
    case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
        if (_state == MinimalCommanderState::ARMED) {
            handle_takeoff_command();
            answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
        } else {
            PX4_WARN("Takeoff rejected - Vehicle not armed");
            answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
        }
        break;

    case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
        // Simple landing - just switch to manual/stabilized
        _vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
        PX4_INFO("Landing mode - Switch to manual control");
        answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
        break;

    default:
        break;
    }
}

void MinimalCommander::check_offboard_timeout()
{
    // Monitor offboard control mode timeout (500ms)
    if (MinimalStateMachine::is_armed(_state) && _last_offboard_timestamp > 0) {
        const hrt_abstime timeout_us = 500_ms;
        const hrt_abstime now = hrt_absolute_time();

        if ((now - _last_offboard_timestamp) > timeout_us) {
            _state = MinimalCommanderState::DISARMED;
            PX4_WARN("DISARMED - Offboard control timeout (>500ms)");
            _last_offboard_timestamp = 0;  // Reset for next session
        }
    }
}

void MinimalCommander::answer_command(const vehicle_command_s &cmd, uint8_t result)
{
    vehicle_command_ack_s ack{};
    ack.timestamp = hrt_absolute_time();
    ack.command = cmd.command;
    ack.result = result;
    ack.from_external = false;
    ack.target_system = cmd.source_system;
    ack.target_component = cmd.source_component;

    _vehicle_command_ack_pub.publish(ack);
}

void MinimalCommander::handle_takeoff_command()
{
    // Simple takeoff via offboard mode activation
    if (_state == MinimalCommanderState::ARMED) {

        // Set navigation state to offboard for external control
        _vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

        PX4_INFO("Takeoff mode enabled - External controller active");
        PX4_INFO("Publish vehicle_rates_setpoint for takeoff control");
    }
}

void MinimalCommander::check_battery_status()
{
    battery_status_s battery_status;
    if (_battery_status_sub.update(&battery_status)) {

        // Update battery warning level
        _battery_warning = battery_status.warning;

        // Handle low battery conditions
        if (_low_battery_disarm_enabled && MinimalStateMachine::is_armed(_state)) {

            // Critical battery - emergency disarm
            if (battery_status.warning >= battery_status_s::WARNING_CRITICAL) {
                _state = MinimalCommanderState::EMERGENCY;
                _emergency_stops++;
                PX4_WARN("CRITICAL BATTERY - Emergency disarm! Voltage: %.2fV",
                         (double)battery_status.voltage_v);
            }

            // Low battery - controlled disarm
            else if (battery_status.warning >= battery_status_s::WARNING_LOW) {
                _state = MinimalCommanderState::DISARMED;
                PX4_WARN("LOW BATTERY - Auto disarm! Voltage: %.2fV",
                         (double)battery_status.voltage_v);
            }
        }

        // Log battery status periodically
        static hrt_abstime last_battery_log = 0;
        if (hrt_elapsed_time(&last_battery_log) > 5_s) {
            PX4_INFO("Battery: %.2fV (%.1f%%), Warning: %d",
                     (double)battery_status.voltage_v,
                     (double)(battery_status.remaining * 100.0f),
                     battery_status.warning);
            last_battery_log = hrt_absolute_time();
        }
    }
}

void MinimalCommander::publish_status()
{
    hrt_abstime now = hrt_absolute_time();

    // Publish vehicle status
    vehicle_status_s status{};
    status.timestamp = now;
    status.nav_state = _vehicle_status.nav_state;
    status.arming_state = MinimalStateMachine::is_armed(_state) ?
                         vehicle_status_s::ARMING_STATE_ARMED :
                         vehicle_status_s::ARMING_STATE_DISARMED;
    status.system_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
    status.system_id = 1;
    status.component_id = 1;
    status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

    // Set failsafe status based on battery
    status.failsafe = (_battery_warning >= battery_status_s::WARNING_LOW);

    _vehicle_status_pub.publish(status);

    // Publish actuator armed status
    actuator_armed_s armed{};
    armed.timestamp = now;
    armed.armed = MinimalStateMachine::is_armed(_state);
    armed.prearmed = false;
    armed.ready_to_arm = (_state == MinimalCommanderState::DISARMED);
    armed.lockdown = (_state == MinimalCommanderState::EMERGENCY);

    _actuator_armed_pub.publish(armed);

    // Publish vehicle control mode
    vehicle_control_mode_s control_mode{};
    control_mode.timestamp = now;
    control_mode.flag_armed = MinimalStateMachine::is_armed(_state);
    control_mode.flag_control_manual_enabled = false;
    control_mode.flag_control_auto_enabled = false;
    control_mode.flag_control_offboard_enabled =
        (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);
    control_mode.flag_control_rates_enabled = true;
    control_mode.flag_control_attitude_enabled = false;
    control_mode.flag_control_position_enabled = false;
    control_mode.flag_control_velocity_enabled = false;
    control_mode.flag_control_altitude_enabled = false;
    control_mode.flag_control_climb_rate_enabled = false;

    _vehicle_control_mode_pub.publish(control_mode);
}

int MinimalCommander::print_status()
{
    PX4_INFO("Minimal Commander Status:");
    PX4_INFO("  State: %s", MinimalStateMachine::state_to_string(_state));
    PX4_INFO("  Armed: %s", MinimalStateMachine::is_armed(_state) ? "YES" : "NO");
    PX4_INFO("  Arm/Disarm cycles: %u", _arm_disarm_cycles);
    PX4_INFO("  Emergency stops: %u", _emergency_stops);
    PX4_INFO("  Battery warning: %u", _battery_warning);

    if (_armed_timestamp > 0) {
        PX4_INFO("  Time armed: %.1f seconds",
                 (double)(hrt_elapsed_time(&_armed_timestamp) / 1000000.0));
    }

    perf_print_counter(_loop_perf);

    return 0;
}

int MinimalCommander::task_spawn(int argc, char *argv[])
{
    MinimalCommander *instance = new MinimalCommander();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init() == PX4_OK) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

MinimalCommander *MinimalCommander::instantiate(int argc, char *argv[])
{
    return new MinimalCommander();
}

int MinimalCommander::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_ERR("minimal_commander is not running");
        return 1;
    }

    MinimalCommander *instance = get_instance();

    if (argc < 1) {
        return print_usage("missing command");
    }

    // ARM command
    if (!strcmp(argv[0], "arm")) {
        if (instance->_state == MinimalCommanderState::DISARMED) {
            if (MinimalStateMachine::can_transition(instance->_state, MinimalCommanderState::ARMED)) {
                if (instance->_safety_checks.checkAndUpdateArmingState()) {
                    instance->_state = MinimalCommanderState::ARMED;
                    instance->_armed_timestamp = hrt_absolute_time();
                    instance->_arm_disarm_cycles++;
                    PX4_INFO("✅ ARMED via console command");
                    return 0;
                } else {
                    PX4_WARN("❌ Arming BLOCKED - Essential safety check failed");
                    return 1;
                }
            } else {
                PX4_WARN("❌ Invalid state transition");
                return 1;
            }
        } else {
            PX4_WARN("❌ Already armed or not in DISARMED state");
            return 1;
        }
    }

    // DISARM command
    if (!strcmp(argv[0], "disarm")) {
        if (MinimalStateMachine::requires_disarm(instance->_state)) {
            instance->_state = MinimalCommanderState::DISARMED;
            PX4_INFO("✅ DISARMED via console command");
            return 0;
        } else {
            PX4_WARN("❌ Already disarmed");
            return 1;
        }
    }

    // TAKEOFF command
    if (!strcmp(argv[0], "takeoff")) {
        if (instance->_state == MinimalCommanderState::ARMED) {
            instance->_vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
            PX4_INFO("✅ TAKEOFF mode enabled - External controller active");
            PX4_INFO("   Send offboard setpoints (attitude/thrust) to control the vehicle");
            return 0;
        } else {
            PX4_WARN("❌ Takeoff rejected - Vehicle not armed");
            PX4_WARN("   Run: minimal_commander arm");
            return 1;
        }
    }

    // STATUS command (detailed status)
    if (!strcmp(argv[0], "status")) {
        PX4_INFO("=== Minimal Commander Status ===");

        // State
        const char *state_str = "UNKNOWN";
        switch (instance->_state) {
            case MinimalCommanderState::INIT: state_str = "INIT"; break;
            case MinimalCommanderState::DISARMED: state_str = "DISARMED"; break;
            case MinimalCommanderState::ARMED: state_str = "ARMED"; break;
            case MinimalCommanderState::EMERGENCY: state_str = "EMERGENCY"; break;
        }
        PX4_INFO("State: %s", state_str);

        // Nav state
        const char *nav_state_str = "UNKNOWN";
        switch (instance->_vehicle_status.nav_state) {
            case vehicle_status_s::NAVIGATION_STATE_MANUAL: nav_state_str = "MANUAL"; break;
            case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: nav_state_str = "OFFBOARD"; break;
            case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF: nav_state_str = "AUTO_TAKEOFF"; break;
            case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND: nav_state_str = "AUTO_LAND"; break;
        }
        PX4_INFO("Navigation: %s", nav_state_str);

        // Battery
        battery_status_s battery;
        if (instance->_battery_status_sub.copy(&battery)) {
            PX4_INFO("Battery: %.2fV (%.1f%%)",
                     (double)battery.voltage_v,
                     (double)(battery.remaining * 100.0f));
        }

        // Statistics
        PX4_INFO("Arm/Disarm cycles: %d", instance->_arm_disarm_cycles);
        PX4_INFO("Emergency stops: %d", instance->_emergency_stops);

        if (instance->_armed_timestamp > 0 && instance->_state == MinimalCommanderState::ARMED) {
            uint64_t armed_duration_ms = (hrt_absolute_time() - instance->_armed_timestamp) / 1000;
            PX4_INFO("Armed for: %llu ms", armed_duration_ms);
        }

        return 0;
    }

    return print_usage("unknown command");
}

int MinimalCommander::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Minimal PX4 Commander - Simplified state machine for external control.

This module provides basic arming/disarming and state management while
bypassing complex validation (GPS, magnetometer, attitude estimators).
Designed for rapid development and testing with external control systems.

### Implementation
The minimal commander:
- Maintains 4 simple states: INIT, DISARMED, ARMED, EMERGENCY
- Checks essential safety only: battery, power, emergency stop
- Enables offboard and manual control
- Auto-arms on offboard commands
- Supports RC stick arming/disarming
- Provides console commands for direct control

### Examples
Start the minimal commander:
$ minimal_commander start

Arm the vehicle:
$ minimal_commander arm

Takeoff (enables offboard mode):
$ minimal_commander takeoff

Check detailed status:
$ minimal_commander status

Disarm the vehicle:
$ minimal_commander disarm

Stop the minimal commander:
$ minimal_commander stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("minimal_commander", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("arm", "Arm the vehicle");
    PRINT_MODULE_USAGE_COMMAND_DESCR("disarm", "Disarm the vehicle");
    PRINT_MODULE_USAGE_COMMAND_DESCR("takeoff", "Enable takeoff/offboard mode");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Show detailed status");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int minimal_commander_main(int argc, char *argv[])
{
    return MinimalCommander::main(argc, argv);
}
