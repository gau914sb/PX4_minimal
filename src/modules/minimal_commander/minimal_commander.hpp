#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

// Essential uORB topics for minimal commander
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/parameter_update.h>

// Safety infrastructure (minimal set)
#include "failsafe/failsafe.h"
#include "failure_detector/FailureDetector.hpp"
#include "Safety.hpp"
#include "worker_thread.hpp"

// Minimal safety checks (bypass position/attitude requirements)
#include "minimal_safety_checks.hpp"

class MinimalCommander : public ModuleBase<MinimalCommander>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    MinimalCommander();
    ~MinimalCommander() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static MinimalCommander *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    int init();
    int print_status() override;

    // State management (public so MinimalStateMachine can access it)
    enum class MinimalCommanderState {
        INIT,
        DISARMED,
        ARMED,
        EMERGENCY
    };

private:
    void Run() override;

    // Core processing functions
    void process_commands();
    void process_takeoff_land_commands(const vehicle_command_s &cmd);
    void handle_takeoff_command();
    void check_battery_status();
    void check_offboard_timeout();
    void check_runtime_safety();  // NEW: Enhanced runtime safety monitoring
    void publish_status();
    void answer_command(const vehicle_command_s &cmd, uint8_t result);

    // State variable
    MinimalCommanderState _state{MinimalCommanderState::INIT};

    // Safety infrastructure (essential systems only)
    MinimalSafetyChecks    _safety_checks{this};

    // uORB subscriptions (minimal set - 5 topics verified)
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
    uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
    uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
    uORB::Subscription _system_power_sub{ORB_ID(system_power)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    // uORB publications
    uORB::Publication<vehicle_status_s>      _vehicle_status_pub{ORB_ID(vehicle_status)};
    uORB::Publication<actuator_armed_s>      _actuator_armed_pub{ORB_ID(actuator_armed)};
    uORB::Publication<vehicle_control_mode_s> _vehicle_control_mode_pub{ORB_ID(vehicle_control_mode)};
    uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

    // Internal state tracking
    vehicle_status_s _vehicle_status{};
    hrt_abstime _armed_timestamp{0};
    hrt_abstime _last_offboard_timestamp{0};
    uint16_t _arm_disarm_cycles{0};
    uint16_t _emergency_stops{0};
    uint8_t _battery_warning{battery_status_s::WARNING_NONE};
    bool _low_battery_disarm_enabled{true};

    // Performance monitoring
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

    // Parameters (minimal set)
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::COM_LOW_BAT_ACT>) _param_com_low_bat_act,
        (ParamFloat<px4::params::BAT_LOW_THR>) _param_bat_low_thr,
        (ParamInt<px4::params::COM_MINCMD_RATE>) _param_com_mincmd_rate  // Update rate in ms
    )
};
