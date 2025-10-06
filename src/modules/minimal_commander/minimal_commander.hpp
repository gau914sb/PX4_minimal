#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

// Essential uORB topics for minimal commander
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/safety.h>

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

    int init() override;
    int print_status() override;

private:
    void Run() override;

    // Core processing functions
    void process_commands();
    void process_takeoff_land_commands(const vehicle_command_s &cmd);
    void handle_takeoff_command();
    void check_battery_status();
    void publish_status();

    // State management
    enum class MinimalCommanderState {
        INIT,
        DISARMED,
        ARMED,
        EMERGENCY
    } _state{MinimalCommanderState::INIT};

    // Safety infrastructure (essential systems only)
    Failsafe                _failsafe_instance{this};
    FailsafeBase           &_failsafe{_failsafe_instance};
    FailureDetector        _failure_detector{this};
    MinimalSafetyChecks    _safety_checks{this, _vehicle_status};
    Safety                 _safety{};
    WorkerThread          _worker_thread{};

    // uORB subscriptions (minimal set - 5 topics verified)
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
    uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
    uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
    uORB::Subscription _system_power_sub{ORB_ID(system_power)};

    // uORB publications
    uORB::Publication<vehicle_status_s>      _vehicle_status_pub{ORB_ID(vehicle_status)};
    uORB::Publication<actuator_armed_s>      _actuator_armed_pub{ORB_ID(actuator_armed)};
    uORB::Publication<vehicle_control_mode_s> _vehicle_control_mode_pub{ORB_ID(vehicle_control_mode)};

    // Internal state tracking
    vehicle_status_s _vehicle_status{};
    hrt_abstime _armed_timestamp{0};
    uint16_t _arm_disarm_cycles{0};
    uint16_t _emergency_stops{0};
    uint8_t _battery_warning{battery_status_s::WARNING_NONE};
    bool _low_battery_disarm_enabled{true};

    // Parameters (minimal set)
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::COM_LOW_BAT_ACT>) _param_com_low_bat_act,
        (ParamFloat<px4::params::BAT_LOW_THR>) _param_bat_low_thr
    )
};
