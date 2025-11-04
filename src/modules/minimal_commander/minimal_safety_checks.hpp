/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
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
 * @file minimal_safety_checks.hpp
 * Enhanced safety validation for development/research commander
 *
 * Validates essential systems + runtime safety:
 * - Battery voltage and charge level
 * - Power system health
 * - Emergency stop status
 * - CPU load monitoring
 * - Sensor health (IMU, Gyro, Accel)
 * - Manual control timeout
 * - Estimator health
 *
 * Optional: GPS, magnetometer, navigation checks (can be disabled)
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>

class MinimalSafetyChecks : public ModuleParams
{
public:
    MinimalSafetyChecks(ModuleParams *parent);
    ~MinimalSafetyChecks() = default;

    /**
     * Check if arming is allowed based on essential safety
     * @param skip_optional Skip optional checks (for emergency arming)
     * @return true if all essential safety checks pass
     */
    bool checkAndUpdateArmingState(bool skip_optional = false);

    /**
     * Runtime safety monitoring (called continuously while armed)
     * @return true if safe to continue flight
     */
    bool checkRuntimeSafety();

    /**
     * Check if vehicle can arm
     * @return true if arming allowed
     */
    bool isArmingAllowed() const { return _can_arm; }

    /**
     * Check if runtime safety is violated
     * @return true if emergency action needed
     */
    bool requiresEmergencyAction() const { return _emergency_required; }

    /**
     * Get detailed status string
     */
    const char* getStatusString() const;

    /**
     * Get failed check details
     */
    uint32_t getFailedChecks() const { return _failed_checks; }

private:
    // Essential pre-arm checks
    bool checkBattery();
    bool checkPower();
    bool checkEmergencyStop();
    
    // Enhanced pre-arm checks
    bool checkCPULoad();
    bool checkSensorHealth();
    bool checkEstimatorHealth();
    bool checkManualControl();
    
    // Runtime checks (called during flight)
    bool checkBatteryRuntime();
    bool checkCPURuntime();
    bool checkSensorRuntime();
    bool checkEstimatorRuntime();
    bool checkGeofence();

    // Failure tracking (bit flags)
    enum class SafetyCheck : uint32_t {
        BATTERY         = (1 << 0),
        POWER          = (1 << 1),
        EMERGENCY_STOP = (1 << 2),
        CPU_LOAD       = (1 << 3),
        SENSOR_HEALTH  = (1 << 4),
        ESTIMATOR      = (1 << 5),
        MANUAL_CONTROL = (1 << 6),
        GEOFENCE       = (1 << 7),
    };

    // State flags
    bool _can_arm{false};
    bool _emergency_required{false};
    uint32_t _failed_checks{0};
    
    // Individual check status
    bool _battery_ok{false};
    bool _power_ok{false};
    bool _emergency_stop_clear{true};
    bool _cpu_ok{true};
    bool _sensors_ok{true};
    bool _estimator_ok{true};
    bool _manual_control_ok{true};
    bool _geofence_ok{true};
    
    // Timing
    hrt_abstime _last_manual_control_time{0};

    // Subscriptions
    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
    uORB::Subscription _system_power_sub{ORB_ID(system_power)};
    uORB::Subscription _cpuload_sub{ORB_ID(cpuload)};
    uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};
    uORB::Subscription _sensors_status_imu_sub{ORB_ID(sensors_status_imu)};
    uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
    uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
    uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};

    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BAT_LOW_THR>) _param_bat_low_thr,
        (ParamFloat<px4::params::BAT_CRIT_THR>) _param_bat_crit_thr,
        (ParamFloat<px4::params::COM_CPU_MAX>) _param_com_cpu_max
    )
};
