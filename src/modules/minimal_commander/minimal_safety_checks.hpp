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
 * Minimal safety validation for development/research commander
 *
 * Only validates essential systems:
 * - Battery voltage and charge level
 * - Power system health
 * - Emergency stop status
 *
 * Bypasses: GPS, magnetometer, attitude estimator, navigation checks
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/vehicle_status.h>

class MinimalSafetyChecks : public ModuleParams
{
public:
    MinimalSafetyChecks(ModuleParams *parent);
    ~MinimalSafetyChecks() = default;

    /**
     * Check if arming is allowed based on essential safety
     * @return true if all essential safety checks pass
     */
    bool checkAndUpdateArmingState();

    /**
     * Check if vehicle can arm
     * @return true if arming allowed
     */
    bool isArmingAllowed() const { return _can_arm; }

    /**
     * Get detailed status string
     */
    const char* getStatusString() const;

private:
    /**
     * Check battery voltage and charge level
     * @return true if battery is OK (>20% charge, voltage > 10.0V)
     */
    bool checkBattery();

    /**
     * Check power system health
     * @return true if power system OK (5V rail > 4.5V)
     */
    bool checkPower();

    /**
     * Check emergency stop status
     * @return true if emergency stop is not engaged
     */
    bool checkEmergencyStop();

    // State flags
    bool _can_arm{false};
    bool _battery_ok{false};
    bool _power_ok{false};
    bool _emergency_stop_clear{true};

    // Subscriptions (essential only)
    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
    uORB::Subscription _system_power_sub{ORB_ID(system_power)};

    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BAT_LOW_THR>) _param_bat_low_thr,
        (ParamFloat<px4::params::BAT_CRIT_THR>) _param_bat_crit_thr
    )
};
