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

#include "minimal_safety_checks.hpp"
#include <px4_platform_common/log.h>

MinimalSafetyChecks::MinimalSafetyChecks(ModuleParams *parent) :
    ModuleParams(parent)
{
}

bool MinimalSafetyChecks::checkAndUpdateArmingState()
{
    // Check essential systems
    _battery_ok = checkBattery();
    _power_ok = checkPower();
    _emergency_stop_clear = checkEmergencyStop();

    // All essential checks must pass
    _can_arm = _battery_ok && _power_ok && _emergency_stop_clear;

    // Log status if arming blocked
    if (!_can_arm) {
        if (!_battery_ok) {
            PX4_WARN("Arming blocked: Battery check failed");
        }
        if (!_power_ok) {
            PX4_WARN("Arming blocked: Power system check failed");
        }
        if (!_emergency_stop_clear) {
            PX4_WARN("Arming blocked: Emergency stop engaged");
        }
    }

    return _can_arm;
}

bool MinimalSafetyChecks::checkBattery()
{
    battery_status_s battery_status;

    if (_battery_status_sub.update(&battery_status)) {
        // Check minimum charge level (default 20%)
        float min_charge = _param_bat_low_thr.get();
        if (battery_status.remaining < min_charge) {
            PX4_DEBUG("Battery check failed: %.1f%% < %.1f%%",
                     (double)(battery_status.remaining * 100.0f),
                     (double)(min_charge * 100.0f));
            return false;
        }

        // Check minimum voltage (10.0V for most systems)
        if (battery_status.voltage_v < 10.0f) {
            PX4_DEBUG("Battery check failed: %.2fV < 10.0V",
                     (double)battery_status.voltage_v);
            return false;
        }

        // Check for critical battery warning
        if (battery_status.warning >= battery_status_s::WARNING_CRITICAL) {
            PX4_DEBUG("Battery check failed: Critical warning");
            return false;
        }

        return true;
    }

    // If no battery data, assume OK (for SITL/HIL)
    PX4_DEBUG("No battery status available - assuming OK (SITL mode)");
    return true;
}

bool MinimalSafetyChecks::checkPower()
{
    system_power_s power;

    if (_system_power_sub.update(&power)) {
        // Check 5V rail (must be > 4.5V)
        if (power.voltage5v_v > 0.0f && power.voltage5v_v < 4.5f) {
            PX4_DEBUG("Power check failed: 5V rail = %.2fV < 4.5V",
                     (double)power.voltage5v_v);
            return false;
        }

        // Check for USB power (optional - some systems require external power)
        // Note: Not blocking arming for USB power to allow SITL/bench testing

        return true;
    }

    // If no power monitoring, assume OK
    PX4_DEBUG("No power status available - assuming OK");
    return true;
}

bool MinimalSafetyChecks::checkEmergencyStop()
{
    // In minimal commander, we don't have a separate emergency stop input
    // This would normally check a hardware safety switch or kill switch
    // For now, always return true (clear)

    // Future enhancement: Check safety switch topic
    // safety_s safety;
    // if (_safety_sub.update(&safety)) {
    //     return !safety.safety_off;
    // }

    return true;
}

const char* MinimalSafetyChecks::getStatusString() const
{
    if (_can_arm) {
        return "All safety checks passed";
    } else if (!_battery_ok) {
        return "Battery check failed";
    } else if (!_power_ok) {
        return "Power system check failed";
    } else if (!_emergency_stop_clear) {
        return "Emergency stop engaged";
    } else {
        return "Unknown safety issue";
    }
}
