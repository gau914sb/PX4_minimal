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
#include <drivers/drv_hrt.h>

using namespace time_literals;

MinimalSafetyChecks::MinimalSafetyChecks(ModuleParams *parent) :
    ModuleParams(parent)
{
}

bool MinimalSafetyChecks::checkAndUpdateArmingState(bool skip_optional)
{
    _failed_checks = 0;

    // Essential checks (always required)
    _battery_ok = checkBattery();
    _power_ok = checkPower();
    _emergency_stop_clear = checkEmergencyStop();

    if (!_battery_ok) _failed_checks |= (uint32_t)SafetyCheck::BATTERY;
    if (!_power_ok) _failed_checks |= (uint32_t)SafetyCheck::POWER;
    if (!_emergency_stop_clear) _failed_checks |= (uint32_t)SafetyCheck::EMERGENCY_STOP;

    // Optional checks (can be skipped for forced arming)
    if (!skip_optional) {
        _cpu_ok = checkCPULoad();
        _sensors_ok = checkSensorHealth();
        _estimator_ok = checkEstimatorHealth();
        _manual_control_ok = checkManualControl();

        if (!_cpu_ok) _failed_checks |= (uint32_t)SafetyCheck::CPU_LOAD;
        if (!_sensors_ok) _failed_checks |= (uint32_t)SafetyCheck::SENSOR_HEALTH;
        if (!_estimator_ok) _failed_checks |= (uint32_t)SafetyCheck::ESTIMATOR;
        if (!_manual_control_ok) _failed_checks |= (uint32_t)SafetyCheck::MANUAL_CONTROL;
    }

    // All essential checks must pass
    _can_arm = _battery_ok && _power_ok && _emergency_stop_clear;

    // Add optional checks if not skipped
    if (!skip_optional) {
        _can_arm = _can_arm && _cpu_ok && _sensors_ok && _estimator_ok;
        // Note: manual_control_ok is warning only, doesn't block arming
    }

    // Log status if arming blocked
    if (!_can_arm) {
        if (!_battery_ok) {
            PX4_ERR("Arming BLOCKED: Battery check failed");
        }
        if (!_power_ok) {
            PX4_ERR("Arming BLOCKED: Power system check failed");
        }
        if (!_emergency_stop_clear) {
            PX4_ERR("Arming BLOCKED: Emergency stop engaged");
        }
        if (!skip_optional) {
            if (!_cpu_ok) {
                PX4_ERR("Arming BLOCKED: CPU load too high");
            }
            if (!_sensors_ok) {
                PX4_ERR("Arming BLOCKED: Sensor health check failed");
            }
            if (!_estimator_ok) {
                PX4_ERR("Arming BLOCKED: Estimator not ready");
            }
        }
    } else {
        PX4_INFO("All pre-arm safety checks PASSED ✓");
    }

    return _can_arm;
}

bool MinimalSafetyChecks::checkRuntimeSafety()
{
    _emergency_required = false;
    _failed_checks = 0;

    // Runtime checks
    _battery_ok = checkBatteryRuntime();
    _cpu_ok = checkCPURuntime();
    _sensors_ok = checkSensorRuntime();
    _estimator_ok = checkEstimatorRuntime();
    _geofence_ok = checkGeofence();

    if (!_battery_ok) _failed_checks |= (uint32_t)SafetyCheck::BATTERY;
    if (!_cpu_ok) _failed_checks |= (uint32_t)SafetyCheck::CPU_LOAD;
    if (!_sensors_ok) _failed_checks |= (uint32_t)SafetyCheck::SENSOR_HEALTH;
    if (!_estimator_ok) _failed_checks |= (uint32_t)SafetyCheck::ESTIMATOR;
    if (!_geofence_ok) _failed_checks |= (uint32_t)SafetyCheck::GEOFENCE;

    // Critical battery requires emergency landing
    if (!_battery_ok) {
        battery_status_s battery;
        if (_battery_status_sub.copy(&battery)) {
            if (battery.warning >= battery_status_s::WARNING_CRITICAL) {
                PX4_ERR("CRITICAL BATTERY - Emergency landing required!");
                _emergency_required = true;
            }
        }
    }

    // Geofence breach requires emergency action
    if (!_geofence_ok) {
        PX4_WARN("Geofence breach detected - Return to safe zone required");
        _emergency_required = true;
    }

    return !_emergency_required;
}

bool MinimalSafetyChecks::checkBattery()
{
    battery_status_s battery_status;

    if (_battery_status_sub.update(&battery_status)) {
        // Check minimum charge level (default 20%)
        float min_charge = _param_bat_low_thr.get();
        if (battery_status.remaining < min_charge) {
            PX4_WARN("Battery too low for arming: %.1f%% < %.1f%%",
                     (double)(battery_status.remaining * 100.0f),
                     (double)(min_charge * 100.0f));
            return false;
        }

        // Check minimum voltage (10.0V for most systems)
        if (battery_status.voltage_v < 10.0f) {
            PX4_WARN("Battery voltage too low: %.2fV < 10.0V",
                     (double)battery_status.voltage_v);
            return false;
        }

        // Check for critical battery warning
        if (battery_status.warning >= battery_status_s::WARNING_CRITICAL) {
            PX4_WARN("Battery in critical state");
            return false;
        }

        return true;
    }

    // If no battery data, assume OK (for SITL/HIL)
    PX4_DEBUG("No battery status available - assuming OK (SITL mode)");
    return true;
}

bool MinimalSafetyChecks::checkBatteryRuntime()
{
    battery_status_s battery;
    
    if (_battery_status_sub.update(&battery)) {
        // Only fail if critically low (emergency landing threshold)
        float crit_charge = _param_bat_crit_thr.get();
        
        if (battery.remaining < crit_charge) {
            return false;
        }
        
        if (battery.voltage_v < 9.0f) {  // Lower threshold for runtime
            return false;
        }
    }
    
    return true;
}

bool MinimalSafetyChecks::checkPower()
{
    system_power_s power;

    if (_system_power_sub.update(&power)) {
        // Check 5V rail (must be > 4.5V)
        if (power.voltage5v_v > 0.0f && power.voltage5v_v < 4.5f) {
            PX4_WARN("5V rail voltage too low: %.2fV < 4.5V",
                     (double)power.voltage5v_v);
            return false;
        }

        return true;
    }

    // If no power monitoring, assume OK
    return true;
}

bool MinimalSafetyChecks::checkEmergencyStop()
{
    // Future: Check hardware safety switch
    // For now, always return true (clear)
    return true;
}

bool MinimalSafetyChecks::checkCPULoad()
{
    cpuload_s cpu;
    
    if (_cpuload_sub.update(&cpu)) {
        float max_load = _param_com_cpu_max.get() / 100.0f;  // Convert percentage
        
        if (cpu.load > max_load) {
            PX4_WARN("CPU load too high: %.1f%% > %.1f%%",
                     (double)(cpu.load * 100.0f),
                     (double)(max_load * 100.0f));
            return false;
        }
    }
    
    return true;
}

bool MinimalSafetyChecks::checkCPURuntime()
{
    cpuload_s cpu;
    
    if (_cpuload_sub.update(&cpu)) {
        // Allow higher threshold during flight (95%)
        if (cpu.load > 0.95f) {
            PX4_ERR("CPU critically overloaded: %.1f%%", (double)(cpu.load * 100.0f));
            return false;
        }
    }
    
    return true;
}

bool MinimalSafetyChecks::checkSensorHealth()
{
    vehicle_imu_status_s imu_status;
    
    if (_vehicle_imu_status_sub.update(&imu_status)) {
        // Check for sensor errors
        if (imu_status.accel_error_count > 100) {
            PX4_WARN("Accelerometer errors detected: %u", imu_status.accel_error_count);
            return false;
        }
        
        if (imu_status.gyro_error_count > 100) {
            PX4_WARN("Gyroscope errors detected: %u", imu_status.gyro_error_count);
            return false;
        }
        
        // Check sensor data rate (should be publishing)
        hrt_abstime now = hrt_absolute_time();
        if (now - imu_status.timestamp > 100_ms) {
            PX4_WARN("IMU data stale");
            return false;
        }
    }
    
    return true;
}

bool MinimalSafetyChecks::checkSensorRuntime()
{
    // Same as pre-arm check
    return checkSensorHealth();
}

bool MinimalSafetyChecks::checkEstimatorHealth()
{
    estimator_status_s est;
    
    if (_estimator_status_sub.update(&est)) {
        // Check if estimator has converged
        if (!est.attitude_estimator_initialized) {
            PX4_WARN("Attitude estimator not initialized");
            return false;
        }
        
        // Check innovation test ratios (should be < 1.0 for healthy estimation)
        if (est.vel_test_ratio > 1.0f || est.pos_test_ratio > 1.0f) {
            PX4_WARN("Estimator innovation test failed (vel: %.2f, pos: %.2f)",
                     (double)est.vel_test_ratio, (double)est.pos_test_ratio);
            return false;
        }
    }
    
    return true;
}

bool MinimalSafetyChecks::checkEstimatorRuntime()
{
    estimator_status_s est;
    
    if (_estimator_status_sub.update(&est)) {
        // Allow higher thresholds during flight
        if (est.vel_test_ratio > 2.0f || est.pos_test_ratio > 2.0f) {
            PX4_ERR("Estimator diverging (vel: %.2f, pos: %.2f)",
                    (double)est.vel_test_ratio, (double)est.pos_test_ratio);
            return false;
        }
    }
    
    return true;
}

bool MinimalSafetyChecks::checkManualControl()
{
    manual_control_setpoint_s manual;
    
    if (_manual_control_sub.update(&manual)) {
        _last_manual_control_time = manual.timestamp;
        return true;
    }
    
    // Check for manual control timeout (warning only)
    hrt_abstime now = hrt_absolute_time();
    if (_last_manual_control_time > 0 && (now - _last_manual_control_time) > 5_s) {
        PX4_WARN("Manual control signal lost (not blocking arming)");
        return false;  // Warning state, but doesn't block arming
    }
    
    return true;
}

bool MinimalSafetyChecks::checkGeofence()
{
    // Placeholder - implement geofence checking if needed
    // Would require geofence_result topic subscription
    return true;
}

const char* MinimalSafetyChecks::getStatusString() const
{
    if (_can_arm) {
        return "All safety checks PASSED ✓";
    }
    
    // Report first failed check
    if (_failed_checks & (uint32_t)SafetyCheck::BATTERY) {
        return "Battery check FAILED";
    }
    if (_failed_checks & (uint32_t)SafetyCheck::POWER) {
        return "Power system check FAILED";
    }
    if (_failed_checks & (uint32_t)SafetyCheck::EMERGENCY_STOP) {
        return "Emergency stop ENGAGED";
    }
    if (_failed_checks & (uint32_t)SafetyCheck::CPU_LOAD) {
        return "CPU load too HIGH";
    }
    if (_failed_checks & (uint32_t)SafetyCheck::SENSOR_HEALTH) {
        return "Sensor health check FAILED";
    }
    if (_failed_checks & (uint32_t)SafetyCheck::ESTIMATOR) {
        return "Estimator not READY";
    }
    if (_failed_checks & (uint32_t)SafetyCheck::MANUAL_CONTROL) {
        return "Manual control WARNING";
    }
    if (_failed_checks & (uint32_t)SafetyCheck::GEOFENCE) {
        return "Geofence BREACH";
    }
    
    return "Unknown failure";
}
