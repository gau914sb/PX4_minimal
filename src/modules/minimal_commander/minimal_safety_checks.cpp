/****************************************************************************/****************************************************************************

 * *

 *   Copyright (c) 2025 PX4 Development Team. All rights reserved. *   Copyright (c) 2025 PX4 Development Team. All rights reserved.

 * *

 * Redistribution and use in source and binary forms, with or without * Redistribution and use in source and binary forms, with or without

 * modification, are permitted provided that the following conditions * modification, are permitted provided that the following conditions

 * are met: * are met:

 * *

 * 1. Redistributions of source code must retain the above copyright * 1. Redistributions of source code must retain the above copyright

 *    notice, this list of conditions and the following disclaimer. *    notice, this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright * 2. Redistributions in binary form must reproduce the above copyright

 *    notice, this list of conditions and the following disclaimer in *    notice, this list of conditions and the following disclaimer in

 *    the documentation and/or other materials provided with the *    the documentation and/or other materials provided with the

 *    distribution. *    distribution.

 * 3. Neither the name PX4 nor the names of its contributors may be * 3. Neither the name PX4 nor the names of its contributors may be

 *    used to endorse or promote products derived from this software *    used to endorse or promote products derived from this software

 *    without specific prior written permission. *    without specific prior written permission.

 * *

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS

 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT

 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS

 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE

 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,

 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,

 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS

 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED

 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT

 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN

 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE

 * POSSIBILITY OF SUCH DAMAGE. * POSSIBILITY OF SUCH DAMAGE.

 * *

 ****************************************************************************/ ****************************************************************************/



#include "minimal_safety_checks.hpp"#include "minimal_safety_checks.hpp"

#include <px4_platform_common/log.h>#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>

MinimalSafetyChecks::MinimalSafetyChecks(ModuleParams *parent) :

using namespace time_literals;    ModuleParams(parent)

{

MinimalSafetyChecks::MinimalSafetyChecks(ModuleParams *parent) :}

	ModuleParams(parent)

{bool MinimalSafetyChecks::checkAndUpdateArmingState()

}{

    // Check essential systems

bool MinimalSafetyChecks::checkAndUpdateArmingState(bool skip_optional)    _battery_ok = checkBattery();

{    _power_ok = checkPower();

	// Clear previous failures    _emergency_stop_clear = checkEmergencyStop();

	_failed_checks = 0;

    // All essential checks must pass

	// ESSENTIAL checks (always required)    _can_arm = _battery_ok && _power_ok && _emergency_stop_clear;

	_battery_ok = checkBattery();

	_power_ok = checkPower();    // Log status if arming blocked

	_emergency_stop_clear = checkEmergencyStop();    if (!_can_arm) {

        if (!_battery_ok) {

	if (!_battery_ok) {            PX4_WARN("Arming blocked: Battery check failed");

		_failed_checks |= static_cast<uint32_t>(SafetyCheck::BATTERY);        }

		PX4_WARN("Pre-arm FAILED: Battery");        if (!_power_ok) {

	}            PX4_WARN("Arming blocked: Power system check failed");

        }

	if (!_power_ok) {        if (!_emergency_stop_clear) {

		_failed_checks |= static_cast<uint32_t>(SafetyCheck::POWER);            PX4_WARN("Arming blocked: Emergency stop engaged");

		PX4_WARN("Pre-arm FAILED: Power system");        }

	}    }



	if (!_emergency_stop_clear) {    return _can_arm;

		_failed_checks |= static_cast<uint32_t>(SafetyCheck::EMERGENCY_STOP);}

		PX4_WARN("Pre-arm FAILED: Emergency stop");

	}bool MinimalSafetyChecks::checkBattery()

{

	// ENHANCED checks (can be skipped for emergency arming)    battery_status_s battery_status;

	if (!skip_optional) {

		_cpu_ok = checkCPULoad();    if (_battery_status_sub.update(&battery_status)) {

		_sensors_ok = checkSensorHealth();        // Check minimum charge level (default 20%)

		_estimator_ok = checkEstimatorHealth();        float min_charge = _param_bat_low_thr.get();

		_manual_control_ok = checkManualControl();        if (battery_status.remaining < min_charge) {

            PX4_DEBUG("Battery check failed: %.1f%% < %.1f%%",

		if (!_cpu_ok) {                     (double)(battery_status.remaining * 100.0f),

			_failed_checks |= static_cast<uint32_t>(SafetyCheck::CPU_LOAD);                     (double)(min_charge * 100.0f));

			PX4_WARN("Pre-arm FAILED: CPU overload");            return false;

		}        }



		if (!_sensors_ok) {        // Check minimum voltage (10.0V for most systems)

			_failed_checks |= static_cast<uint32_t>(SafetyCheck::SENSOR_HEALTH);        if (battery_status.voltage_v < 10.0f) {

			PX4_WARN("Pre-arm FAILED: Sensor health");            PX4_DEBUG("Battery check failed: %.2fV < 10.0V",

		}                     (double)battery_status.voltage_v);

            return false;

		if (!_estimator_ok) {        }

			_failed_checks |= static_cast<uint32_t>(SafetyCheck::ESTIMATOR);

			PX4_WARN("Pre-arm FAILED: Estimator");        // Check for critical battery warning

		}        if (battery_status.warning >= battery_status_s::WARNING_CRITICAL) {

            PX4_DEBUG("Battery check failed: Critical warning");

		if (!_manual_control_ok) {            return false;

			_failed_checks |= static_cast<uint32_t>(SafetyCheck::MANUAL_CONTROL);        }

			PX4_INFO("Pre-arm WARNING: No manual control (offboard mode OK)");

			// Note: Not blocking arming for missing manual control (allows offboard-only operation)        return true;

		}    }

	}

    // If no battery data, assume OK (for SITL/HIL)

	// All essential checks must pass, enhanced checks only if not skipped    PX4_DEBUG("No battery status available - assuming OK (SITL mode)");

	_can_arm = _battery_ok && _power_ok && _emergency_stop_clear;    return true;

}

	if (!skip_optional) {

		_can_arm = _can_arm && _cpu_ok && _sensors_ok && _estimator_ok;bool MinimalSafetyChecks::checkPower()

		// Note: _manual_control_ok is informational only{

	}    system_power_s power;



	if (_can_arm) {    if (_system_power_sub.update(&power)) {

		PX4_INFO("Pre-arm checks: PASSED (failed_checks=0x%08X)", _failed_checks);        // Check 5V rail (must be > 4.5V)

	} else {        if (power.voltage5v_v > 0.0f && power.voltage5v_v < 4.5f) {

		PX4_WARN("Pre-arm checks: FAILED (failed_checks=0x%08X)", _failed_checks);            PX4_DEBUG("Power check failed: 5V rail = %.2fV < 4.5V",

	}                     (double)power.voltage5v_v);

            return false;

	return _can_arm;        }

}

        // Check for USB power (optional - some systems require external power)

bool MinimalSafetyChecks::checkRuntimeSafety()        // Note: Not blocking arming for USB power to allow SITL/bench testing

{

	// Runtime checks for armed vehicle        return true;

	bool battery_ok = checkBatteryRuntime();    }

	bool cpu_ok = checkCPURuntime();

	bool sensors_ok = checkSensorRuntime();    // If no power monitoring, assume OK

	bool estimator_ok = checkEstimatorRuntime();    PX4_DEBUG("No power status available - assuming OK");

	bool geofence_ok = checkGeofence();    return true;

}

	// Check if emergency action needed (critical failures)

	_emergency_required = !battery_ok || !sensors_ok || !estimator_ok;bool MinimalSafetyChecks::checkEmergencyStop()

{

	if (_emergency_required) {    // In minimal commander, we don't have a separate emergency stop input

		PX4_ERR("Runtime safety CRITICAL: Emergency action required!");    // This would normally check a hardware safety switch or kill switch

		if (!battery_ok) {    // For now, always return true (clear)

			PX4_ERR("  - Battery critical");

		}    // Future enhancement: Check safety switch topic

		if (!sensors_ok) {    // safety_s safety;

			PX4_ERR("  - Sensor failure");    // if (_safety_sub.update(&safety)) {

		}    //     return !safety.safety_off;

		if (!estimator_ok) {    // }

			PX4_ERR("  - Estimator failure");

		}    return true;

	}}



	// CPU and geofence are warnings but not immediate emergenciesconst char* MinimalSafetyChecks::getStatusString() const

	if (!cpu_ok) {{

		PX4_WARN("Runtime safety: CPU overload");    if (_can_arm) {

	}        return "All safety checks passed";

    } else if (!_battery_ok) {

	if (!geofence_ok) {        return "Battery check failed";

		PX4_WARN("Runtime safety: Geofence violation");    } else if (!_power_ok) {

	}        return "Power system check failed";

    } else if (!_emergency_stop_clear) {

	return !_emergency_required;        return "Emergency stop engaged";

}    } else {

        return "Unknown safety issue";

// ============================================================================    }

// ESSENTIAL PRE-ARM CHECKS}

// ============================================================================

bool MinimalSafetyChecks::checkBattery()
{
	battery_status_s battery_status;

	if (_battery_status_sub.update(&battery_status)) {
		// Check minimum charge level (default 20%)
		float min_charge = _param_bat_low_thr.get();

		if (battery_status.remaining < min_charge) {
			PX4_DEBUG("Battery: %.1f%% < %.1f%% (threshold)",
			          (double)(battery_status.remaining * 100.0f),
			          (double)(min_charge * 100.0f));
			return false;
		}

		// Check minimum voltage (10.0V for most systems)
		if (battery_status.voltage_v < 10.0f) {
			PX4_DEBUG("Battery: %.2fV < 10.0V (minimum)",
			          (double)battery_status.voltage_v);
			return false;
		}

		// Check for critical battery warning
		if (battery_status.warning >= battery_status_s::WARNING_CRITICAL) {
			PX4_DEBUG("Battery: Critical warning");
			return false;
		}

		return true;
	}

	// If no battery data, assume OK (for SITL/HIL)
	PX4_DEBUG("No battery status - assuming OK (SITL mode)");
	return true;
}

bool MinimalSafetyChecks::checkPower()
{
	system_power_s power;

	if (_system_power_sub.update(&power)) {
		// Check 5V rail (must be > 4.5V)
		if (power.voltage5v_v > 0.0f && power.voltage5v_v < 4.5f) {
			PX4_DEBUG("Power: 5V rail = %.2fV < 4.5V",
			          (double)power.voltage5v_v);
			return false;
		}

		return true;
	}

	// If no power monitoring, assume OK
	PX4_DEBUG("No power status - assuming OK");
	return true;
}

bool MinimalSafetyChecks::checkEmergencyStop()
{
	// Future: Check hardware safety switch
	// For now, always return true (clear)
	return true;
}

// ============================================================================
// ENHANCED PRE-ARM CHECKS
// ============================================================================

bool MinimalSafetyChecks::checkCPULoad()
{
	cpuload_s cpuload;

	if (_cpuload_sub.update(&cpuload)) {
		float max_load = _param_com_cpu_max.get();

		// Check if CPU load is acceptable (default < 90%)
		if (cpuload.load > max_load) {
			PX4_DEBUG("CPU: %.1f%% > %.1f%% (max)",
			          (double)(cpuload.load * 100.0f),
			          (double)(max_load * 100.0f));
			return false;
		}

		// Check RAM usage
		if (cpuload.ram_usage > 0.95f) {
			PX4_DEBUG("RAM: %.1f%% > 95%% (max)",
			          (double)(cpuload.ram_usage * 100.0f));
			return false;
		}

		return true;
	}

	// If no CPU load data, assume OK
	return true;
}

bool MinimalSafetyChecks::checkSensorHealth()
{
	sensors_status_imu_s sensors_status;

	if (_sensors_status_imu_sub.update(&sensors_status)) {
		// Check if primary IMU is healthy
		if (sensors_status.accel_inconsistency_m_s_s > 5.0f) {
			PX4_DEBUG("Sensors: Accel inconsistency %.2f > 5.0 m/s²",
			          (double)sensors_status.accel_inconsistency_m_s_s);
			return false;
		}

		if (sensors_status.gyro_inconsistency_rad_s > 0.5f) {
			PX4_DEBUG("Sensors: Gyro inconsistency %.2f > 0.5 rad/s",
			          (double)sensors_status.gyro_inconsistency_rad_s);
			return false;
		}

		return true;
	}

	// Check individual IMU status
	vehicle_imu_status_s imu_status;

	if (_vehicle_imu_status_sub.update(&imu_status)) {
		// Check for sensor errors
		if (imu_status.accel_error_count > 100 || imu_status.gyro_error_count > 100) {
			PX4_DEBUG("Sensors: High error count (accel=%u, gyro=%u)",
			          imu_status.accel_error_count, imu_status.gyro_error_count);
			return false;
		}

		return true;
	}

	// If no sensor data, assume OK (for SITL)
	return true;
}

bool MinimalSafetyChecks::checkEstimatorHealth()
{
	estimator_status_s estimator_status;

	if (_estimator_status_sub.update(&estimator_status)) {
		// Check innovation test ratios (should be < 1.0 for healthy estimation)
		bool healthy = true;

		// Check velocity innovations
		for (int i = 0; i < 3; i++) {
			if (estimator_status.vel_test_ratio[i] > 1.0f) {
				PX4_DEBUG("Estimator: Velocity test ratio[%d] = %.2f > 1.0",
				          i, (double)estimator_status.vel_test_ratio[i]);
				healthy = false;
			}
		}

		// Check position innovations
		for (int i = 0; i < 2; i++) {
			if (estimator_status.pos_test_ratio[i] > 1.0f) {
				PX4_DEBUG("Estimator: Position test ratio[%d] = %.2f > 1.0",
				          i, (double)estimator_status.pos_test_ratio[i]);
				healthy = false;
			}
		}

		// Check height innovation
		if (estimator_status.hgt_test_ratio > 1.0f) {
			PX4_DEBUG("Estimator: Height test ratio = %.2f > 1.0",
			          (double)estimator_status.hgt_test_ratio);
			healthy = false;
		}

		return healthy;
	}

	// If no estimator data, assume OK (for SITL or simple setups)
	return true;
}

bool MinimalSafetyChecks::checkManualControl()
{
	manual_control_setpoint_s manual_control;

	if (_manual_control_sub.update(&manual_control)) {
		_last_manual_control_time = manual_control.timestamp;

		// Check if manual control is recent (< 1 second old)
		if (hrt_elapsed_time(&_last_manual_control_time) < 1_s) {
			return true;
		}
	}

	// Check if we have any recent manual control
	if (_last_manual_control_time > 0) {
		if (hrt_elapsed_time(&_last_manual_control_time) < 5_s) {
			return true;
		}
	}

	// No manual control available - not blocking (allows offboard-only)
	PX4_DEBUG("Manual control: Not available (offboard mode OK)");
	return false; // Returns false but doesn't block arming
}

// ============================================================================
// RUNTIME CHECKS (During Flight)
// ============================================================================

bool MinimalSafetyChecks::checkBatteryRuntime()
{
	battery_status_s battery_status;

	if (_battery_status_sub.update(&battery_status)) {
		// Check critical battery level
		float crit_charge = _param_bat_crit_thr.get();

		if (battery_status.remaining < crit_charge) {
			return false;
		}

		// Check for emergency battery warning
		if (battery_status.warning >= battery_status_s::WARNING_EMERGENCY) {
			return false;
		}

		return true;
	}

	return true; // No data = assume OK
}

bool MinimalSafetyChecks::checkCPURuntime()
{
	cpuload_s cpuload;

	if (_cpuload_sub.update(&cpuload)) {
		// Runtime CPU check is more lenient (95% vs 90% for pre-arm)
		if (cpuload.load > 0.95f) {
			return false;
		}
	}

	return true;
}

bool MinimalSafetyChecks::checkSensorRuntime()
{
	vehicle_imu_status_s imu_status;

	if (_vehicle_imu_status_sub.update(&imu_status)) {
		// Check for sudden increase in sensor errors
		static uint32_t prev_accel_errors = 0;
		static uint32_t prev_gyro_errors = 0;

		uint32_t accel_errors_delta = imu_status.accel_error_count - prev_accel_errors;
		uint32_t gyro_errors_delta = imu_status.gyro_error_count - prev_gyro_errors;

		prev_accel_errors = imu_status.accel_error_count;
		prev_gyro_errors = imu_status.gyro_error_count;

		// Allow up to 10 errors per check cycle
		if (accel_errors_delta > 10 || gyro_errors_delta > 10) {
			return false;
		}
	}

	return true;
}

bool MinimalSafetyChecks::checkEstimatorRuntime()
{
	estimator_status_s estimator_status;

	if (_estimator_status_sub.update(&estimator_status)) {
		// Runtime estimator check is more lenient (2.0 vs 1.0 for pre-arm)
		bool healthy = true;

		for (int i = 0; i < 3; i++) {
			if (estimator_status.vel_test_ratio[i] > 2.0f) {
				healthy = false;
			}
		}

		return healthy;
	}

	return true;
}

bool MinimalSafetyChecks::checkGeofence()
{
	// Future: Implement geofence checking
	// For now, always return true
	return true;
}

// ============================================================================
// STATUS REPORTING
// ============================================================================

const char *MinimalSafetyChecks::getStatusString() const
{
	if (_can_arm) {
		return "All safety checks passed";

	} else if (!_battery_ok) {
		return "Battery check failed";

	} else if (!_power_ok) {
		return "Power system check failed";

	} else if (!_emergency_stop_clear) {
		return "Emergency stop engaged";

	} else if (!_cpu_ok) {
		return "CPU overload";

	} else if (!_sensors_ok) {
		return "Sensor health check failed";

	} else if (!_estimator_ok) {
		return "Estimator unhealthy";

	} else {
		return "Safety check failed (unknown)";
	}
}
