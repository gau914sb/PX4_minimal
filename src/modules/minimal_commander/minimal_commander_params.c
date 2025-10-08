/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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
 * @file minimal_commander_params.c
 *
 * Parameters required by minimal_commander and dependent modules.
 * Extracted from commander module to enable standalone operation.
 *
 * @author Gaurav Singh Bhati (Minimal Commander implementation)
 */

/**
 * Manual control loss timeout
 *
 * The time in seconds without a new setpoint from RC or Joystick, after which the connection is considered lost.
 * This must be kept short as the vehicle will use the last supplied setpoint until the timeout triggers.
 * Ensure the value is not set lower than the update interval of the RC or Joystick.
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 35
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_RC_LOSS_T, 0.5f);

/**
 * RC control input mode
 *
 * A value of 0 enables RC transmitter control (only). A valid RC transmitter calibration is required.
 * A value of 1 allows joystick control only. RC input handling and the associated checks are disabled.
 * A value of 2 allows either RC Transmitter or Joystick input. The first valid input is used, will fallback to other sources if the input stream becomes invalid.
 * A value of 3 allows either input from RC or joystick. The first available source is selected and used until reboot.
 * A value of 4 ignores any stick input.
 * A value of 5 allows either RC Transmitter or Joystick input. But RC has priority and whenever available is immediately used.
 * A value of 6 allows either RC Transmitter or Joystick input. But Joystick has priority and whenever available is immediately used.
 *
 * @group Commander
 * @min 0
 * @max 6
 * @value 0 RC Transmitter only
 * @value 1 Joystick only
 * @value 2 RC and Joystick with fallback
 * @value 3 RC or Joystick keep first
 * @value 4 Stick input disabled
 * @value 5 RC priority, Joystick fallback
 * @value 6 Joystick priority, RC fallback
 */
PARAM_DEFINE_INT32(COM_RC_IN_MODE, 3);

/**
 * RC input arm/disarm command duration
 *
 * The default value of 1000 requires the stick to be held in the arm or disarm position for 1 second.
 *
 * @group Commander
 * @min 100
 * @max 1500
 * @unit ms
 */
PARAM_DEFINE_INT32(COM_RC_ARM_HYST, 1000);

/**
 * Arm switch is a momentary button
 *
 * 0: Arming/disarming triggers on switch transition.
 * 1: Arming/disarming triggers when holding the momentary button down
 * for COM_RC_ARM_HYST like the stick gesture.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_SWISBTN, 0);

/**
 * RC stick override threshold
 *
 * If COM_RC_OVERRIDE is enabled and the joystick input is moved more than this threshold
 * the autopilot the pilot takes over control.
 *
 * @group Commander
 * @unit %
 * @min 5
 * @max 80
 * @decimal 0
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_RC_STICK_OV, 30.0f);

/**
 * Low battery action
 *
 * The action the system takes at critical battery level.
 *
 * @group Commander
 * @value 0 None
 * @value 1 Warning
 * @value 2 Return
 * @value 3 Land
 */
PARAM_DEFINE_INT32(COM_LOW_BAT_ACT, 0);

/**
 * Flight mode 1
 *
 * The main features of flight modes are the automatic heading control and throttle handling.
 *
 * @group Commander
 * @min -1
 * @max 10
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Orbit
 * @value 10 Takeoff
 */
PARAM_DEFINE_INT32(COM_FLTMODE1, -1);

/**
 * Flight mode 2
 *
 * @group Commander
 * @min -1
 * @max 10
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Orbit
 * @value 10 Takeoff
 */
PARAM_DEFINE_INT32(COM_FLTMODE2, -1);

/**
 * Flight mode 3
 *
 * @group Commander
 * @min -1
 * @max 10
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Orbit
 * @value 10 Takeoff
 */
PARAM_DEFINE_INT32(COM_FLTMODE3, -1);

/**
 * Flight mode 4
 *
 * @group Commander
 * @min -1
 * @max 10
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Orbit
 * @value 10 Takeoff
 */
PARAM_DEFINE_INT32(COM_FLTMODE4, -1);

/**
 * Flight mode 5
 *
 * @group Commander
 * @min -1
 * @max 10
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Orbit
 * @value 10 Takeoff
 */
PARAM_DEFINE_INT32(COM_FLTMODE5, -1);

/**
 * Flight mode 6
 *
 * @group Commander
 * @min -1
 * @max 10
 * @value -1 Unassigned
 * @value 0 Manual
 * @value 1 Altitude
 * @value 2 Position
 * @value 3 Mission
 * @value 4 Hold
 * @value 5 Return
 * @value 6 Acro
 * @value 7 Offboard
 * @value 8 Stabilized
 * @value 9 Orbit
 * @value 10 Takeoff
 */
PARAM_DEFINE_INT32(COM_FLTMODE6, -1);
