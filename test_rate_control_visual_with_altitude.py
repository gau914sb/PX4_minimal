#!/usr/bin/env python3
"""
Test script for PX4 rate control with live visualization (with altitude and thrust)
Shows drone attitude, altitude, and thrust in real-time using matplotlib
"""

import time
import math
from pymavlink import mavutil
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Constants
RATE_TARGET_TYPEMASK = 0b10000000

# Connect to PX4
print("Connecting to PX4...")
connection_strings = [
    'udp:127.0.0.1:14540',
    'udpin:localhost:14540',
    'udp:localhost:14540',
]

master = None
for conn_str in connection_strings:
    try:
        print(f"Trying {conn_str}...")
        master = mavutil.mavlink_connection(conn_str)
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=5)
        print(f"✓ Connected! System ID: {master.target_system}")
        break
    except Exception as e:
        print(f"  Failed: {e}")
        continue

if master is None:
    print("\n❌ Could not connect to PX4!")
    exit(1)

# Setup matplotlib
plt.ion()
fig = plt.figure(figsize=(18, 12))

# Create subplots (4 rows x 2 columns)
ax1 = plt.subplot(4, 2, 1)  # Roll over time
ax2 = plt.subplot(4, 2, 2)  # Pitch over time
ax3 = plt.subplot(4, 2, 3)  # Yaw over time
ax4 = plt.subplot(4, 2, 4)  # Roll rate over time
ax5 = plt.subplot(4, 2, 5)  # Pitch rate over time
ax6 = plt.subplot(4, 2, 6)  # Yaw rate over time
ax7 = plt.subplot(4, 2, 7)  # Altitude over time
ax8 = plt.subplot(4, 2, 8)  # Thrust over time

# Data storage
max_points = 200
times = []
rolls = []
pitches = []
yaws = []
roll_rates = []
pitch_rates = []
yaw_rates = []
setpoint_roll_rates = []
setpoint_pitch_rates = []
setpoint_yaw_rates = []
altitudes = []
thrusts = []

def is_armed():
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg:
        return msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0
    return False

def arm():
    if is_armed():
        print("\n[1/3] Vehicle is already ARMED ✓")
        return

    print("\n[1/3] Arming vehicle...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)
    print("✓ Armed")

def set_mode_offboard():
    print("\n[2/3] Setting OFFBOARD mode...")
    start_time = time.time()
    for i in range(50):
        time_boot_ms = int((time.time() - start_time) * 1000) % 4294967295
        master.mav.set_attitude_target_send(
            time_boot_ms, master.target_system, master.target_component,
            RATE_TARGET_TYPEMASK, [1, 0, 0, 0], 0, 0, 0, 0.5
        )
        time.sleep(0.02)

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6,
        0, 0, 0, 0, 0
    )
    time.sleep(0.1)
    print("✓ OFFBOARD mode set")

def update_plots():
    """Update all plots with latest data"""
    if len(times) == 0:
        return

    # Clear and redraw
    for ax in [ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8]:
        ax.clear()

    # Attitude plots
    ax1.plot(times, rolls, 'b-', linewidth=2, label='Roll')
    ax1.set_ylabel('Roll (deg)')
    ax1.set_title('Roll Attitude')
    ax1.grid(True)
    ax1.legend()

    ax2.plot(times, pitches, 'g-', linewidth=2, label='Pitch')
    ax2.set_ylabel('Pitch (deg)')
    ax2.set_title('Pitch Attitude')
    ax2.grid(True)
    ax2.legend()

    ax3.plot(times, yaws, 'r-', linewidth=2, label='Yaw')
    ax3.set_ylabel('Yaw (deg)')
    ax3.set_title('Yaw Attitude')
    ax3.grid(True)
    ax3.legend()

    # Rate plots
    ax4.plot(times, roll_rates, 'b-', linewidth=2, label='Actual')
    if setpoint_roll_rates:
        ax4.plot(times, setpoint_roll_rates, 'b--', linewidth=1, label='Setpoint')
    ax4.set_ylabel('Roll Rate (deg/s)')
    ax4.set_title('Roll Rate')
    ax4.grid(True)
    ax4.legend()

    ax5.plot(times, pitch_rates, 'g-', linewidth=2, label='Actual')
    if setpoint_pitch_rates:
        ax5.plot(times, setpoint_pitch_rates, 'g--', linewidth=1, label='Setpoint')
    ax5.set_ylabel('Pitch Rate (deg/s)')
    ax5.set_title('Pitch Rate')
    ax5.grid(True)
    ax5.legend()

    ax6.plot(times, yaw_rates, 'r-', linewidth=2, label='Actual')
    if setpoint_yaw_rates:
        ax6.plot(times, setpoint_yaw_rates, 'r--', linewidth=1, label='Setpoint')
    ax6.set_ylabel('Yaw Rate (deg/s)')
    ax6.set_title('Yaw Rate')
    ax6.grid(True)
    ax6.legend()

    # Altitude plot
    ax7.plot(times, altitudes, 'm-', linewidth=2, label='Altitude')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Altitude (m)')
    ax7.set_title('Altitude Above Ground')
    ax7.grid(True)
    ax7.legend()

    # Thrust plot
    ax8.plot(times, thrusts, 'c-', linewidth=2, label='Thrust')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Thrust (normalized)')
    ax8.set_title('Thrust Command')
    ax8.grid(True)
    ax8.legend()
    ax8.set_ylim([0, 1])

    plt.tight_layout()
    plt.draw()
    plt.pause(0.001)

def test_with_visualization():
    print("\n[3/3] Testing rate control with live visualization...")
    print("Close the plot window to stop the test")

    start_time = time.time()
    message_count = 0
    last_update = 0

    # Target altitude for the test
    TARGET_ALTITUDE = 3.0  # meters
    HOVER_THRUST = 0.5  # Base hover thrust

    # PD altitude controller gains (much gentler)
    KP_ALT = 0.08  # Proportional gain for altitude control (reduced)
    KD_ALT = 0.10  # Derivative gain for damping (velocity feedback)

    # Phase tracking
    phase = "TAKEOFF"  # TAKEOFF or YAW_SPIN
    takeoff_complete_time = None

    # Previous altitude for derivative calculation
    prev_alt = 0.0
    prev_time = start_time

    print("\n=== Phase 1: Takeoff to 3 meters ===")
    print("Watch the altitude graph!")

    try:
        while True:
            elapsed = time.time() - start_time
            time_boot_ms = int(elapsed * 1000) % 4294967295

            # Get current altitude and velocity
            alt_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.01)
            current_alt = 0.0
            current_vz = 0.0  # Vertical velocity (positive = up)

            if alt_msg:
                current_alt = -alt_msg.z  # NED frame, negate Z for altitude
                current_vz = -alt_msg.vz  # NED frame, negate Z velocity
            elif altitudes:
                current_alt = altitudes[-1]

            # PD altitude controller
            altitude_error = TARGET_ALTITUDE - current_alt

            # Proportional term
            thrust_p = KP_ALT * altitude_error

            # Derivative term (damping based on vertical velocity)
            # Negative velocity feedback: if moving up too fast, reduce thrust
            thrust_d = -KD_ALT * current_vz

            # Combined thrust
            thrust = HOVER_THRUST + thrust_p + thrust_d
            thrust = max(0.35, min(0.65, thrust))  # Tighter limits for stability

            # Phase management
            if phase == "TAKEOFF":
                # Check if we've reached target altitude (within 0.3m tolerance)
                if abs(altitude_error) < 0.3 and current_alt > 2.5 and abs(current_vz) < 0.3:
                    if takeoff_complete_time is None:
                        takeoff_complete_time = elapsed
                        print(f"\n✓ Altitude stabilized at {current_alt:.2f}m, holding for 3 seconds...")
                    # Hold altitude for 3 seconds before starting yaw
                    elif elapsed - takeoff_complete_time > 3.0:
                        phase = "YAW_SPIN"
                        print(f"\n=== Phase 2: Yaw spin at 10 deg/s (maintaining {TARGET_ALTITUDE}m altitude) ===")
                        print("Watch the yaw rate and altitude graphs!")
                else:
                    # Reset timer if we drift away
                    takeoff_complete_time = None

                # Takeoff mode: no yaw rate
                yaw_rate_deg = 0.0
                yaw_rate_rad = 0.0

            else:  # YAW_SPIN phase
                # Yaw rate setpoint (10 deg/s)
                yaw_rate_deg = 10.0
                yaw_rate_rad = math.radians(yaw_rate_deg)

            # Send rate command with altitude-adjusted thrust
            master.mav.set_attitude_target_send(
                time_boot_ms, master.target_system, master.target_component,
                RATE_TARGET_TYPEMASK, [1, 0, 0, 0],
                0, 0, yaw_rate_rad, thrust
            )

            # Get feedback
            msg = master.recv_match(type='ATTITUDE', blocking=False, timeout=0.01)
            if msg:
                # Append attitude data
                times.append(elapsed)
                rolls.append(math.degrees(msg.roll))
                pitches.append(math.degrees(msg.pitch))
                yaws.append(math.degrees(msg.yaw))
                roll_rates.append(math.degrees(msg.rollspeed))
                pitch_rates.append(math.degrees(msg.pitchspeed))
                yaw_rates.append(math.degrees(msg.yawspeed))
                setpoint_roll_rates.append(0)
                setpoint_pitch_rates.append(0)
                setpoint_yaw_rates.append(yaw_rate_deg)
                thrusts.append(thrust)  # Current thrust command

            # Get altitude data (LOCAL_POSITION_NED has relative altitude)
            alt_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.01)
            if alt_msg:
                altitudes.append(-alt_msg.z)  # NED frame, so negate Z for altitude

            # If we have more time entries than altitude entries, pad altitude
            while len(altitudes) < len(times):
                altitudes.append(altitudes[-1] if altitudes else 0.0)

            if msg:
                # Keep only last N points
                if len(times) > max_points:
                    times.pop(0)
                    rolls.pop(0)
                    pitches.pop(0)
                    yaws.pop(0)
                    roll_rates.pop(0)
                    pitch_rates.pop(0)
                    yaw_rates.pop(0)
                    setpoint_roll_rates.pop(0)
                    setpoint_pitch_rates.pop(0)
                    setpoint_yaw_rates.pop(0)
                    altitudes.pop(0)
                    thrusts.pop(0)

                # Update plots every 0.1 seconds
                if elapsed - last_update > 0.1:
                    update_plots()
                    last_update = elapsed

                    # Print status
                    if message_count % 50 == 0:
                        current_alt_display = altitudes[-1] if altitudes else 0.0
                        print(f"[{elapsed:.1f}s] Phase: {phase:10s} | "
                              f"Alt: {current_alt_display:5.2f}m (target: {TARGET_ALTITUDE}m) | "
                              f"Vz: {current_vz:5.2f}m/s | "
                              f"Thrust: {thrust:.3f} | "
                              f"Yaw: {math.degrees(msg.yaw):6.1f}° | "
                              f"Yaw rate: {math.degrees(msg.yawspeed):5.1f} deg/s (sp: {yaw_rate_deg:.1f})")

            message_count += 1
            time.sleep(0.02)  # 50 Hz

            # Check if window is closed
            if not plt.fignum_exists(fig.number):
                print("\n✓ Plot window closed, stopping test")
                break

    except KeyboardInterrupt:
        print("\n✓ Test interrupted by user")

    # Return to hover at current altitude
    print("\nReturning to hover...")
    for i in range(100):
        elapsed = time.time() - start_time
        time_boot_ms = int(elapsed * 1000) % 4294967295

        # Get current altitude for final hover
        alt_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.01)
        if alt_msg:
            current_alt = -alt_msg.z
        elif altitudes:
            current_alt = altitudes[-1]
        else:
            current_alt = 0.0

        # Maintain current altitude with no yaw rate
        altitude_error = current_alt - current_alt  # Try to maintain current altitude
        thrust = HOVER_THRUST

        master.mav.set_attitude_target_send(
            time_boot_ms, master.target_system, master.target_component,
            RATE_TARGET_TYPEMASK, [1, 0, 0, 0], 0, 0, 0, thrust
        )
        time.sleep(0.02)

# Main execution
print("=" * 50)
print("PX4 Rate Control Test with Altitude & Thrust Visualization")
print("=" * 50)

arm()
set_mode_offboard()
test_with_visualization()

print("\n" + "=" * 50)
print("Test complete!")
print("=" * 50)
