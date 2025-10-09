#!/usr/bin/env python3
"""
Test script for PX4 rate control with live visualization
Shows drone attitude in real-time using matplotlib
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
fig = plt.figure(figsize=(15, 10))

# Create subplots
ax1 = plt.subplot(2, 3, 1)  # Roll over time
ax2 = plt.subplot(2, 3, 2)  # Pitch over time
ax3 = plt.subplot(2, 3, 3)  # Yaw over time
ax4 = plt.subplot(2, 3, 4)  # Roll rate over time
ax5 = plt.subplot(2, 3, 5)  # Pitch rate over time
ax6 = plt.subplot(2, 3, 6)  # Yaw rate over time

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
    for ax in [ax1, ax2, ax3, ax4, ax5, ax6]:
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
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Roll Rate (deg/s)')
    ax4.set_title('Roll Rate')
    ax4.grid(True)
    ax4.legend()
    
    ax5.plot(times, pitch_rates, 'g-', linewidth=2, label='Actual')
    if setpoint_pitch_rates:
        ax5.plot(times, setpoint_pitch_rates, 'g--', linewidth=1, label='Setpoint')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Pitch Rate (deg/s)')
    ax5.set_title('Pitch Rate')
    ax5.grid(True)
    ax5.legend()
    
    ax6.plot(times, yaw_rates, 'r-', linewidth=2, label='Actual')
    if setpoint_yaw_rates:
        ax6.plot(times, setpoint_yaw_rates, 'r--', linewidth=1, label='Setpoint')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Yaw Rate (deg/s)')
    ax6.set_title('Yaw Rate')
    ax6.grid(True)
    ax6.legend()
    
    plt.tight_layout()
    plt.draw()
    plt.pause(0.001)

def test_with_visualization():
    print("\n[3/3] Testing rate control with live visualization...")
    print("Close the plot window to stop the test")
    
    start_time = time.time()
    message_count = 0
    last_update = 0
    
    # Test: Constant yaw rate (30 deg/s)
    print("\n=== Spinning with 30 deg/s yaw rate ===")
    print("Watch the plots update in real-time!")
    
    try:
        while True:
            elapsed = time.time() - start_time
            time_boot_ms = int(elapsed * 1000) % 4294967295
            
            # Yaw rate setpoint (30 deg/s)
            yaw_rate_deg = 30.0
            yaw_rate_rad = math.radians(yaw_rate_deg)
            
            # Send rate command
            master.mav.set_attitude_target_send(
                time_boot_ms, master.target_system, master.target_component,
                RATE_TARGET_TYPEMASK, [1, 0, 0, 0],
                0, 0, yaw_rate_rad, 0.5
            )
            
            # Get feedback
            msg = master.recv_match(type='ATTITUDE', blocking=False, timeout=0.01)
            if msg:
                # Append data
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
                
                # Update plots every 0.1 seconds
                if elapsed - last_update > 0.1:
                    update_plots()
                    last_update = elapsed
                    
                    # Print status
                    if message_count % 50 == 0:
                        print(f"[{elapsed:.1f}s] Yaw: {math.degrees(msg.yaw):6.1f}° | "
                              f"Yaw rate: {math.degrees(msg.yawspeed):5.1f} deg/s | "
                              f"Setpoint: {yaw_rate_deg} deg/s")
            
            message_count += 1
            time.sleep(0.02)  # 50 Hz
            
            # Check if window is closed
            if not plt.fignum_exists(fig.number):
                print("\n✓ Plot window closed, stopping test")
                break
                
    except KeyboardInterrupt:
        print("\n✓ Test interrupted by user")
    
    # Return to hover
    print("\nReturning to hover...")
    for i in range(50):
        elapsed = time.time() - start_time
        time_boot_ms = int(elapsed * 1000) % 4294967295
        master.mav.set_attitude_target_send(
            time_boot_ms, master.target_system, master.target_component,
            RATE_TARGET_TYPEMASK, [1, 0, 0, 0], 0, 0, 0, 0.5
        )
        time.sleep(0.02)

# Main execution
print("=" * 50)
print("PX4 Rate Control Test with Visualization")
print("=" * 50)

arm()
set_mode_offboard()
test_with_visualization()

print("\n" + "=" * 50)
print("Test complete!")
print("=" * 50)
