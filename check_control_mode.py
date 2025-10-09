#!/usr/bin/env python3
"""
Quick check: What flags are set in offboard_control_mode and vehicle_control_mode?
"""

from pymavlink import mavutil
import time

# Connect
print("Connecting...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14540')
mav.wait_heartbeat()
print("✓ Connected\n")

# Arm
print("Arming...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(0.5)

# Switch to OFFBOARD
print("Switching to OFFBOARD...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0, 1, 6, 0, 0, 0, 0, 0)  # base_mode=1 (custom), custom_mode=6 (OFFBOARD)
time.sleep(0.5)

# Send attitude setpoints for 5 seconds
print("\nSending attitude setpoints with thrust...")
print("Check PX4 console for 'OFFBOARD flags' debug message\n")

for i in range(250):  # 5 seconds at 50Hz
    # SET_ATTITUDE_TARGET with attitude + thrust
    mav.mav.set_attitude_target_send(
        0,  # time_boot_ms (0 = use receiver timestamp)
        mav.target_system,
        mav.target_component,
        0b00000111,  # type_mask: ignore rates (bits 0-2 set)
        [1, 0, 0, 0],  # q: quaternion for level attitude
        0, 0, 0,  # body roll/pitch/yaw rates (ignored)
        0.6  # thrust (0.6 = 60%)
    )
    time.sleep(0.02)  # 50Hz

print("✓ Done - Check PX4 console output above")
