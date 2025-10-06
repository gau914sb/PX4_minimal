#!/usr/bin/env python3
"""
PX4 Console Commands for Manual Control
Uses PX4 shell commands for direct control
"""

import time
from pymavlink import mavutil

def console_takeoff():
    print("=" * 50)
    print("PX4 CONSOLE TAKEOFF")
    print("=" * 50)

    # Connect to PX4 console (different approach)
    print("🔗 Connecting to PX4...")
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}")

    print("\n🚁 Manual takeoff sequence...")

    # Force arm
    print("🔓 Force arming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm
        21196,  # force arm (magic number)
        0, 0, 0, 0, 0
    )

    time.sleep(2)

    # Try takeoff command
    print("🚀 Sending takeoff command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0,  # pitch
        0, 0, 0, 0, 0,
        3   # altitude (3 meters)
    )

    print("⏳ Waiting for takeoff...")
    time.sleep(10)

    print("✅ Takeoff command sent")
    print("🎮 Check if drone is flying!")

def console_control():
    """Alternative: Use raw PWM control"""
    print("\n🎮 RAW PWM CONTROL TEST")

    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()

    print("🔧 Sending raw PWM commands...")

    # Send raw servo commands
    for i in range(100):
        master.mav.servo_output_raw_send(
            0,  # time_usec
            0,  # port
            1500,  # servo1 (roll) - center
            1500,  # servo2 (pitch) - center
            1700,  # servo3 (throttle) - 70%
            1500,  # servo4 (yaw) - center
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )

        if i % 10 == 0:
            print(f"⚡ PWM commands sent... {i}/100")

        time.sleep(0.1)

    print("✅ PWM sequence complete")

if __name__ == "__main__":
    try:
        console_takeoff()
        time.sleep(5)
        console_control()
    except Exception as e:
        print(f"❌ Error: {e}")
