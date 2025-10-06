#!/usr/bin/env python3
"""
PX4 Manual Takeoff Script
Takes off using manual control (no position control needed)
"""

import time
from pymavlink import mavutil

def manual_takeoff():
    print("=" * 50)
    print("PX4 MANUAL TAKEOFF - No Position Control")
    print("=" * 50)

    # Connect
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}")

    # Switch to Manual mode first
    print("🔄 Switching to Manual mode...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0  # Manual mode
    )
    time.sleep(2)

    # Arm the vehicle
    print("🔓 Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm
        0, 0, 0, 0, 0, 0
    )

    # Wait for arming
    print("⏳ Waiting for arming...")
    for i in range(20):  # 5 seconds
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("✅ Vehicle armed!")
            break
        time.sleep(0.25)
    else:
        print("❌ Failed to arm vehicle")
        return

    # Manual takeoff using RC override
    print("🚁 Manual takeoff - applying thrust...")
    takeoff_thrust = 1800  # 80% throttle for takeoff

    for i in range(100):  # 10 seconds of takeoff thrust
        # Send RC override for takeoff
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500,  # Roll (center)
            1500,  # Pitch (center)
            takeoff_thrust,  # Throttle (high for takeoff)
            1500,  # Yaw (center)
            0, 0, 0, 0  # Other channels
        )

        if i == 50:  # After 5 seconds, reduce to hover thrust
            takeoff_thrust = 1600  # 60% throttle for hover
            print("🔄 Reducing to hover thrust...")

        time.sleep(0.1)

    # Stop RC override - return to normal control
    print("✅ Takeoff complete - returning manual control")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
    )

    print("🎮 Manual control returned - drone should be hovering")
    print("🔧 You can now test OFFBOARD mode with attitude commands!")

if __name__ == "__main__":
    manual_takeoff()
