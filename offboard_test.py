#!/usr/bin/env python3
"""
PX4 OFFBOARD Mode Test
Quick test to verify OFFBOARD mode switching works
"""

import time
from pymavlink import mavutil

def test_offboard_mode():
    print("=" * 50)
    print("PX4 OFFBOARD MODE TEST")
    print("=" * 50)

    # Connect
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}")

    # Get current mode
    print("Getting current flight mode...")
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        current_mode = msg.custom_mode
        print(f"Current mode: {current_mode}")

    # Try switching to OFFBOARD
    print("\n🔄 Attempting to switch to OFFBOARD mode...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6  # OFFBOARD mode
    )

    # Check if it worked
    print("Checking mode change...")
    success = False
    for i in range(10):  # 2.5 seconds
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.custom_mode == 6:
            print("✅ SUCCESS! OFFBOARD mode active")
            success = True
            break
        time.sleep(0.25)

    if not success:
        print("❌ FAILED to enter OFFBOARD mode")
        print("   flight_mode_manager might still be disabled")
        return False

    # Send a quick attitude command to test control
    print("\n🚁 Testing attitude control (3 seconds)...")
    for i in range(30):
        master.mav.set_attitude_target_send(
            0, master.target_system, master.target_component,
            0b00000111,  # ignore attitude, use rates
            [1, 0, 0, 0],  # quaternion (ignored)
            0.2,  # roll rate
            0,    # pitch rate
            0,    # yaw rate
            0.6   # thrust
        )
        time.sleep(0.1)

    print("✅ Test complete! If you saw rolling, external control works!")
    return True

if __name__ == "__main__":
    test_offboard_mode()
