#!/usr/bin/env python3
"""
PX4 Safe Ground Motor Test
Tests motor control while drone is on ground (armed but not flying)
"""

import time
from pymavlink import mavutil

def ground_motor_test():
    print("=" * 60)
    print("PX4 SAFE GROUND MOTOR TEST")
    print("=" * 60)
    print("🟡 This test arms the drone but keeps it on ground")
    print("🔧 You should hear motor speed changes")
    print("⚠️  DO NOT HOLD DRONE - let it stay on ground!")
    print("")

    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}")

    # Arm the vehicle
    print("🔓 Arming drone...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

    time.sleep(3)
    print("✅ Drone should be armed - motors will spin slowly")

    print("\n🔧 Testing motor speed variations (drone stays on ground)...")

    base_speed = 1100  # Low speed - drone won't takeoff

    for test_num in range(3):
        print(f"\n🧪 Test {test_num + 1}/3: Motor differential")

        # Test 1: All motors equal (should sound balanced)
        print("   Equal speeds - should sound balanced")
        for i in range(20):
            master.mav.servo_output_raw_send(
                0, 0,
                base_speed, base_speed, base_speed, base_speed,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0
            )
            time.sleep(0.1)

        # Test 2: One motor faster (should sound different)
        print("   Motor 1 faster - should sound different")
        for i in range(20):
            master.mav.servo_output_raw_send(
                0, 0,
                base_speed + 100, base_speed, base_speed, base_speed,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0
            )
            time.sleep(0.1)

        # Test 3: Diagonal motors faster
        print("   Diagonal pattern - should sound complex")
        for i in range(20):
            master.mav.servo_output_raw_send(
                0, 0,
                base_speed + 50, base_speed - 50, base_speed + 50, base_speed - 50,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0
            )
            time.sleep(0.1)

    print("\n🔓 Disarming drone...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    print("✅ Ground test complete!")
    print("📊 Results:")
    print("   - If you heard motor speed changes: ✅ Control works!")
    print("   - If all motors sounded the same: ❌ Control not working")

if __name__ == "__main__":
    try:
        ground_motor_test()
    except KeyboardInterrupt:
        print("\n⏹️  Test stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
