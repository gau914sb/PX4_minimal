#!/usr/bin/env python3
"""
PX4 Visual Motor Test for jMAVSim
Creates dramatic motor differences that should be visible in jMAVSim 3D view
"""

import time
from pymavlink import mavutil

def visual_motor_test():
    print("=" * 60)
    print("PX4 VISUAL MOTOR TEST - Watch jMAVSim 3D View!")
    print("=" * 60)
    print("👁️  Watch the jMAVSim 3D quadcopter model")
    print("🔄 You should see DRAMATIC rotation/tilting")
    print("⚠️  Ground test - drone stays on ground but tilts visually")
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
    print("✅ Drone armed - watch jMAVSim 3D view!")

    base_speed = 1200  # Higher speed for more dramatic visual effect

    print("\n👁️  WATCH JMAVSIM 3D VIEW - SHOULD SEE DRAMATIC MOVEMENTS!")

    print("\n🟢 Phase 1: Balanced hover (5 seconds)")
    print("   → Quadcopter should look level")
    for i in range(50):
        master.mav.servo_output_raw_send(
            0, 0,
            base_speed, base_speed, base_speed, base_speed,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)

    print("\n↔️  Phase 2: MASSIVE ROLL LEFT (5 seconds)")
    print("   → Quadcopter should TILT HEAVILY to the left!")
    for i in range(50):
        master.mav.servo_output_raw_send(
            0, 0,
            base_speed - 300,  # Front Right - much slower
            base_speed + 300,  # Rear Left - much faster
            base_speed + 300,  # Front Left - much faster
            base_speed - 300,  # Rear Right - much slower
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)

    print("\n📍 Phase 3: Return to level (3 seconds)")
    for i in range(30):
        master.mav.servo_output_raw_send(
            0, 0,
            base_speed, base_speed, base_speed, base_speed,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)

    print("\n↔️  Phase 4: MASSIVE ROLL RIGHT (5 seconds)")
    print("   → Quadcopter should TILT HEAVILY to the right!")
    for i in range(50):
        master.mav.servo_output_raw_send(
            0, 0,
            base_speed + 300,  # Front Right - much faster
            base_speed - 300,  # Rear Left - much slower
            base_speed - 300,  # Front Left - much slower
            base_speed + 300,  # Rear Right - much faster
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)

    print("\n📍 Phase 5: Return to level (3 seconds)")
    for i in range(30):
        master.mav.servo_output_raw_send(
            0, 0,
            base_speed, base_speed, base_speed, base_speed,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)

    print("\n⬆️  Phase 6: MASSIVE PITCH FORWARD (5 seconds)")
    print("   → Quadcopter should PITCH FORWARD dramatically!")
    for i in range(50):
        master.mav.servo_output_raw_send(
            0, 0,
            base_speed + 300,  # Front Right - faster (front motors)
            base_speed - 300,  # Rear Left - slower (rear motors)
            base_speed + 300,  # Front Left - faster (front motors)
            base_speed - 300,  # Rear Right - slower (rear motors)
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)

    print("\n🔄 Phase 7: MASSIVE YAW SPIN (5 seconds)")
    print("   → Quadcopter should SPIN around Z-axis!")
    for i in range(50):
        master.mav.servo_output_raw_send(
            0, 0,
            base_speed + 400,  # Front Right - much faster
            base_speed - 400,  # Rear Left - much slower
            base_speed + 400,  # Front Left - much faster
            base_speed - 400,  # Rear Right - much slower
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)

    print("\n📍 Final: Return to level and disarm")
    for i in range(30):
        master.mav.servo_output_raw_send(
            0, 0,
            base_speed, base_speed, base_speed, base_speed,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.1)

    # Disarm
    print("🔓 Disarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    print("✅ VISUAL TEST COMPLETE!")
    print("📊 Results:")
    print("   ✅ If you saw dramatic tilting/spinning: CONTROL WORKS!")
    print("   ❌ If quadcopter stayed perfectly level: Control not working")

if __name__ == "__main__":
    try:
        visual_motor_test()
    except KeyboardInterrupt:
        print("\n⏹️  Test stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
