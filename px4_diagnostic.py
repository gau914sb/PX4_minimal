#!/usr/bin/env python3
"""
PX4 Diagnostic Script
Checks MAVLink connection and current flight mode
"""

import time
from pymavlink import mavutil

def main():
    print("=" * 60)
    print("PX4 DIAGNOSTIC - Checking Connection & Flight Mode")
    print("=" * 60)

    # Connect
    print("Connecting to PX4...")
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")

    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}, component {master.target_component}")

    # Request data streams
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,  # Hz
        1   # Enable
    )

    print("\n🔍 Monitoring for 10 seconds...")
    print("Flight Mode | Armed | Alt | Roll | Pitch | Yaw")
    print("-" * 50)

    start_time = time.time()
    while time.time() - start_time < 10:
        msg = master.recv_match(blocking=False)
        if msg:
            if msg.get_type() == 'HEARTBEAT':
                # Decode flight mode
                custom_mode = msg.custom_mode
                base_mode = msg.base_mode
                armed = "ARMED" if (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else "DISARMED"

                # Common PX4 modes
                mode_map = {
                    0: "MANUAL",
                    1: "ALTCTL",
                    2: "POSCTL",
                    3: "AUTO_MISSION",
                    4: "AUTO_LOITER",
                    5: "AUTO_RTL",
                    6: "OFFBOARD",
                    7: "STABILIZED",
                    8: "RATTITUDE",
                    9: "ACRO"
                }

                mode_name = mode_map.get(custom_mode, f"UNKNOWN({custom_mode})")
                print(f"{mode_name:11s} | {armed:7s} |", end="")

            elif msg.get_type() == 'ATTITUDE':
                roll_deg = math.degrees(msg.roll)
                pitch_deg = math.degrees(msg.pitch)
                yaw_deg = math.degrees(msg.yaw)
                print(f" {roll_deg:4.0f}° | {pitch_deg:4.0f}° | {yaw_deg:4.0f}°")

            elif msg.get_type() == 'GLOBAL_POSITION_INT':
                alt_m = msg.relative_alt / 1000.0  # mm to m
                print(f" {alt_m:4.1f}m |", end="")

        time.sleep(0.25)

    print(f"\n✅ Diagnostic complete")

    # Test switching to OFFBOARD
    print(f"\n🔄 Testing OFFBOARD mode switch...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6  # OFFBOARD
    )

    print("Checking if mode changed...")
    for i in range(20):  # 5 seconds
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and msg.custom_mode == 6:
            print("✅ Successfully switched to OFFBOARD mode!")
            break
        time.sleep(0.25)
    else:
        print("❌ Failed to switch to OFFBOARD mode")
        print("   This might be why external commands don't work!")

if __name__ == "__main__":
    import math
    main()
