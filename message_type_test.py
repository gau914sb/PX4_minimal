#!/usr/bin/env python3
"""
PX4 Message Type Diagnostic
Tests different MAVLink message types to find what actually works
"""

import time
from pymavlink import mavutil

def test_all_control_methods():
    print("=" * 60)
    print("PX4 COMPREHENSIVE CONTROL TEST")
    print("=" * 60)

    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}")

    # Enable data streams to see feedback
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,  # Hz
        1    # Enable
    )

    # Arm the vehicle first
    print("🔓 Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3)

    print("\n🧪 Testing Method 1: SET_ACTUATOR_CONTROL_TARGET")
    for i in range(20):
        master.mav.set_actuator_control_target_send(
            0,  # time_boot_ms
            0,  # group_mlx
            master.target_system,
            master.target_component,
            [0.8, 0.2, 0.8, 0.2, 0, 0, 0, 0]  # Dramatic difference
        )
        if i % 5 == 0:
            print(f"   Sent actuator control {i+1}/20")
        time.sleep(0.1)

    print("\n🧪 Testing Method 2: MANUAL_CONTROL")
    for i in range(20):
        master.mav.manual_control_send(
            master.target_system,
            500,   # x (pitch) - center = 0, range ±1000
            -500,  # y (roll) - strong left roll
            600,   # z (throttle) - 0-1000
            0,     # r (yaw) - center = 0, range ±1000
            0      # buttons
        )
        if i % 5 == 0:
            print(f"   Sent manual control {i+1}/20")
        time.sleep(0.1)

    print("\n🧪 Testing Method 3: SET_POSITION_TARGET (with rates)")
    for i in range(20):
        master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b111111000111,  # type_mask (ignore pos/vel, use rates)
            0, 0, 0,    # x, y, z position (ignored)
            0, 0, 0,    # vx, vy, vz velocity (ignored)
            0, 0, 0,    # afx, afy, afz acceleration (ignored)
            0,          # yaw (ignored)
            0.5         # yaw_rate (rad/s)
        )
        if i % 5 == 0:
            print(f"   Sent position target {i+1}/20")
        time.sleep(0.1)

    print("\n🧪 Testing Method 4: HIL_ACTUATOR_CONTROLS")
    for i in range(20):
        master.mav.hil_actuator_controls_send(
            0,  # time_usec
            [0.8, 0.2, 0.8, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # controls
            mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,  # mode
            0   # flags
        )
        if i % 5 == 0:
            print(f"   Sent HIL actuator {i+1}/20")
        time.sleep(0.1)

    print("\n🧪 Testing Method 5: Force OFFBOARD mode first")
    # Force switch to offboard mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6  # OFFBOARD mode
    )
    time.sleep(1)

    # Then try attitude setpoint
    for i in range(20):
        master.mav.set_attitude_target_send(
            0,  # time_boot_ms
            master.target_system,
            master.target_component,
            0b00000111,  # type_mask (ignore attitude, use rates)
            [1, 0, 0, 0],  # quaternion (ignored)
            1.0,  # body_roll_rate (rad/s) - strong rate
            0,    # body_pitch_rate
            0,    # body_yaw_rate
            0.6   # thrust
        )
        if i % 5 == 0:
            print(f"   Sent attitude target {i+1}/20")
        time.sleep(0.1)

    # Disarm
    print("\n🔓 Disarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    print("\n📊 TEST COMPLETE!")
    print("Which method showed movement in jMAVSim?")
    print("1. SET_ACTUATOR_CONTROL_TARGET")
    print("2. MANUAL_CONTROL")
    print("3. SET_POSITION_TARGET")
    print("4. HIL_ACTUATOR_CONTROLS")
    print("5. SET_ATTITUDE_TARGET (OFFBOARD)")
    print("\nIf NONE showed movement, the issue is deeper...")

if __name__ == "__main__":
    try:
        test_all_control_methods()
    except KeyboardInterrupt:
        print("\n⏹️  Test stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
