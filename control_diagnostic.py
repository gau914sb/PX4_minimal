#!/usr/bin/env python3
"""
PX4 Control Diagnostic
Checks what control modes are active and why commands might not work
"""

import time
from pymavlink import mavutil

def diagnose_control():
    print("=" * 50)
    print("PX4 CONTROL DIAGNOSTIC")
    print("=" * 50)

    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}")

    # Request all data streams
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,  # Hz
        1   # Enable
    )

    print("\n🔍 Monitoring control status for 10 seconds...")
    print("Mode | Armed | RC Override | Manual Input | Attitude")
    print("-" * 55)

    start_time = time.time()
    while time.time() - start_time < 10:
        msg = master.recv_match(blocking=False)
        if msg:
            if msg.get_type() == 'HEARTBEAT':
                mode = msg.custom_mode
                armed = "ARM" if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else "DISARM"
                manual_input = "MAN" if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) else "AUTO"
                print(f"{mode:4} | {armed:7} | {manual_input:11} |", end="")

            elif msg.get_type() == 'RC_CHANNELS':
                # Check if RC override is active
                override_active = any(ch != 65535 for ch in [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw])
                print(f" {'OVERRIDE' if override_active else 'NORMAL':8} |", end="")

            elif msg.get_type() == 'ATTITUDE':
                roll_deg = math.degrees(msg.roll)
                pitch_deg = math.degrees(msg.pitch)
                yaw_deg = math.degrees(msg.yaw)
                print(f" R:{roll_deg:+4.0f}° P:{pitch_deg:+4.0f}° Y:{yaw_deg:+4.0f}°")

        time.sleep(0.25)

    print(f"\n🔧 Testing RC override directly...")

    # Send a strong RC override command
    for i in range(20):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1200,  # Roll left (strong command)
            1500,  # Pitch center
            1600,  # Throttle 60%
            1500,  # Yaw center
            0, 0, 0, 0
        )
        print(f"📡 RC Override {i+1}/20: Roll=1200 (strong left)")
        time.sleep(0.1)

    # Stop override
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
    )

    print("✅ Diagnostic complete!")
    print("📊 Analysis:")
    print("   - If RC Override showed 'OVERRIDE' but drone didn't move:")
    print("     → Flight controller is ignoring RC commands")
    print("   - If RC Override showed 'NORMAL':")
    print("     → RC override commands not being accepted")
    print("   - Try direct motor control next!")

if __name__ == "__main__":
    import math
    diagnose_control()
