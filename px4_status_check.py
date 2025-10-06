#!/usr/bin/env python3
"""
PX4 System Status Check
Checks what modules are actually running in PX4
"""

import time
from pymavlink import mavutil

def check_px4_status():
    print("=" * 60)
    print("PX4 SYSTEM STATUS CHECK")
    print("=" * 60)

    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}")

    # Send a command to get system status
    print("\n🔍 Requesting system status...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
        0, 1, 0, 0, 0, 0, 0, 0
    )

    # Monitor messages for a while
    print("📡 Monitoring system messages...")
    start_time = time.time()
    while time.time() - start_time < 5:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            if msg_type in ['AUTOPILOT_VERSION', 'SYS_STATUS', 'HEARTBEAT']:
                print(f"📋 {msg_type}: {msg}")

    print("\n🧪 Testing if we can send commands to PX4 console...")

    # Try to get module status via MAVLink shell
    master.mav.serial_control_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
        mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
        0,  # timeout
        b"ps\n"  # List running processes
    )

    time.sleep(2)

    # Check for shell response
    for i in range(10):
        msg = master.recv_match(type='SERIAL_CONTROL', blocking=False)
        if msg:
            try:
                response = msg.data.decode('utf-8')
                print(f"🖥️  PX4 Console Output: {response}")
            except:
                print(f"🖥️  PX4 Console Raw: {msg.data}")

    print("\n✅ Status check complete!")

if __name__ == "__main__":
    try:
        check_px4_status()
    except Exception as e:
        print(f"❌ Error: {e}")
