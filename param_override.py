#!/usr/bin/env python3
"""
PX4 Parameter Override for Takeoff Issues
Overrides parameters that might prevent takeoff
"""

import time
from pymavlink import mavutil

def override_parameters():
    print("=" * 50)
    print("PX4 PARAMETER OVERRIDE FOR TAKEOFF")
    print("=" * 50)

    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}")

    # Parameters that might help with takeoff
    params_to_set = [
        ("COM_ARM_WO_GPS", 1),      # Allow arming without GPS
        ("COM_DISARM_PRFLT", 0),    # Disable preflight disarming
        ("MC_AIRMODE", 1),          # Enable airmode for better control
        ("COM_TAKEOFF_ACT", 0),     # Disable takeoff action requirement
        ("NAV_ACC_RAD", 10.0),      # Acceptance radius
        ("RTL_RETURN_ALT", 30.0),   # Return altitude
    ]

    print("🔧 Setting parameters...")
    for param_name, value in params_to_set:
        try:
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                param_name.encode('utf-8'),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32 if isinstance(value, float) else mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            print(f"  ✅ {param_name} = {value}")
            time.sleep(0.5)
        except Exception as e:
            print(f"  ❌ Failed to set {param_name}: {e}")

    print("\n🚀 Attempting direct takeoff...")

    # Force arm with override
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm
        21196,  # force override
        0, 0, 0, 0, 0
    )

    time.sleep(2)

    # Manual throttle control
    print("🎮 Manual throttle control...")
    for i in range(50):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500,  # roll
            1500,  # pitch
            1800,  # throttle (80%)
            1500,  # yaw
            0, 0, 0, 0
        )
        time.sleep(0.1)

    # Return control
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
    )

    print("✅ Takeoff attempt complete!")

if __name__ == "__main__":
    override_parameters()
