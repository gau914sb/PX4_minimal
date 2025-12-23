#!/usr/bin/env python3
"""
Simple example: Send attitude + thrust commands to PX4
This will make the drone hover at 0.55 thrust with level attitude
"""

import time
from pymavlink import mavutil

# Connect to PX4
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
print(f"✓ Connected! System {master.target_system}")

# Type mask for attitude + thrust control
ATTITUDE_TYPEMASK = 0b00000111  # Ignore body rates, use quaternion + thrust

def send_attitude_thrust(roll_deg, pitch_deg, yaw_deg, thrust):
    """
    Send attitude setpoint with thrust

    Args:
        roll_deg: Roll angle in degrees (-180 to 180)
        pitch_deg: Pitch angle in degrees (-90 to 90)
        yaw_deg: Yaw angle in degrees (-180 to 180)
        thrust: Thrust value (0.0 to 1.0, ~0.55 for hover)
    """
    import math

    # Convert Euler angles to quaternion
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    q = [w, x, y, z]

    time_boot_ms = int(time.time() * 1000) % 4294967295

    master.mav.set_attitude_target_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        ATTITUDE_TYPEMASK,
        q,           # Quaternion [w, x, y, z]
        0, 0, 0,    # Body rates (ignored)
        thrust      # Thrust (0-1)
    )

print("\nSending attitude + thrust commands...")
print("Press Ctrl+C to stop\n")

try:
    # Send at 50 Hz for smooth control
    while True:
        # Example: level attitude with hover thrust
        send_attitude_thrust(
            roll_deg=0,      # Level roll
            pitch_deg=0,     # Level pitch
            yaw_deg=0,       # North heading
            thrust=0.55      # Hover thrust (~50-60%)
        )

        time.sleep(0.02)  # 50 Hz

except KeyboardInterrupt:
    print("\nStopped sending commands")
    # Send zero thrust before exit
    for _ in range(10):
        send_attitude_thrust(0, 0, 0, 0.0)
        time.sleep(0.02)
    print("✓ Safely stopped")
