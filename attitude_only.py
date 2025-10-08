#!/usr/bin/env python3
"""
Send ONLY attitude commands (assumes vehicle is already armed)
"""

from pymavlink import mavutil
import time
import math

# Connect to PX4
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udpin:127.0.0.1:14540')
master.wait_heartbeat()
print(f"✅ Connected! System {master.target_system}, Component {master.target_component}")

start_time = time.time()

def send_attitude_target(roll, pitch, yaw, thrust):
    """Send attitude setpoint"""
    elapsed = time.time() - start_time
    boot_time_ms = int(elapsed * 1000) & 0xFFFFFFFF

    # Calculate quaternion from Euler angles
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy

    master.mav.set_attitude_target_send(
        boot_time_ms,
        master.target_system,
        master.target_component,
        0b00000000,  # type_mask
        [q_w, q_x, q_y, q_z],
        0, 0, 0,  # body rates
        thrust
    )

def monitor_altitude():
    """Monitor current altitude"""
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.01)
    if msg:
        altitude = -msg.z
        vz = -msg.vz
        return altitude, vz
    return None, None

print("\n" + "="*60)
print("ATTITUDE COMMANDS - Sending at 10Hz")
print("="*60)
print("Press Ctrl+C to stop")
print("\n     Roll    Pitch    Yaw    Thrust  |  Alt     Vz")
print("-" * 60)

try:
    rate = 10  # Hz
    interval = 1.0 / rate
    i = 0

    while True:
        # Level hover attitude
        roll = 0.0      # radians
        pitch = 0.0     # radians
        yaw = 0.0       # radians
        thrust = 0.6    # 60% thrust

        send_attitude_target(roll, pitch, yaw, thrust)

        # Monitor altitude every 10 iterations (1 second)
        if i % 10 == 0:
            alt, vz = monitor_altitude()
            if alt is not None:
                print(f"  {roll:6.2f}  {pitch:6.2f}  {yaw:5.2f}  {thrust:5.2f}  | {alt:6.2f}m  {vz:+6.2f}m/s")

        i += 1
        time.sleep(interval)

except KeyboardInterrupt:
    print("\n\n⚠️  Interrupted by user")
    print("Sending zero thrust for 2 seconds...")

    # Send zero thrust to land
    for _ in range(20):
        send_attitude_target(0, 0, 0, 0.0)
        time.sleep(0.1)

    print("✅ Done!")
