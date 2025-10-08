#!/usr/bin/env python3
"""
Send ONLY manual control commands (assumes vehicle is already armed)
Use this if ARM/DISARM commands aren't working
"""

from pymavlink import mavutil
import time

# Connect to PX4
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udpin:127.0.0.1:14540')
master.wait_heartbeat()
print(f"✅ Connected! System {master.target_system}, Component {master.target_component}")

def send_manual_control(roll, pitch, yaw, throttle):
    """Send manual control inputs (like RC sticks)"""
    master.mav.manual_control_send(
        master.target_system,
        int(pitch),      # x: pitch
        int(roll),       # y: roll
        int(throttle),   # z: throttle
        int(yaw),        # r: yaw
        0                # buttons
    )

def monitor_state():
    """Monitor altitude and attitude"""
    pos_msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.01)
    att_msg = master.recv_match(type='ATTITUDE', blocking=False, timeout=0.01)

    alt, vz, roll, pitch, yaw = None, None, None, None, None

    if pos_msg:
        alt = -pos_msg.z
        vz = -pos_msg.vz

    if att_msg:
        import math
        roll = math.degrees(att_msg.roll)
        pitch = math.degrees(att_msg.pitch)
        yaw = math.degrees(att_msg.yaw)

    return alt, vz, roll, pitch, yaw

print("\n" + "="*60)
print("MANUAL CONTROL - Sending Commands at 10Hz")
print("="*60)
print("\n⚠️  IMPORTANT: Arm the vehicle FIRST in PX4 console:")
print("   pxh> minimal_commander arm\n")
print("Press Ctrl+C to stop\n")

print("Commands (RC sticks)         |  Feedback")
print(" Roll  Pitch  Yaw  Throttle  |  Roll°  Pitch°  Yaw°  |  Alt     Vz")
print("-" * 80)

try:
    rate = 10  # Hz
    interval = 1.0 / rate
    i = 0

    while True:
        # Manual control values
        roll_cmd = 0        # Center stick (no roll)
        pitch_cmd = 0       # Center stick (no pitch)
        yaw_cmd = 300       # Right stick (rotate clockwise ~30%)
        throttle_cmd = 600  # 60% throttle (hover)

        send_manual_control(roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd)

        # Monitor feedback every 10 iterations (1 second)
        if i % 10 == 0:
            alt, vz, roll_fb, pitch_fb, yaw_fb = monitor_state()
            if alt is not None and roll_fb is not None:
                print(f"{roll_cmd:5d}  {pitch_cmd:5d}  {yaw_cmd:4d}  {throttle_cmd:4d}      | "
                      f"{roll_fb:6.1f}° {pitch_fb:6.1f}° {yaw_fb:6.1f}° | "
                      f"{alt:6.2f}m  {vz:+6.2f}m/s")

        i += 1
        time.sleep(interval)

except KeyboardInterrupt:
    print("\n\n⚠️  Stopped by user")
    print("🛬 Sending zero throttle for 2 seconds...")

    # Reduce throttle to land
    for _ in range(20):
        send_manual_control(0, 0, 0, 0)
        time.sleep(0.1)

    print("\n💡 Remember to disarm in PX4 console:")
    print("   pxh> minimal_commander disarm")
    print("\n✅ Done!")
