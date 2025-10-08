#!/usr/bin/env python3
"""
Send DIRECT motor commands (bypass attitude controllers)
This works with the minimal build without needing MC controllers
"""

from pymavlink import mavutil
import time

# Connect to PX4
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udpin:127.0.0.1:14540')
master.wait_heartbeat()
print(f"✅ Connected! System {master.target_system}, Component {master.target_component}")

def get_state():
    """Get current vehicle state from heartbeat"""
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg:
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        return armed
    return None

def arm():
    """Arm the vehicle"""
    # Check if already armed
    armed = get_state()
    if armed:
        print("ℹ️  Vehicle already ARMED, skipping ARM command")
        return True

    print("\n🔧 Sending ARM command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if ack.result == 0:
            print("✅ ARM command ACCEPTED!")
            return True
        else:
            print(f"❌ ARM command DENIED with result: {ack.result}")
            print("   Try arming from PX4 console: pxh> minimal_commander arm")
            return False
    print("⚠️  No ACK received - check if PX4 is running")
    return False

def disarm():
    """Disarm the vehicle"""
    print("\n🔧 Sending DISARM command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.result == 0:
        print("✅ DISARM command ACCEPTED!")
        return True
    return False

def send_manual_control(roll, pitch, yaw, throttle):
    """
    Send manual control inputs (like RC sticks)

    Args:
        roll: -1000 to 1000 (left to right)
        pitch: -1000 to 1000 (back to forward)
        yaw: -1000 to 1000 (ccw to cw)
        throttle: 0 to 1000 (min to max)
    """
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
print("MANUAL CONTROL TEST - Direct Motor Commands")
print("="*60)
print("\nℹ️  Note: If ARM fails, manually arm first in PX4 console:")
print("   pxh> minimal_commander arm")
print("   Then run this script again.\n")

# ARM
if not arm():
    print("\n💡 TIP: Arm manually in PX4 console and try again")
    print("   In PX4 terminal: pxh> minimal_commander arm")
    print("   Then re-run: python3 manual_control_test.py")
    exit(1)

time.sleep(1)

print("\n📡 Sending manual control at 10Hz...")
print("Commands (RC sticks)         |  Feedback")
print(" Roll  Pitch  Yaw  Throttle  |  Roll°  Pitch°  Yaw°  |  Alt     Vz")
print("-" * 80)

try:
    rate = 10  # Hz
    interval = 1.0 / rate
    i = 0

    for _ in range(100):  # 10 seconds
        # Manual control values (-1000 to 1000 for roll/pitch/yaw, 0-1000 for throttle)
        roll_cmd = 0        # Center stick (no roll)
        pitch_cmd = 0       # Center stick (no pitch)
        yaw_cmd = 300       # Right stick (rotate clockwise)
        throttle_cmd = 600  # 60% throttle (hover)

        send_manual_control(roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd)

        # Monitor feedback every 10 iterations
        if i % 10 == 0:
            alt, vz, roll_fb, pitch_fb, yaw_fb = monitor_state()
            if alt is not None and roll_fb is not None:
                print(f"{roll_cmd:5d}  {pitch_cmd:5d}  {yaw_cmd:4d}  {throttle_cmd:4d}      | "
                      f"{roll_fb:6.1f}° {pitch_fb:6.1f}° {yaw_fb:6.1f}° | "
                      f"{alt:6.2f}m  {vz:+6.2f}m/s")

        i += 1
        time.sleep(interval)

    print("\n✅ Commands completed!")

except KeyboardInterrupt:
    print("\n⚠️  Interrupted by user")

# Landing: reduce throttle
print("\n🛬 Landing...")
for _ in range(30):
    send_manual_control(0, 0, 0, 300)  # Low throttle
    time.sleep(0.1)

# DISARM
time.sleep(1)
disarm()

print("\n" + "="*60)
print("✅ Test complete!")
print("="*60)
