#!/usr/bin/env python3
"""
Test minimal_commander console commands via MAVLink shell access
"""
from pymavlink import mavutil
import time

# Connect to PX4
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udpin:localhost:14540')
master.wait_heartbeat()
print(f"✅ Connected to PX4 system {master.target_system}\n")

def send_shell_command(cmd):
    """Send a shell command to PX4 and wait for response"""
    print(f"🖥️  Sending: {cmd}")
    # We can't directly execute shell commands via MAVLink in this simplified way
    # Instead, we'll use MAVLink commands
    time.sleep(0.5)

print("=" * 60)
print("MINIMAL_COMMANDER CONSOLE COMMANDS TEST")
print("=" * 60)
print()
print("Available commands when connected to PX4 console:")
print("  minimal_commander status   - Show detailed status")
print("  minimal_commander arm      - Arm the vehicle")
print("  minimal_commander takeoff  - Enable offboard/takeoff mode")
print("  minimal_commander disarm   - Disarm the vehicle")
print()
print("=" * 60)
print()

# Test via MAVLink commands instead
print("Testing via MAVLink commands...")
print()

# 1. ARM
print("1️⃣  ARMING via MAVLink...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
if msg and msg.result == 0:
    print("   ✅ Armed successfully")
else:
    print(f"   ❌ Arm failed: {msg.result if msg else 'timeout'}")
time.sleep(1)

# 2. TAKEOFF
print("\n2️⃣  TAKEOFF via MAVLink...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 2.5
)
msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
if msg and msg.result == 0:
    print("   ✅ Takeoff mode enabled")
else:
    print(f"   ❌ Takeoff failed: {msg.result if msg else 'timeout'}")
time.sleep(2)

# 3. Check heartbeat to see arming state
print("\n3️⃣  Checking vehicle state...")
msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
if msg:
    armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    print(f"   Armed: {armed}")
    print(f"   System status: {msg.system_status}")
    print(f"   Flight mode: {msg.custom_mode}")

# 4. DISARM
print("\n4️⃣  DISARMING via MAVLink...")
time.sleep(2)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0
)
msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
if msg and msg.result == 0:
    print("   ✅ Disarmed successfully")
else:
    print(f"   ❌ Disarm failed: {msg.result if msg else 'timeout'}")

print("\n" + "=" * 60)
print("✅ Test complete!")
print()
print("To test console commands directly, run PX4 interactively:")
print("  cd /Users/gauravsinghbhati/Documents/PX4-Autopilot")
print("  ./build/px4_sitl_default/bin/px4")
print()
print("Then at the pxh> prompt, type:")
print("  pxh> minimal_commander status")
print("  pxh> minimal_commander arm")
print("  pxh> minimal_commander takeoff")
print("  pxh> minimal_commander disarm")
print("=" * 60)
