#!/usr/bin/env python3
"""
Send attitude commands to PX4 minimal_commander via MAVLink
"""

from pymavlink import mavutil
import time
import math

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
        0,  # confirmation
        1,  # param1: 1 to arm
        0, 0, 0, 0, 0, 0
    )
    # Wait for acknowledgment
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if ack.result == 0:
            print("✅ ARM command ACCEPTED!")
            return True
        else:
            print(f"❌ ARM command failed with result: {ack.result}")
            return False
    print("⚠️  No ACK received")
    return False

def disarm():
    """Disarm the vehicle"""
    print("\n🔧 Sending DISARM command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # param1: 0 to disarm
        0, 0, 0, 0, 0, 0
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        if ack.result == 0:
            print("✅ DISARM command ACCEPTED!")
            return True
    return False

def takeoff():
    """Send takeoff command"""
    print("\n🚁 Sending TAKEOFF command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0, 0, 0, 0, 0, 0, 2.0  # param7: altitude 2m
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        if ack.result == 0:
            print("✅ TAKEOFF command ACCEPTED!")
            return True
    return False

def send_attitude_target(roll, pitch, yaw, thrust):
    """
    Send attitude setpoint

    Args:
        roll: Roll angle in radians (-pi to pi)
        pitch: Pitch angle in radians (-pi to pi)
        yaw: Yaw angle in radians (-pi to pi)
        thrust: Thrust value (0.0 to 1.0)
    """
    # Get current time (milliseconds since boot)
    elapsed = time.time() - start_time
    boot_time_ms = int(elapsed * 1000) & 0xFFFFFFFF  # Keep within uint32 range

    # Calculate quaternion from Euler angles (ZYX order)
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

    # Type mask:
    # bit 7 (0x80) = ignore attitude (we use quaternion)
    # bit 0-2 = ignore body roll/pitch/yaw rates
    # To control yaw rate: DON'T set bit 2
    type_mask = 0b10000011  # Ignore attitude quaternion, control via rates instead

    # For this test, we'll use body rates instead of quaternion for better visibility
    # Calculate desired rates from attitude (simple P controller)
    roll_rate = roll * 2.0    # rad/s (proportional to desired roll)
    pitch_rate = pitch * 2.0  # rad/s
    yaw_rate = yaw           # rad/s (this is what we want to control)

    master.mav.set_attitude_target_send(
        boot_time_ms,           # time_boot_ms
        master.target_system,   # target system
        master.target_component,# target component
        type_mask,              # type_mask
        [q_w, q_x, q_y, q_z],   # quaternion (w, x, y, z) - will be ignored due to type_mask
        roll_rate, pitch_rate, yaw_rate,  # body roll rate, pitch rate, yaw rate (rad/s)
        thrust                  # thrust (0 to 1)
    )

def monitor_altitude():
    """Monitor current altitude"""
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if msg:
        altitude = -msg.z  # NED frame, so negate Z
        vz = -msg.vz
        return altitude, vz
    return None, None

def monitor_attitude():
    """Monitor actual drone attitude (roll, pitch, yaw)"""
    msg = master.recv_match(type='ATTITUDE', blocking=False, timeout=0.1)
    if msg:
        # Convert radians to degrees for easier reading
        roll_deg = math.degrees(msg.roll)
        pitch_deg = math.degrees(msg.pitch)
        yaw_deg = math.degrees(msg.yaw)
        return roll_deg, pitch_deg, yaw_deg
    return None, None, None

# Main flight sequence
print("\n" + "="*60)
print("ATTITUDE CONTROL TEST - Minimal Commander")
print("="*60)

start_time = time.time()

# Step 1: ARM
if not arm():
    print("❌ Failed to arm. Exiting.")
    exit(1)

time.sleep(1)

# Step 2: TAKEOFF (enables offboard mode)
if not takeoff():
    print("❌ Failed to takeoff. Exiting.")
    exit(1)

time.sleep(1)

# Step 3: Send attitude commands
print("\n📡 Sending attitude rate commands at 10Hz...")
print("COMMANDED RATES (rad/s)      |  ACTUAL FEEDBACK")
print("Roll   Pitch   Yaw   Thrust  |  Roll°  Pitch°  Yaw°  |  Alt     Vz")
print("-" * 80)

try:
    # Send commands for 10 seconds
    duration = 10.0
    rate = 10  # Hz
    interval = 1.0 / rate

    for i in range(int(duration * rate)):
        # Hover attitude with rotating yaw
        roll = 0.0      # rad/s (keep level)
        pitch = 0.0     # rad/s (keep level)
        yaw = 0.5       # rad/s (about 28.6 degrees/second - should rotate ~286° in 10 sec)
        thrust = 0.6    # 60% thrust (adjust as needed)

        # Alternative: Oscillating yaw rate (uncomment to test)
        # yaw = 0.5 * math.sin(2 * math.pi * i / (duration * rate))  # Swing back and forth

        send_attitude_target(roll, pitch, yaw, thrust)

        # Monitor feedback every 10 iterations
        if i % 10 == 0:
            alt, vz = monitor_altitude()
            roll_actual, pitch_actual, yaw_actual = monitor_attitude()
            if alt is not None and roll_actual is not None:
                print(f"{roll:5.2f}  {pitch:5.2f}  {yaw:5.2f}  {thrust:5.2f}  | "
                      f"{roll_actual:6.1f}° {pitch_actual:6.1f}° {yaw_actual:6.1f}° | "
                      f"{alt:6.2f}m  {vz:+6.2f}m/s")

        time.sleep(interval)

    print("\n✅ Attitude commands completed!")

except KeyboardInterrupt:
    print("\n⚠️  Interrupted by user")

# Step 4: Reduce thrust for landing
print("\n🛬 Landing...")
for i in range(30):
    send_attitude_target(0, 0, 0, 0.3)  # Low thrust
    time.sleep(0.1)

# Step 5: DISARM
time.sleep(1)
disarm()

print("\n" + "="*60)
print("✅ Test complete!")
print("="*60)
