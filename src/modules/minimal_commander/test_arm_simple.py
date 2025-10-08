#!/usr/bin/env python3
"""
Simple ARM + TAKEOFF test using pymavlink
Tests minimal_commander with offboard control
"""
from pymavlink import mavutil
import time

# Connect to PX4
print("Connecting to PX4...")
master = mavutil.mavlink_connection('udpin:localhost:14540')

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"✅ Heartbeat from system {master.target_system} component {master.target_component}")

# Send ARM command
print("\n🚁 Sending ARM command (MAV_CMD_COMPONENT_ARM_DISARM = 400)")
print(f"   Target: system={master.target_system}, component={master.target_component}")

master.mav.command_long_send(
    master.target_system,  # target_system
    master.target_component,  # target_component  
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command (400)
    0,  # confirmation
    1,  # param1 (1=arm, 0=disarm)
    0,  # param2
    0,  # param3
    0,  # param4
    0,  # param5
    0,  # param6
    0   # param7
)

print("✅ ARM command sent!")

# Wait for acknowledgment
print("Waiting for ARM ACK...")
msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if msg:
    print(f"📬 ACK received: result={msg.result}, command={msg.command}")
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ ARM command ACCEPTED!")
    else:
        print(f"❌ ARM command rejected: {msg.result}")
        print("\nDone!")
        exit(1)
else:
    print("⏱️  No ACK received (timeout)")
    print("\nDone!")
    exit(1)

# Wait a bit for system to stabilize
print("\n⏳ Waiting 2 seconds for system to stabilize...")
time.sleep(2)

# Send TAKEOFF command
print("\n🚁 Sending TAKEOFF command (MAV_CMD_NAV_TAKEOFF = 22)")
print(f"   Target altitude: 2.5 meters")

master.mav.command_long_send(
    master.target_system,  # target_system
    master.target_component,  # target_component  
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command (22)
    0,  # confirmation
    0,  # param1 (pitch)
    0,  # param2 (empty)
    0,  # param3 (empty)
    0,  # param4 (yaw angle)
    0,  # param5 (latitude)
    0,  # param6 (longitude)
    2.5  # param7 (altitude in meters)
)

print("✅ TAKEOFF command sent!")

# Wait for acknowledgment
print("Waiting for TAKEOFF ACK...")
msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if msg:
    print(f"📬 ACK received: result={msg.result}, command={msg.command}")
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ TAKEOFF command ACCEPTED!")
    else:
        print(f"❌ TAKEOFF command rejected: {msg.result}")
else:
    print("⏱️  No ACK received (timeout)")

print("✅ TAKEOFF command sent!")

# Wait for acknowledgment
print("Waiting for TAKEOFF ACK...")
msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
if msg:
    print(f"📬 ACK received: result={msg.result}, command={msg.command}")
    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ TAKEOFF command ACCEPTED!")
    else:
        print(f"❌ TAKEOFF command rejected: {msg.result}")
else:
    print("⚠️  No ACK received (continuing anyway - minimal_commander may not ACK TAKEOFF)")

# Now send offboard attitude/thrust setpoints for takeoff
print("\n🚀 Sending OFFBOARD attitude setpoints for takeoff...")
print("   Sending thrust commands to climb to 2.5m...")

target_altitude = 2.5
current_altitude = 0.0
start_time = time.time()
climb_duration = 10  # seconds to reach target altitude

# Get boot time for timestamp
boot_time_ms = 0

while time.time() - start_time < climb_duration:
    # Calculate thrust (0.6 = hover, 0.7-0.8 = climb)
    elapsed = time.time() - start_time
    boot_time_ms = int(elapsed * 1000)  # Convert to milliseconds
    
    if elapsed < 2:
        thrust = 0.75  # Initial climb thrust
    elif current_altitude < target_altitude - 0.3:
        thrust = 0.65  # Moderate climb
    else:
        thrust = 0.55  # Hover at target altitude
    
    # Send attitude setpoint with thrust
    # SET_ATTITUDE_TARGET message (type 82)
    master.mav.set_attitude_target_send(
        boot_time_ms,  # time_boot_ms (integer milliseconds)
        master.target_system,  # target system
        master.target_component,  # target component
        0b00000111,  # type_mask (ignore attitude, use rates)
        [1.0, 0.0, 0.0, 0.0],  # q (quaternion, w,x,y,z) - level attitude
        0.0,  # body_roll_rate
        0.0,  # body_pitch_rate  
        0.0,  # body_yaw_rate
        thrust  # thrust (0 to 1)
    )
    
    # Check altitude
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.05)
    if msg:
        current_altitude = -msg.z  # NED frame, negate Z
        vz = -msg.vz
        print(f"  Alt: {current_altitude:.2f}m | Vz: {vz:.2f}m/s | Thrust: {thrust:.2f}")
    
    time.sleep(0.1)  # 10Hz update rate

# Hover for a bit
print("\n✈️  Hovering at target altitude...")
hover_start = time.time()
for i in range(30):
    elapsed = time.time() - start_time
    boot_time_ms = int(elapsed * 1000)
    
    master.mav.set_attitude_target_send(
        boot_time_ms,
        master.target_system,
        master.target_component,
        0b00000111,
        [1.0, 0.0, 0.0, 0.0],
        0.0, 0.0, 0.0,
        0.55  # Hover thrust
    )
    
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.05)
    if msg:
        alt = -msg.z
        print(f"  Hovering: Alt={alt:.2f}m")
    
    time.sleep(0.1)

# Land by reducing thrust
print("\n🛬 Landing...")
land_start = time.time()
for i in range(30):
    elapsed = time.time() - start_time
    boot_time_ms = int(elapsed * 1000)
    
    thrust = max(0.2, 0.55 - (i * 0.012))  # Gradually reduce thrust
    master.mav.set_attitude_target_send(
        boot_time_ms,
        master.target_system,
        master.target_component,
        0b00000111,
        [1.0, 0.0, 0.0, 0.0],
        0.0, 0.0, 0.0,
        thrust
    )
    
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.05)
    if msg:
        alt = -msg.z
        print(f"  Landing: Alt={alt:.2f}m | Thrust={thrust:.2f}")
    
    time.sleep(0.1)

# Disarm
print("\n🔒 Disarming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,  # 0 = disarm
    0, 0, 0, 0, 0, 0
)

msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("✅ DISARMED successfully!")
else:
    print("⚠️  Disarm command failed or timed out")

print("\n✅ Flight test complete!")
