#!/usr/bin/env python3
"""
Direct check: Listen to PX4 topics to verify control chain
"""

from pymavlink import mavutil
import time

print("Connecting...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14540')
mav.wait_heartbeat()
print("✓ Connected\n")

# Arm
print("Arming...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(1)

# Switch to OFFBOARD
print("Switching to OFFBOARD...")
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0, 1, 6, 0, 0, 0, 0, 0)
time.sleep(1)

print("\n✓ Setup complete")
print("Now sending attitude setpoints and checking servo outputs...\n")

# Send attitude setpoints and monitor servo outputs
for i in range(100):  # 2 seconds at 50Hz
    # Send attitude setpoint
    mav.mav.set_attitude_target_send(
        0, mav.target_system, mav.target_component,
        0b00000111,  # ignore rates
        [1, 0, 0, 0],  # level attitude
        0, 0, 0,
        0.6  # 60% thrust
    )
    
    # Check for servo outputs every 10 messages
    if i % 10 == 0:
        # Try multiple message types to find motor output
        for msg_type in ['SERVO_OUTPUT_RAW', 'ACTUATOR_OUTPUT_STATUS', 'HIL_ACTUATOR_CONTROLS']:
            msg = mav.recv_match(type=msg_type, blocking=False)
            if msg:
                print(f"{msg_type}: {msg}")
                break
    
    time.sleep(0.02)

print("\n✓ Test complete")
print("\nIf you see SERVO_OUTPUT_RAW or ACTUATOR_OUTPUT_STATUS with non-zero values,")
print("the control chain is working end-to-end!")
