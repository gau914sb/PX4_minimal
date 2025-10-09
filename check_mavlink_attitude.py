#!/usr/bin/env python3
"""
Direct check: Are attitude setpoints being published?
"""
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
print("Connected!\n")

# Send one attitude command
print("Sending SET_ATTITUDE_TARGET...")
master.mav.set_attitude_target_send(
    0,  # time_boot_ms
    master.target_system,
    master.target_component,
    0b00000111,  # type_mask
    [1, 0, 0, 0],  # quaternion
    0, 0, 0,  # body rates
    0.6  # thrust
)

print("Waiting for messages...\n")

# Listen for what comes back
start = time.time()
count = 0
while time.time() - start < 3:
    msg = master.recv_match(blocking=False, timeout=0.1)
    if msg:
        msg_type = msg.get_type()
        if msg_type in ['ATTITUDE_TARGET', 'STATUSTEXT', 'COMMAND_ACK']:
            count += 1
            print(f"[{count}] {msg_type}: {msg}")

print(f"\nReceived {count} relevant messages in 3 seconds")
