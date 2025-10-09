#!/usr/bin/env python3
"""
Check if control_allocator has loaded CA parameters correctly
"""

import time
from pymavlink import mavutil

# Connect to PX4
master = mavutil.mavlink_connection('udpin:127.0.0.1:14540')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"✅ Connected to system {master.target_system}, component {master.target_component}")

# Request parameters we need
params_to_check = [
    'CA_AIRFRAME',
    'CA_ROTOR_COUNT',
    'CA_ROTOR0_PX',
    'CA_ROTOR0_PY',
    'MAV_TYPE',
]

print("\n" + "="*60)
print("CHECKING CA PARAMETERS")
print("="*60)

for param_name in params_to_check:
    # Request parameter
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1
    )
    
    # Wait for response (with timeout)
    start = time.time()
    received = False
    while time.time() - start < 2.0:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
        if msg and msg.param_id.decode('utf-8').rstrip('\x00') == param_name:
            print(f"  {param_name:20s} = {msg.param_value}")
            received = True
            break
    
    if not received:
        print(f"  {param_name:20s} = NOT FOUND ❌")

print("="*60)
