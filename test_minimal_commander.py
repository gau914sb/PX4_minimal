#!/usr/bin/env python3
"""
Test script to verify minimal_commander parameter access via MAVLink
"""

from pymavlink import mavutil
import time

# Connect to PX4 SITL via UDP
print("Connecting to PX4 SITL via UDP...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14550', source_system=255)

# Wait for heartbeat
print("Waiting for heartbeat...")
mav.wait_heartbeat()
print(f"Connected! System ID: {mav.target_system}, Component ID: {mav.target_component}")

# Test parameters
test_params = [
    'COM_RC_IN_MODE',
    'COM_RC_LOSS_T',
    'COM_RC_STICK_OV',
    'COM_RC_ARM_HYST',
    'COM_ARM_SWISBTN',
    'COM_LOW_BAT_ACT',
    'COM_FLTMODE1',
    'COM_FLTMODE2',
    'COM_FLTMODE3',
    'COM_FLTMODE4',
    'COM_FLTMODE5',
    'COM_FLTMODE6',
    'BAT_LOW_THR'
]

print("\n" + "="*60)
print("TESTING EXTRACTED PARAMETERS")
print("="*60)

for param_name in test_params:
    print(f"\nRequesting parameter: {param_name}")

    # Request parameter value
    mav.mav.param_request_read_send(
        mav.target_system,
        mav.target_component,
        param_name.encode('utf-8'),
        -1
    )

    # Wait for response
    msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)

    if msg:
        print(f"  ✅ {msg.param_id}: {msg.param_value} (type: {msg.param_type})")
    else:
        print(f"  ❌ Timeout - parameter not found!")

print("\n" + "="*60)
print("VERIFICATION COMPLETE")
print("="*60)
