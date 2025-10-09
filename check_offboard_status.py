#!/usr/bin/env python3
"""
Quick diagnostic to check why OFFBOARD mode isn't working
"""
from pymavlink import mavutil
import time

print("\n" + "="*60)
print("OFFBOARD MODE DIAGNOSTIC")
print("="*60)

# Connect
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
print("\nWaiting for heartbeat...")
master.wait_heartbeat()
print(f"✓ Connected to System {master.target_system}")

# Wait for initial messages
time.sleep(1)

# Check for STATUSTEXT messages (error messages from PX4)
print("\n--- Checking for PX4 error messages ---")
print("Listening for 3 seconds...\n")

start = time.time()
errors_found = False
while time.time() - start < 3:
    msg = master.recv_match(type='STATUSTEXT', blocking=False, timeout=0.1)
    if msg:
        errors_found = True
        severity = msg.severity
        text = msg.text
        severity_names = {0: 'EMERG', 1: 'ALERT', 2: 'CRIT', 3: 'ERROR', 
                         4: 'WARNING', 5: 'NOTICE', 6: 'INFO', 7: 'DEBUG'}
        print(f"[{severity_names.get(severity, severity)}] {text}")

if not errors_found:
    print("(No error messages captured)")

# Check current mode
print("\n--- Current Flight Mode ---")
hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if hb:
    base_mode = hb.base_mode
    custom_mode = hb.custom_mode
    armed = base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    print(f"Base Mode: {base_mode} (Armed: {bool(armed)})")
    print(f"Custom Mode: {custom_mode}")
    
    # PX4 custom mode decoding
    mode_names = {
        0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
        4: "AUTO_LOITER", 5: "AUTO_RTL", 6: "ACRO", 7: "OFFBOARD",
        8: "STABILIZED", 9: "RATTITUDE", 10: "AUTO_TAKEOFF",
        11: "AUTO_LAND", 12: "AUTO_FOLLOW_TARGET", 13: "AUTO_PRECLAND"
    }
    mode_name = mode_names.get(custom_mode, f"UNKNOWN({custom_mode})")
    print(f"Mode Name: {mode_name}")
else:
    print("⚠ No heartbeat received")

print("\n" + "="*60)
print("DIAGNOSTIC COMPLETE")
print("="*60 + "\n")
