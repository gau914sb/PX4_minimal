#!/usr/bin/env python3
"""
Debug: Check if vehicle_attitude_setpoint is being published
"""
import subprocess
import time
import signal
import sys

# Start PX4 if not running
print("Checking PX4 topics during attitude control...")
print("=" * 60)

# Create a simple test that sends attitude commands
test_script = """
from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection('udp:127.0.0.1:14540', timeout=5)
mav.wait_heartbeat(timeout=5)

# Arm
mav.mav.command_long_send(mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(1)

# OFFBOARD
mav.mav.command_long_send(mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
time.sleep(1)

# Send setpoints for 10 seconds
for i in range(500):
    mav.mav.set_attitude_target_send(0, mav.target_system, mav.target_component,
        0b00000111, [1, 0, 0, 0], 0, 0, 0, 0.6)
    time.sleep(0.02)
"""

# Write test script
with open('/tmp/quick_test.py', 'w') as f:
    f.write(test_script)

# Run test in background
print("1. Starting attitude control test...")
test_proc = subprocess.Popen(['python3', '/tmp/quick_test.py'], 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL)
time.sleep(3)  # Wait for arm + offboard

print("2. Checking vehicle_attitude_setpoint topic...")
try:
    result = subprocess.run(
        ['./build/px4_sitl_default/bin/px4-listener', 'vehicle_attitude_setpoint', '5'],
        capture_output=True, text=True, timeout=6, cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
    )
    if result.stdout:
        print("✓ vehicle_attitude_setpoint IS being published:")
        print(result.stdout[:500])
    else:
        print("✗ NO vehicle_attitude_setpoint data!")
except Exception as e:
    print(f"✗ Error: {e}")

print("\n3. Checking vehicle_rates_setpoint topic...")
try:
    result = subprocess.run(
        ['./build/px4_sitl_default/bin/px4-listener', 'vehicle_rates_setpoint', '5'],
        capture_output=True, text=True, timeout=6, cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
    )
    if result.stdout:
        print("✓ vehicle_rates_setpoint IS being published:")
        print(result.stdout[:500])
    else:
        print("✗ NO vehicle_rates_setpoint data!")
except Exception as e:
    print(f"✗ Error: {e}")

print("\n4. Checking vehicle_torque_setpoint topic...")
try:
    result = subprocess.run(
        ['./build/px4_sitl_default/bin/px4-listener', 'vehicle_torque_setpoint', '5'],
        capture_output=True, text=True, timeout=6, cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
    )
    if result.stdout:
        print("✓ vehicle_torque_setpoint IS being published:")
        print(result.stdout[:500])
    else:
        print("✗ NO vehicle_torque_setpoint data!")
except Exception as e:
    print(f"✗ Error: {e}")

print("\n5. Checking actuator_motors topic...")
try:
    result = subprocess.run(
        ['./build/px4_sitl_default/bin/px4-listener', 'actuator_motors', '5'],
        capture_output=True, text=True, timeout=6, cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
    )
    if result.stdout:
        print("✓ actuator_motors IS being published:")
        print(result.stdout[:500])
    else:
        print("✗ NO actuator_motors data!")
except Exception as e:
    print(f"✗ Error: {e}")

# Cleanup
test_proc.terminate()
print("\n" + "=" * 60)
print("Analysis: Check which topics show data above")
print("The FIRST missing topic is where the control chain breaks!")
