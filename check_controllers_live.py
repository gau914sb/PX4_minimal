#!/usr/bin/env python3
"""
Check if controllers are actually processing setpoints in real-time
"""
import subprocess
import time

print("Checking if mc_att_control is actively processing setpoints...")
print("=" * 70)

# First, check vehicle_attitude_setpoint (INPUT to controller)
print("\n1. INPUT to mc_att_control: vehicle_attitude_setpoint")
print("-" * 70)
result = subprocess.run(
    ['./build/px4_sitl_default/bin/px4-listener', 'vehicle_attitude_setpoint', '1'],
    capture_output=True, text=True, timeout=3, 
    cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
)
for line in result.stdout.split('\n'):
    if 'thrust_body' in line or 'q_d' in line:
        print(line)

# Check vehicle_rates_setpoint (OUTPUT from mc_att_control)
print("\n2. OUTPUT from mc_att_control: vehicle_rates_setpoint")
print("-" * 70)
result = subprocess.run(
    ['./build/px4_sitl_default/bin/px4-listener', 'vehicle_rates_setpoint', '1'],
    capture_output=True, text=True, timeout=3,
    cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
)
for line in result.stdout.split('\n'):
    if 'roll:' in line or 'pitch:' in line or 'yaw:' in line or 'thrust_body' in line:
        print(line)

# Check vehicle_torque_setpoint (OUTPUT from mc_rate_control)
print("\n3. OUTPUT from mc_rate_control: vehicle_torque_setpoint")
print("-" * 70)
result = subprocess.run(
    ['./build/px4_sitl_default/bin/px4-listener', 'vehicle_torque_setpoint', '1'],
    capture_output=True, text=True, timeout=3,
    cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
)
for line in result.stdout.split('\n'):
    if 'xyz' in line or 'timestamp' in line:
        print(line)

# Check actuator_motors (OUTPUT from control_allocator)
print("\n4. OUTPUT from control_allocator: actuator_motors")
print("-" * 70)
result = subprocess.run(
    ['./build/px4_sitl_default/bin/px4-listener', 'actuator_motors', '1'],
    capture_output=True, text=True, timeout=3,
    cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
)
print(result.stdout[:800] if result.stdout else "✗ NO DATA")

# Check actuator_controls_0 (alternative output)
print("\n5. Alternative output: actuator_controls_0")
print("-" * 70)
result = subprocess.run(
    ['./build/px4_sitl_default/bin/px4-listener', 'actuator_controls_0', '1'],
    capture_output=True, text=True, timeout=3,
    cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
)
print(result.stdout[:800] if result.stdout else "✗ NO DATA")

print("\n" + "=" * 70)
print("Analysis:")
print("- If thrust_body shows -0.6 in vehicle_attitude_setpoint: ✓ MAVLink OK")
print("- If vehicle_rates_setpoint has thrust_body: ✓ mc_att_control OK")
print("- If vehicle_torque_setpoint has values: ✓ mc_rate_control OK")
print("- If actuator_motors has values: ✓ control_allocator OK")
print("- If all above OK but thrust still 0.00: Problem is in test script feedback")
