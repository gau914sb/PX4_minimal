#!/usr/bin/env python3
"""
Check if control_allocator is actually running
"""
import subprocess
import time

print("Checking control_allocator status...")
print("=" * 70)

# Run the test to generate some activity first
print("\n0. Starting PX4 and waiting for it to stabilize...")
subprocess.Popen(['make', 'px4_sitl_default', 'none_iris'], 
                 cwd='/Users/gauravsinghbhati/Documents/PX4_minimal',
                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(5)

# Method 1: Check if it's running via dmesg
print("\n1. Checking startup logs for control_allocator:")
result = subprocess.run(
    ['./build/px4_sitl_default/bin/px4-listener', 'dmesg'],
    capture_output=True, text=True, timeout=5,
    cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
)
found = False
for line in result.stdout.split('\n'):
    if 'control_allocator' in line.lower() or 'allocator' in line.lower():
        print(f"  {line}")
        found = True

if not found:
    print("  ✗ NO control_allocator messages in startup logs!")

# Method 2: Check if actuator_motors is being published
print("\n2. Checking if actuator_motors topic is publishing:")
result = subprocess.run(
    ['./build/px4_sitl_default/bin/px4-listener', 'actuator_motors', '1'],
    capture_output=True, text=True, timeout=3,
    cwd='/Users/gauravsinghbhati/Documents/PX4_minimal'
)
if 'never published' in result.stdout:
    print("  ✗ actuator_motors NEVER PUBLISHED - control_allocator not running!")
elif result.stdout:
    print("  ✅ actuator_motors IS PUBLISHING!")
    print(result.stdout[:400])
else:
    print("  ⚠ Unknown status")

print("\n" + "=" * 70)
