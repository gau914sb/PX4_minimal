#!/bin/bash
# Test script for minimal_commander console commands

echo "Starting PX4 SITL..."
cd /Users/gauravsinghbhati/Documents/PX4-Autopilot
./build/px4_sitl_default/bin/px4 -d -s etc/init.d-posix/rcS > /dev/null 2>&1

sleep 5

echo ""
echo "=== Testing minimal_commander console commands ==="
echo ""

# Test status command
echo "📊 1. Checking initial status..."
./build/px4_sitl_default/bin/px4-commander minimal_commander status
sleep 1

# Test arm command
echo ""
echo "🔓 2. Arming vehicle..."
./build/px4_sitl_default/bin/px4-commander minimal_commander arm
sleep 1

# Check status after arming
echo ""
echo "📊 3. Status after arming..."
./build/px4_sitl_default/bin/px4-commander minimal_commander status
sleep 1

# Test takeoff command
echo ""
echo "🚁 4. Enabling takeoff mode..."
./build/px4_sitl_default/bin/px4-commander minimal_commander takeoff
sleep 2

# Check status during flight
echo ""
echo "📊 5. Status during takeoff..."
./build/px4_sitl_default/bin/px4-commander minimal_commander status
sleep 2

# Disarm
echo ""
echo "🔒 6. Disarming..."
./build/px4_sitl_default/bin/px4-commander minimal_commander disarm
sleep 1

# Final status
echo ""
echo "📊 7. Final status..."
./build/px4_sitl_default/bin/px4-commander minimal_commander status

echo ""
echo "✅ Test complete!"
echo ""
echo "Stopping PX4..."
pkill -9 px4
