#!/bin/bash

# Test script to validate attitude-only configuration
echo "=== PX4 Attitude-Only Configuration Test ==="

# Kill any existing processes
pkill -9 px4 2>/dev/null

# Wait a moment
sleep 2

# Start PX4 in daemon mode
echo "Starting PX4 daemon..."
cd /Users/gauravsinghbhati/Documents/PX4-Autopilot
./build/px4_sitl_default/bin/px4 -d > /tmp/px4_startup.log 2>&1 &
PX4_PID=$!

# Wait for startup
sleep 5

# Test if specific modules are NOT running (they should be commented out in rc.mc_apps)
echo "Checking if position control modules are disabled..."

# Try to connect and list processes
timeout 5 ./build/px4_sitl_default/bin/px4-commander status > /tmp/px4_status.log 2>&1

# Check the logs
echo "=== PX4 Startup Log ==="
cat /tmp/px4_startup.log

echo "=== Module Status ==="
cat /tmp/px4_status.log

# Clean up
kill $PX4_PID 2>/dev/null
pkill -9 px4 2>/dev/null

echo "=== Test Complete ==="
echo "Check the logs above to verify that mc_pos_control, flight_mode_manager, and mc_hover_thrust_estimator are NOT started"
