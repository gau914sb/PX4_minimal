#!/bin/bash

echo "=== Testing PX4 Attitude-Only Configuration ==="
echo "Testing position control commands to verify they are disabled..."

cd /Users/gauravsinghbhati/Documents/PX4-Autopilot

# Wait for PX4 to be ready
sleep 5

echo "=== 1. Checking running processes ==="
echo "ps" | ./build/px4_sitl_default/bin/px4-commander || echo "Failed to connect"

echo ""
echo "=== 2. Checking commander status ==="
echo "commander status" | ./build/px4_sitl_default/bin/px4-commander || echo "Failed to get status"

echo ""
echo "=== 3. Attempting to arm ==="
echo "commander arm" | ./build/px4_sitl_default/bin/px4-commander || echo "Failed to arm"

echo ""
echo "=== 4. Testing takeoff command (should fail without position control) ==="
echo "commander takeoff" | ./build/px4_sitl_default/bin/px4-commander || echo "Takeoff command failed/rejected"

echo ""
echo "=== 5. Testing goto command (should fail without position control) ==="
echo "commander goto 10 10 10" | ./build/px4_sitl_default/bin/px4-commander || echo "Goto command failed/rejected"

echo ""
echo "=== 6. Testing position hold command (should fail) ==="
echo "commander mode posctl" | ./build/px4_sitl_default/bin/px4-commander || echo "Position control mode failed/rejected"

echo ""
echo "=== 7. Testing manual mode (should work) ==="
echo "commander mode manual" | ./build/px4_sitl_default/bin/px4-commander || echo "Manual mode failed"

echo ""
echo "=== Test Complete ==="
echo "If position control is truly disabled:"
echo "- Takeoff/goto/posctl commands should fail or be rejected"
echo "- Manual mode should work"
echo "- No mc_pos_control, flight_mode_manager, or mc_hover_thrust_estimator should be running"
