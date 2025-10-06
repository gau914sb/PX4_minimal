#!/bin/bash

echo "==============================================="
echo "PX4 ATTITUDE-ONLY CONFIGURATION TEST"
echo "==============================================="
echo ""
echo "This script tests if position control is disabled"
echo "Run this in a SEPARATE terminal while PX4 simulation is running"
echo ""

# Change to PX4 directory
cd /Users/gauravsinghbhati/Documents/PX4-Autopilot

echo "Waiting 3 seconds to ensure PX4 is ready..."
sleep 3

echo ""
echo "=== TEST 1: List Running Processes ==="
echo "Looking for position control modules (should NOT be present):"
echo "- mc_pos_control"
echo "- flight_mode_manager"
echo "- mc_hover_thrust_estimator"
echo ""
echo "ps" | timeout 5 ./build/px4_sitl_default/bin/px4-commander 2>/dev/null || echo "Command failed or timed out"

echo ""
echo "=== TEST 2: Check Commander Status ==="
echo "commander status" | timeout 5 ./build/px4_sitl_default/bin/px4-commander 2>/dev/null || echo "Command failed or timed out"

echo ""
echo "=== TEST 3: Attempt to Arm Vehicle ==="
echo "commander arm" | timeout 5 ./build/px4_sitl_default/bin/px4-commander 2>/dev/null || echo "Arm command failed or timed out"

echo ""
echo "=== TEST 4: Test Manual Mode (Should Work) ==="
echo "commander mode manual" | timeout 5 ./build/px4_sitl_default/bin/px4-commander 2>/dev/null || echo "Manual mode failed"

echo ""
echo "=== TEST 5: Test Position Control Mode (Should FAIL) ==="
echo "This should fail because mc_pos_control is disabled:"
echo "commander mode posctl" | timeout 5 ./build/px4_sitl_default/bin/px4-commander 2>/dev/null || echo "Position control mode REJECTED (as expected)"

echo ""
echo "=== TEST 6: Test Auto Takeoff (Should FAIL) ==="
echo "This should fail because flight_mode_manager is disabled:"
echo "commander takeoff" | timeout 5 ./build/px4_sitl_default/bin/px4-commander 2>/dev/null || echo "Takeoff command REJECTED (as expected)"

echo ""
echo "=== TEST 7: Test Goto Command (Should FAIL) ==="
echo "This should fail because position control is disabled:"
echo "commander goto 5 5 5" | timeout 5 ./build/px4_sitl_default/bin/px4-commander 2>/dev/null || echo "Goto command REJECTED (as expected)"

echo ""
echo "=== TEST 8: Test Land Command (Should FAIL) ==="
echo "This should fail because position control is disabled:"
echo "commander mode land" | timeout 5 ./build/px4_sitl_default/bin/px4-commander 2>/dev/null || echo "Land mode REJECTED (as expected)"

echo ""
echo "=== TEST 9: Check Available Flight Modes ==="
echo "List available modes (position-dependent modes should be missing):"
echo "commander mode" | timeout 5 ./build/px4_sitl_default/bin/px4-commander 2>/dev/null || echo "Mode list command failed"

echo ""
echo "==============================================="
echo "TEST RESULTS INTERPRETATION:"
echo "==============================================="
echo ""
echo "✅ SUCCESS INDICATORS (Attitude-Only Mode Working):"
echo "  - ps command shows NO mc_pos_control module"
echo "  - ps command shows NO flight_mode_manager module"
echo "  - ps command shows NO mc_hover_thrust_estimator module"
echo "  - Manual mode works"
echo "  - Position control mode FAILS/REJECTED"
echo "  - Takeoff command FAILS/REJECTED"
echo "  - Goto command FAILS/REJECTED"
echo "  - Land mode FAILS/REJECTED"
echo ""
echo "❌ FAILURE INDICATORS (Position Control Still Active):"
echo "  - Any of the above modules appear in ps output"
echo "  - Position control commands succeed"
echo "  - Auto modes work normally"
echo ""
echo "==============================================="
echo "TEST COMPLETE"
echo "==============================================="
