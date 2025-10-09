#!/bin/bash
# Verify the entire attitude control chain

echo "=== Checking PX4 Control Chain ==="
echo ""

# Start PX4 in background
cd /Users/gauravsinghbhati/Documents/PX4_minimal
./build/px4_sitl_default/bin/px4 -d > /tmp/px4_output.log 2>&1 &
PX4_PID=$!

# Wait for PX4 to start
sleep 5

echo "1. Checking if mc_att_control is running..."
echo "mc_att_control status" | ./build/px4_sitl_default/bin/px4-send_command > /tmp/check1.txt 2>&1
sleep 1

echo "2. Checking if mc_rate_control is running..."
echo "mc_rate_control status" | ./build/px4_sitl_default/bin/px4-send_command > /tmp/check2.txt 2>&1
sleep 1

echo ""
echo "3. Starting attitude control test in background..."
python3 test_attitude_control.py > /tmp/test_output.txt 2>&1 &
TEST_PID=$!

# Wait for test to arm and enter offboard
sleep 5

echo "4. Listening to vehicle_attitude_setpoint (should show incoming setpoints)..."
timeout 5 ./build/px4_sitl_default/bin/px4-listener vehicle_attitude_setpoint 2>&1 | head -20

echo ""
echo "5. Listening to vehicle_rates_setpoint (output from mc_att_control)..."
timeout 5 ./build/px4_sitl_default/bin/px4-listener vehicle_rates_setpoint 2>&1 | head -20

echo ""
echo "6. Listening to actuator_motors (final motor commands)..."
timeout 5 ./build/px4_sitl_default/bin/px4-listener actuator_motors 2>&1 | head -20

# Cleanup
kill $TEST_PID 2>/dev/null
kill $PX4_PID 2>/dev/null

echo ""
echo "=== Check complete ==="
echo "If you see data in steps 4-6, the control chain is working!"
