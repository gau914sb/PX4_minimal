#!/bin/bash
# Quick debug script to check topic data

echo "=== Checking vehicle_status ==="
echo "listener vehicle_status -n 1" | nc localhost 4560 | grep -E "nav_state|arming_state" | head -3

echo ""
echo "=== Checking vehicle_control_mode ==="
echo "listener vehicle_control_mode -n 1" | nc localhost 4560 | grep -E "flag_control" | head -8

echo ""
echo "=== Checking vehicle_attitude_setpoint ==="
echo "listener vehicle_attitude_setpoint -n 1" | nc localhost 4560 | grep -E "q_d|thrust" | head -5

echo ""
echo "=== Checking offboard_control_mode ==="
echo "listener offboard_control_mode -n 1" | nc localhost 4560 | grep -E "attitude|body_rate|timestamp" | head -5
