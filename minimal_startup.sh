#!/bin/bash
# Minimal startup script for PX4 with minimal_commander
# This keeps PX4 running without exiting

cd /Users/gauravsinghbhati/Documents/PX4-Autopilot

# Start PX4 with a simple command that keeps it running
./build/px4_sitl_default/bin/px4 <<EOF
simulator_sih start
minimal_commander start
manual_control start
land_detector start multicopter
battery_simulator start
logger start -t
mavlink start -x -u 14556 -r 4000000
mavlink start -x -u 14557 -r 4000000 -m onboard -o 14540
mavlink boot_complete
sleep 1000000
EOF
