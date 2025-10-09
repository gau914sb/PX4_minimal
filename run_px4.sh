#!/bin/bash
# Helper script to run PX4 with the correct environment
# Usage: ./run_px4.sh

cd "$(dirname "$0")"

# Kill any existing PX4 processes
pkill -9 px4 2>/dev/null || true
sleep 1

# Set the simulation model to load the airframe parameters
export PX4_SIM_MODEL=none_iris

# Use SIH (Software-In-Hardware) simulator instead of waiting for Gazebo
export PX4_SIMULATOR=sihsim

# Run PX4
./build/px4_sitl_default/bin/px4
