#!/bin/bash

# Script to launch multiple drones in PX4 Gazebo simulation
# Usage: ./launch_multi_drone.sh [number_of_drones]

NUM_DRONES=${1:-2}  # Default to 2 drones if not specified
DRONE_SPACING=2.0   # Spacing between drones in meters
PX4_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "================================================"
echo " Launching $NUM_DRONES drones in Gazebo"
echo "================================================"

# Kill any existing PX4 and Gazebo instances
echo "Cleaning up existing instances..."
pkill -9 px4 || true
pkill -9 -f 'gz sim' || true
rm -rf /tmp/px4* || true
sleep 2

# Build PX4 first
echo "Building PX4..."
cd "$PX4_DIR"
make px4_sitl_default > /dev/null 2>&1

# Start first drone (instance 0) - this will launch Gazebo world
echo ""
echo "Starting Drone 0 (MAVLink port 14540)..."
cd "$PX4_DIR"
PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0" make px4_sitl gz_x500 &
DRONE0_PID=$!
sleep 20  # Wait for Gazebo to fully start

# Start additional drones
for i in $(seq 1 $((NUM_DRONES-1))); do
    INSTANCE=$i
    MAVLINK_PORT=$((14540 + INSTANCE))
    POSITION_Y=$(echo "$i * $DRONE_SPACING" | bc)

    echo ""
    echo "Starting Drone $INSTANCE (MAVLink port $MAVLINK_PORT)..."
    echo "Position: (0, $POSITION_Y, 0.5)"

    # Launch additional PX4 instance - let it connect to existing Gazebo
    cd "$PX4_DIR"

    # Set environment for this instance
    export PX4_SIM_MODEL=gz_x500
    export PX4_GZ_MODEL_POSE="0,$POSITION_Y,0.5,0,0,0"
    export PX4_GZ_STANDALONE=1  # Connect to existing Gazebo

    # Run PX4 binary directly with instance number
    build/px4_sitl_default/bin/px4 \
        -i $INSTANCE \
        -d ROMFS/px4fmu_common \
        -s etc/init.d-posix/rcS \
        -t test_data &

    sleep 8  # Wait between spawning each drone
done

echo ""
echo "================================================"
echo " All $NUM_DRONES drones launched successfully!"
echo "================================================"
echo ""
echo "Drone MAVLink connection ports:"
for i in $(seq 0 $((NUM_DRONES-1))); do
    PORT=$((14540 + i))
    echo "  Drone $i: udp://127.0.0.1:$PORT"
done
echo ""
echo "Press Ctrl+C to stop all drones"
echo ""

# Wait for all background processes
wait
