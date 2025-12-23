#!/bin/bash

# Official PX4 Multi-Vehicle Simulation Script
# Based on: https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation

NUM_DRONES=${1:-3}  # Default to 3 drones

echo "================================================"
echo " PX4 Official Multi-Vehicle Simulation"
echo " Launching $NUM_DRONES drones"
echo "================================================"

# Kill existing instances
echo "Cleaning up..."
pkill -9 px4 || true
pkill -9 -f 'gz sim' || true
rm -rf /tmp/px4* || true
sleep 2

# Build PX4
echo "Building PX4..."
make px4_sitl

echo ""
echo "================================================"
echo " Starting drones..."
echo "================================================"
echo ""
echo "Starting each drone in the background..."
echo "Wait for all drones to spawn in Gazebo"
echo ""

# Launch vehicles (instances 0, 1, 2, ...)
for i in $(seq 0 $((NUM_DRONES-1))); do
    INSTANCE=$i
    Y_POS=$(echo "$i * 2.0" | bc)
    
    if [ $i -eq 0 ]; then
        echo "Drone $i (instance $INSTANCE): No standalone mode - will start Gazebo"
        PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
            ./build/px4_sitl_default/bin/px4 -i $INSTANCE > /tmp/drone_${INSTANCE}.log 2>&1 &
        sleep 20  # Wait for Gazebo to start
    else
        echo "Drone $i (instance $INSTANCE): Standalone mode at position (0, $Y_POS)"
        PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 \
            PX4_GZ_MODEL_POSE="0,$Y_POS" PX4_SIM_MODEL=gz_x500 \
            ./build/px4_sitl_default/bin/px4 -i $INSTANCE > /tmp/drone_${INSTANCE}.log 2>&1 &
        sleep 8
    fi
done

echo ""
echo "================================================"
echo " All $NUM_DRONES drones launched!"
echo "================================================"
echo ""
echo "MAVLink connection info:"
for i in $(seq 0 $((NUM_DRONES-1))); do
    PORT=$((14540 + i))
    SYS_ID=$((i + 1))
    echo "  Drone $i (System ID $SYS_ID): udp://127.0.0.1:$PORT"
done
echo ""
echo "Logs are in /tmp/drone_*.log"
echo "Press Ctrl+C to stop"
echo ""

# Wait for user interrupt
wait
