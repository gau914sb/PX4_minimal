#!/bin/bash

echo "==============================================="
echo "QGROUNDCONTROL CONNECTION HELPER"
echo "==============================================="
echo ""
echo "PX4 SITL with jMAVSim should be running in another terminal"
echo ""
echo "QGroundControl Connection Settings:"
echo "  Protocol: TCP"
echo "  Host: localhost (127.0.0.1)"
echo "  Port: 4560"
echo ""
echo "Expected behavior with attitude-only configuration:"
echo ""
echo "✅ SHOULD WORK:"
echo "  - Vehicle connects to QGroundControl"
echo "  - Manual flight mode available"
echo "  - Attitude control (roll, pitch, yaw) works"
echo "  - Arming works"
echo "  - Manual throttle control works"
echo ""
echo "❌ SHOULD NOT WORK / BE MISSING:"
echo "  - Position Hold mode"
echo "  - Auto Takeoff"
echo "  - Auto Land"
echo "  - Mission mode"
echo "  - Return to Land (RTL)"
echo "  - Goto waypoint commands"
echo "  - Position control in any form"
echo ""
echo "==============================================="
echo "FLIGHT TEST PROCEDURE:"
echo "==============================================="
echo ""
echo "1. Connect QGroundControl to the simulation"
echo "2. ARM the vehicle in Manual mode"
echo "3. Try increasing throttle - vehicle should lift off"
echo "4. Test roll/pitch/yaw inputs - should respond immediately"
echo "5. Try switching to Position Hold - should FAIL"
echo "6. Try Auto modes - should be UNAVAILABLE or FAIL"
echo ""
echo "If vehicle maintains position automatically = FAIL"
echo "If vehicle only responds to manual inputs = SUCCESS"
echo ""
echo "==============================================="

# Check if PX4 is running
if pgrep -f "px4" > /dev/null; then
    echo "✅ PX4 process detected running"
else
    echo "❌ PX4 process not detected - start simulation first"
fi

# Check if jMAVSim is running
if pgrep -f "jmavsim" > /dev/null; then
    echo "✅ jMAVSim process detected running"
else
    echo "❌ jMAVSim process not detected - start simulation first"
fi

# Check if port 4560 is in use
if lsof -i :4560 > /dev/null 2>&1; then
    echo "✅ Port 4560 is active (MAVLink connection ready)"
else
    echo "❌ Port 4560 not active - simulation may not be ready"
fi

echo ""
echo "Now run QGroundControl and connect to localhost:4560"
echo "Then run the attitude-only test script:"
echo "./test_attitude_only_manual.sh"
echo ""
