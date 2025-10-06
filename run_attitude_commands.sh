#!/bin/bash

echo "==============================================="
echo "PX4 ATTITUDE COMMAND SETUP"
echo "==============================================="
echo ""
echo "This script sets up and runs attitude commands for your flying drone"
echo ""

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found. Please install Python 3."
    exit 1
fi

echo "✅ Python 3 found"

# Check if pymavlink is installed
if python3 -c "import pymavlink" 2>/dev/null; then
    echo "✅ pymavlink already installed"
else
    echo "📦 Installing pymavlink..."
    pip3 install pymavlink
fi

echo ""
echo "==============================================="
echo "SAFETY CHECKLIST BEFORE RUNNING:"
echo "==============================================="
echo ""
echo "⚠️  CRITICAL SAFETY REQUIREMENTS:"
echo "   ✅ Drone must be AIRBORNE and stable"
echo "   ✅ Drone must be in Manual or Stabilized mode"
echo "   ✅ Be ready to take manual control immediately"
echo "   ✅ Have emergency stop/disarm ready"
echo "   ✅ Clear flight area with no obstacles"
echo ""
echo "📡 MAVLINK CONNECTION:"
echo "   - Connecting to: udp:127.0.0.1:14540"
echo "   - This connects to PX4 SITL onboard MAVLink port"
echo ""
echo "🎮 ATTITUDE COMMANDS TO EXECUTE:"
echo "   1. Hover (neutral attitude)"
echo "   2. Roll left/right oscillations"
echo "   3. Pitch forward/back oscillations"
echo "   4. Yaw rotation oscillations"
echo "   5. Combined attitude movements"
echo "   6. Return to hover"
echo ""
echo "⏹️  To stop: Press Ctrl+C (will send neutral attitude)"
echo ""

read -p "Are you ready to start attitude commands? (y/N): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "🚁 Starting attitude commander..."
    echo "Press Ctrl+C anytime to stop and return to neutral attitude"
    echo ""
    sleep 2

    cd /Users/gauravsinghbhati/Documents/PX4-Autopilot
    python3 attitude_commander.py
else
    echo "❌ Cancelled. Run this script when you're ready."
fi

echo ""
echo "==============================================="
echo "MANUAL CONTROL TIPS AFTER ATTITUDE COMMANDS:"
echo "==============================================="
echo ""
echo "- Switch back to manual control in QGroundControl"
echo "- Test that manual stick inputs still work"
echo "- Land the drone safely"
echo "- Remember: No position hold - manual control only!"
echo ""
