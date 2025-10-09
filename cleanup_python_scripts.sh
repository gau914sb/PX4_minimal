#!/bin/bash

# Cleanup script to remove temporary/debug Python scripts
# Keeps only the 3 final testing scripts

set -e

cd /Users/gauravsinghbhati/Documents/PX4_minimal

echo "======================================"
echo "Python Scripts Cleanup"
echo "======================================"
echo ""
echo "This will DELETE 32 Python files from the root directory."
echo "KEEPING only these 3 files:"
echo "  - test_attitude_control.py"
echo "  - test_rate_control.py"
echo "  - test_rate_control_visual.py"
echo ""
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    echo "Aborted."
    exit 1
fi

echo ""
echo "Phase 1: Deleting debug/diagnostic scripts (15 files)..."
rm -fv check_allocator_running.py
rm -fv check_ca_parameters.py
rm -fv check_control_mode.py
rm -fv check_controllers_live.py
rm -fv check_mavlink_attitude.py
rm -fv check_motor_outputs.py
rm -fv check_offboard_status.py
rm -fv control_diagnostic.py
rm -fv debug_control_chain.py
rm -fv px4_diagnostic.py
rm -fv px4_status_check.py
rm -fv message_type_test.py
rm -fv offboard_test.py
rm -fv param_override.py
rm -fv test_minimal_commander.py

echo ""
echo "Phase 2: Deleting early/experimental test scripts (12 files)..."
rm -fv attitude_commander.py
rm -fv attitude_commander_slow.py
rm -fv attitude_only.py
rm -fv console_motor_control.py
rm -fv console_takeoff.py
rm -fv direct_motor_control.py
rm -fv ground_motor_test.py
rm -fv rate_commander.py
rm -fv send_attitude_commands.py
rm -fv visual_motor_test.py
rm -fv takeoff_hover.py
rm -fv takeoff_hover_pymavlink.py

echo ""
echo "Phase 3: Deleting manual control experiments (5 files)..."
rm -fv manual_control_simple.py
rm -fv manual_control_test.py
rm -fv manual_takeoff.py
rm -fv rc_attitude_control.py
rm -fv stick_commander.py

echo ""
echo "======================================"
echo "✅ Cleanup Complete!"
echo "======================================"
echo ""
echo "Remaining Python scripts in root:"
ls -lh test_*.py

echo ""
echo "Total: 3 final testing scripts kept"
echo "  ✓ test_attitude_control.py"
echo "  ✓ test_rate_control.py"
echo "  ✓ test_rate_control_visual.py"
