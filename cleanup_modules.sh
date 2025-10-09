#!/bin/bash
# Script to remove unused modules from src/modules/
# Keeps only modules needed for minimal multicopter build

echo "======================================================"
echo "Removing unused modules from src/modules/"
echo "======================================================"

cd src/modules

echo ""
echo "Modules that will be KEPT:"
echo "  ✓ minimal_commander      (Your custom commander)"
echo "  ✓ mc_rate_control        (Rate controller)"
echo "  ✓ control_allocator      (Motor mixer)"
echo "  ✓ ekf2                   (State estimator)"
echo "  ✓ sensors                (Sensor processing)"
echo "  ✓ mavlink                (Communication)"
echo "  ✓ logger                 (Logging)"
echo "  ✓ land_detector          (Landing detection)"
echo "  ✓ navigator              (Required by land_detector)"
echo "  ✓ dataman                (Data storage)"
echo "  ✓ events                 (Event system)"
echo "  ✓ battery_status         (Battery monitoring)"
echo "  ✓ simulation             (SIH simulator)"
echo "  ✓ load_mon               (System monitoring)"
echo "  ✓ rc_update              (RC input processing)"
echo "  ✓ commander              (Keep as reference)"
echo ""

read -p "Press Enter to start deletion or Ctrl+C to cancel..."

echo ""
echo "[1/4] Deleting other vehicle type modules..."
rm -rfv airship_att_control/
rm -rfv fw_att_control/
rm -rfv fw_autotune_attitude_control/
rm -rfv fw_lateral_longitudinal_control/
rm -rfv fw_mode_manager/
rm -rfv fw_rate_control/
rm -rfv rover_ackermann/
rm -rfv rover_differential/
rm -rfv rover_mecanum/
rm -rfv spacecraft/
rm -rfv internal_combustion_engine_control/
echo "✓ Other vehicle modules removed"

echo ""
echo "[2/4] Deleting unused multicopter modules..."
rm -rfv mc_att_control/                    # Already disabled
rm -rfv mc_autotune_attitude_control/
rm -rfv mc_hover_thrust_estimator/
rm -rfv mc_nn_control/
rm -rfv mc_pos_control/                    # Position control not needed for rate-only
echo "✓ Unused multicopter modules removed"

echo ""
echo "[3/4] Deleting advanced/optional features..."
rm -rfv airspeed_selector/
rm -rfv attitude_estimator_q/              # Old estimator
rm -rfv camera_feedback/
rm -rfv esc_battery/
rm -rfv flight_mode_manager/               # Not needed for offboard-only
rm -rfv gimbal/
rm -rfv gyro_calibration/
rm -rfv gyro_fft/
rm -rfv hardfault_stream/
rm -rfv landing_target_estimator/
rm -rfv local_position_estimator/          # Old estimator
rm -rfv mag_bias_estimator/
rm -rfv manual_control/                    # Not needed for offboard-only
rm -rfv muorb/                            # Micro-ROS bridge
rm -rfv payload_deliverer/
rm -rfv px4iofirmware/                    # PX4IO firmware (separate)
rm -rfv replay/
rm -rfv temperature_compensation/
echo "✓ Advanced features removed"

echo ""
echo "[4/4] Checking remaining modules..."
echo ""
echo "Remaining modules in src/modules/:"
ls -1
echo ""

echo "======================================================"
echo "Module cleanup complete!"
echo "======================================================"
echo ""
echo "Modules deleted: ~40"
echo "Modules kept: ~17"
echo ""
echo "Next steps:"
echo "1. Test build: make px4_sitl_default"
echo "2. If successful: git add -A && git commit -m 'Remove unused modules'"
echo "======================================================"
