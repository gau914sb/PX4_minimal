#!/bin/bash
# Script to remove unnecessary markdown documentation files from PX4_minimal
# Run this from the PX4_minimal root directory

echo "======================================================"
echo "Cleaning up unnecessary documentation files"
echo "======================================================"

# Root directory - Delete debug/analysis files
echo ""
echo "[1/3] Removing root directory analysis files..."
rm -fv PARAMETER_DEPENDENCY_ANALYSIS.md
rm -fv ATTITUDE_CONTROL_FIX.md
rm -fv px4_interdependencies.md
rm -fv BUILD_SUCCESS_SUMMARY.md
rm -fv ArduPilot_vs_PX4_Communication_Analysis.md
rm -fv FIXES_APPLIED_SUMMARY.md
rm -fv MISSING_FEATURES_ANALYSIS.md
rm -fv px4_function_modifications.md
rm -fv Work_Item_Scheduling_Flow.md
rm -fv PX4_Control_Architecture_Analysis.md
rm -fv MC_CONTROLLER_DEPENDENCIES.md
rm -fv Minimal_Commander_Complete_Roadmap.md
rm -fv TEST_SCRIPT_UPDATES.md
rm -fv COMMANDER_STUDY_SUMMARY.md
rm -fv SUCCESS_SUMMARY.md
rm -fv QUICK_START.md

echo "✓ Root cleanup complete"

# minimal_commander module - Delete debug/analysis files
echo ""
echo "[2/3] Removing minimal_commander debug files..."
rm -fv src/modules/minimal_commander/FILES_NEEDED_ANALYSIS.md
rm -fv src/modules/minimal_commander/COMPILATION_REPORT.md
rm -fv src/modules/minimal_commander/STANDALONE_SETUP_COMPLETE.md
rm -fv src/modules/minimal_commander/MINIMAL_BUILD_CONFIGURATION.md
rm -fv src/modules/minimal_commander/CRITICAL_FIXES_APPLY_NOW.md
rm -fv src/modules/minimal_commander/IMPLEMENTATION_SUMMARY.md
rm -fv src/modules/minimal_commander/BUILD_SUCCESS_REPORT.md
rm -fv src/modules/minimal_commander/GAPS_AND_LOOPHOLES_ANALYSIS.md
rm -fv src/modules/minimal_commander/CONSOLE_COMMANDS.md
rm -fv src/modules/minimal_commander/COMMANDER_DEPENDENCY_ANALYSIS.md

echo "✓ minimal_commander cleanup complete"

# docs directory - Remove entire documentation folder
echo ""
echo "[3/3] Removing docs/ directory (15,000+ PX4 doc files)..."
if [ -d "docs" ]; then
    echo "  Calculating size..."
    du -sh docs/
    rm -rf docs/
    echo "✓ docs/ directory removed"
else
    echo "  docs/ directory not found (already removed)"
fi

# PX4 analysis directory - Remove if exists
echo ""
echo "[4/4] Removing PX4/ analysis directory..."
if [ -d "PX4" ]; then
    echo "  Calculating size..."
    du -sh PX4/
    rm -rf PX4/
    echo "✓ PX4/ directory removed"
else
    echo "  PX4/ directory not found (already removed)"
fi

echo ""
echo "======================================================"
echo "Cleanup Summary"
echo "======================================================"
echo ""
echo "✓ Removed ~26 markdown analysis/debug files"
echo "✓ Removed docs/ directory (PX4 documentation)"
echo "✓ Removed PX4/ directory (analysis notes)"
echo ""
echo "Kept files:"
echo "  - README.md (main documentation)"
echo "  - CODE_OF_CONDUCT.md"
echo "  - CONTRIBUTING.md"
echo "  - MAINTAINERS.md"
echo "  - SECURITY.md"
echo "  - src/modules/minimal_commander/README.md"
echo ""
echo "Space saved: Run 'du -sh .' to check new size"
echo "======================================================"
