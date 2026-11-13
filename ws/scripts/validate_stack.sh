#!/usr/bin/env bash
# Run from a sourced ROS environment. Package fixtures own separate ROS contexts.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export ROS_DOMAIN_ID="${TEST_ROS_DOMAIN_ID:-42}"
python3 -m pytest src/autodrive_bridge/test -q
python3 -m pytest src/control/test -q
python3 -m pytest src/perception/test -q
python3 -m pytest scripts/test_tire_lap_report.py scripts/test_wheel_slip_comparison.py \
  scripts/test_compare_map_experiment.py scripts/test_characterize_wheel_slip.py \
  scripts/test_analyze_straight_trace.py -q
