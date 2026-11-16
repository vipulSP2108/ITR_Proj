#!/usr/bin/env bash
# Convenience launcher: sources the ROS workspace, then starts the collector.
# Robot side first (README): ./cobot run -> 1 -> ./free_drive      (in the ssh terminal)
set -e
WS="${WS:-$HOME/Debojit_WS/addverb_ws}"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
cd "$(dirname "$0")"
exec python3 scripts/collect_data.py "$@"
