#!/usr/bin/env bash
# Convenience launcher: sources the ROS workspace, then starts the collector.
# Robot side first (README): ./cobot run -> 1 -> ./free_drive      (in the ssh terminal)
set -e

# Auto-detect ROS 2 distribution if not set
if [ -z "$ROS_DISTRO" ]; then
    for distro in humble iron foxy galactic rolling jazzy; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            source "/opt/ros/$distro/setup.bash"
            echo "[INFO] Sourced /opt/ros/$distro/setup.bash"
            break
        fi
    done
else
    [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] && source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Auto-detect HEAL / Addverb workspace
WS_CANDIDATES=(
    "${WS:-}"
    "$HOME/Debojit_WS/addverb_ws"
    "$HOME/addverb_ws"
    "/home/cobot/Debojit_WS/addverb_ws"
)

for ws in "${WS_CANDIDATES[@]}"; do
    if [ -n "$ws" ] && [ -f "$ws/install/setup.bash" ]; then
        source "$ws/install/setup.bash"
        echo "[INFO] Sourced $ws/install/setup.bash"
        break
    fi
done

cd "$(dirname "$0")"
exec python3 scripts/collect_data.py "$@"
