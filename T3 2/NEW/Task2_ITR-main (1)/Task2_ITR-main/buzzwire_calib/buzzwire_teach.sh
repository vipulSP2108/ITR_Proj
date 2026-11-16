#!/usr/bin/env bash
# =============================================================================
# buzzwire_teach.sh - one entry point for the whole Buzz Wire teach -> merge -> replay workflow
#
#   ./buzzwire_teach.sh teleop [--new|--session D]   METHOD 2: keyboard driving (robot in heal_server mode)
#   ./buzzwire_teach.sh record [--new|--session D]   METHOD 1: free-drive hand teaching (robot in ./free_drive)
#   ./buzzwire_teach.sh sim                          practise the keyboard teleop with a simulated arm (no robot)
#   ./buzzwire_teach.sh inspect [--session D] [--plot]   segment stats, gaps, wire coverage
#   ./buzzwire_teach.sh merge   [--session D] [--clearance 0.003] [--exclude N] [--reverse-seg N]
#   ./buzzwire_teach.sh replay  [--session D] [--speed 0.3] [--resume] [--dry-run]
#   ./buzzwire_teach.sh test                         run every no-hardware self-test
#   ./buzzwire_teach.sh clean   [SESSION_DIR]        delete merged output, keep raw segments
#
# Any extra flags are forwarded to the Python script (see each script's --help).
# Read README.md first; docs/ has the details.
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-python3}"
cd "$HERE"     # data/ and results/ are always relative to the project folder

source_ros() {
  # Source ROS 2 + the HEAL workspace if available. Override with:  WS=/path/to/addverb_ws ./buzzwire_teach.sh ...
  local ws="${WS:-$HOME/Debojit_WS/addverb_ws}"
  if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; fi
  if [[ -f "$ws/install/setup.bash" ]]; then source "$ws/install/setup.bash"; export WS="$ws"; fi
  if ! "$PY" -c "import rclpy" 2>/dev/null; then
    echo "[buzzwire] ERROR: rclpy not importable. Source your ROS 2 + workspace (set WS=...)." >&2; exit 1
  fi
}

session_from_args() {          # prints the value after --session if present
  local prev=""
  for a in "$@"; do
    if [[ "$prev" == "--session" ]]; then echo "$a"; return; fi
    prev="$a"
  done
}

CMD="${1:-help}"; [[ $# -gt 0 ]] && shift

case "$CMD" in
  teleop)  source_ros; exec "$PY" "$HERE/scripts/teleop.py" "$@" ;;
  record)  source_ros; exec "$PY" "$HERE/scripts/record_segment.py" "$@" ;;
  sim)     exec "$PY" "$HERE/scripts/teleop.py" --sim "$@" ;;
  inspect) exec "$PY" "$HERE/scripts/inspect_segments.py" "$@" ;;
  merge)
    "$PY" "$HERE/scripts/merge_segments.py" "$@"
    S="$(session_from_args "$@")"
    echo; if [[ -n "$S" ]]; then "$PY" "$HERE/scripts/inspect_segments.py" --session "$S"; else "$PY" "$HERE/scripts/inspect_segments.py"; fi
    echo; echo "[buzzwire] Next: ./buzzwire_teach.sh replay --dry-run   then   ./buzzwire_teach.sh replay --speed 0.2"
    ;;
  replay)
    ARGS=("$@")
    if [[ " ${ARGS[*]} " == *" --dry-run "* || " ${ARGS[*]} " == *" --sim "* ]]; then exec "$PY" "$HERE/scripts/replay_trajectory.py" "$@"; fi
    source_ros; exec "$PY" "$HERE/scripts/replay_trajectory.py" "$@" ;;
  test)
    for t in selftest_solver selftest_teleop selftest_pipeline; do echo "=== $t ==="; "$PY" "$HERE/scripts/$t.py" | tail -3; done
    ;;
  clean)
    S="${1:-$(ls -dt "$HERE"/data/session_* 2>/dev/null | head -1 || true)}"
    [[ -z "$S" ]] && { echo "no session found"; exit 1; }
    rm -f "$S/merged_trajectory.csv" "$S/merge_report.json" "$S/replay_state.json"; echo "cleaned $S (segments kept)" ;;
  help|-h|--help|*) sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
