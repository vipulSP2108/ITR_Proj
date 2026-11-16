#!/usr/bin/env bash
# =============================================================================
# buzzwire_teach.sh - one entry point for the whole Buzz Wire teach -> build -> replay workflow
#
#   ./buzzwire_teach.sh new [NAME]                   create a FRESH session folder data/session_<date>_<time>
#                                                    (or data/NAME) and make it the ACTIVE session
#   ./buzzwire_teach.sh status                       show the active session and what files it has
#   ./buzzwire_teach.sh use SESSION_DIR              make an existing session the active one
#   ./buzzwire_teach.sh teleop                       METHOD 2: keyboard driving (robot in heal_server mode)
#   ./buzzwire_teach.sh record                       METHOD 1: free-drive hand teaching (robot in ./free_drive)
#   ./buzzwire_teach.sh sim                          practise the keyboard teleop with a simulated arm (no robot)
#   ./buzzwire_teach.sh inspect [--plot]             segment stats, gaps, wire coverage
#   ./buzzwire_teach.sh merge   [--clearance 0.003] [--exclude N] [--reverse-seg N]
#   ./buzzwire_teach.sh build   [merge flags]        merge + calibrate -> LR, RL, LR_smooth, RL_smooth CSVs
#   ./buzzwire_teach.sh replay  [--speed 0.3] [--resume] [--dry-run]
#   ./buzzwire_teach.sh test                         run every no-hardware self-test
#   ./buzzwire_teach.sh clean   [SESSION_DIR]        delete merged/calibrated output, keep raw segments
#
# Every command works on the ACTIVE session (data/ACTIVE_SESSION) unless you pass --session D.
# Any extra flags are forwarded to the Python script (see each script's --help).
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-python3}"
cd "$HERE"     # data/ and results/ are always relative to the project folder
ACTIVE_FILE="$HERE/data/ACTIVE_SESSION"

source_ros() {
  # Source ROS 2 + the HEAL workspace if available. Override with:  WS=/path/to/addverb_ws ./buzzwire_teach.sh ...
  local ws="${WS:-$HOME/Debojit_WS/addverb_ws}"
  set +u
  if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; fi
  if [[ -f "$ws/install/setup.bash" ]]; then source "$ws/install/setup.bash"; export WS="$ws"; fi
  set -u
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

active_session() {             # prints the active session dir (absolute) or nothing
  if [[ -f "$ACTIVE_FILE" ]]; then
    local s; s="$(head -1 "$ACTIVE_FILE")"
    [[ "$s" != /* ]] && s="$HERE/$s"
    [[ -d "$s" ]] && echo "$s"
  fi
}

set_active() {
  mkdir -p "$HERE/data"
  echo "$1" > "$ACTIVE_FILE"
  echo "[buzzwire] Active session -> $1"
}

# SESSION = explicit --session, else the active session. SARGS = "--session X" to inject when not given.
CMD="${1:-help}"; [[ $# -gt 0 ]] && shift
SESSION="$(session_from_args "$@")"
SARGS=()
if [[ -z "$SESSION" ]]; then
  SESSION="$(active_session || true)"
  [[ -n "$SESSION" ]] && SARGS=(--session "$SESSION")
fi

need_session() {
  if [[ -z "$SESSION" ]]; then
    echo "[buzzwire] No active session. Create one first:  ./buzzwire_teach.sh new" >&2; exit 1
  fi
}

case "$CMD" in
  new)
    NAME="${1:-session_$(date +%Y%m%d_%H%M%S)}"
    D="$HERE/data/$NAME"
    if [[ -e "$D" ]]; then echo "[buzzwire] $D already exists - pick another name." >&2; exit 1; fi
    mkdir -p "$D/segments"
    set_active "$D"
    cat <<MSG

Fresh session created: $D
Old sessions in data/ are untouched.

Next (robot side: ./heal_server + velocity control for teleop, or ./free_drive for record):
  ./buzzwire_teach.sh teleop        # drive along the wire, one segment per run (repeat as needed)
  ./buzzwire_teach.sh inspect       # check segments / gaps
  ./buzzwire_teach.sh build         # merge + calibrate -> LR / RL / LR_smooth / RL_smooth
MSG
    ;;
  use)
    D="${1:?usage: ./buzzwire_teach.sh use SESSION_DIR}"
    [[ "$D" != /* ]] && { [[ -d "$D" ]] && D="$(cd "$D" && pwd)" || D="$HERE/data/$D"; }
    [[ -d "$D" ]] || { echo "[buzzwire] $D not found" >&2; exit 1; }
    set_active "$D"
    ;;
  status)
    need_session
    echo "Active session: $SESSION"
    echo "Segments      : $(ls "$SESSION"/segments/seg_????.csv 2>/dev/null | wc -l)"
    for f in merged_trajectory.csv calibrated_wire_LR.csv calibrated_wire_RL.csv calibrated_wire_LR_smooth.csv calibrated_wire_RL_smooth.csv; do
      if [[ -f "$SESSION/$f" ]]; then printf "  [x] %s (%s pts)\n" "$f" "$(( $(wc -l < "$SESSION/$f") - 1 ))"; else printf "  [ ] %s\n" "$f"; fi
    done
    "$PY" - "$SESSION" <<'PYEOF' 2>/dev/null || true
import csv, math, sys, glob, os
sd = sys.argv[1]
def pts(f): return [(float(d['x']), float(d['y']), float(d['z'])) for d in csv.DictReader(open(f))]
def mm(p): return "[" + ", ".join(f"{v*1000:6.0f}" for v in p) + "]"
print("\nSegments (start -> end, mm)  - false starts / repeats should be left out with --only or --exclude:")
for f in sorted(glob.glob(os.path.join(sd, "segments", "seg_????.csv"))):
    r = pts(f)
    if r: print(f"  {os.path.basename(f)[4:8]}: {len(r):5d} pts  {mm(r[0])} -> {mm(r[-1])}  arc {1000*sum(math.dist(a,b) for a,b in zip(r,r[1:])):6.0f} mm")
f = os.path.join(sd, "calibrated_wire_LR.csv")
if os.path.exists(f):
    r = pts(f)
    print(f"\nLR path: L = {mm(r[0])}   R = {mm(r[-1])}   arc {1000*sum(math.dist(a,b) for a,b in zip(r,r[1:])):.0f} mm")
    print("  Check: L must be the start post and R the goal post. If not, rebuild with --only <good segment ids>.")
PYEOF
    ;;
  teleop)  need_session; source_ros; exec "$PY" "$HERE/scripts/teleop.py" "${SARGS[@]+"${SARGS[@]}"}" "$@" ;;
  record)  need_session; source_ros; exec "$PY" "$HERE/scripts/record_segment.py" "${SARGS[@]+"${SARGS[@]}"}" "$@" ;;
  sim)     exec "$PY" "$HERE/scripts/teleop.py" --sim "$@" ;;
  inspect) need_session; exec "$PY" "$HERE/scripts/inspect_segments.py" "${SARGS[@]+"${SARGS[@]}"}" "$@" ;;
  merge)
    need_session
    "$PY" "$HERE/scripts/merge_segments.py" "${SARGS[@]+"${SARGS[@]}"}" "$@"
    echo; "$PY" "$HERE/scripts/inspect_segments.py" --session "$SESSION"
    echo; echo "[buzzwire] Next: ./buzzwire_teach.sh build   (or calibrate) to make the LR/RL files"
    ;;
  build)
    need_session
    echo "[buzzwire] Building trajectories for $SESSION"
    "$PY" "$HERE/scripts/merge_segments.py" "${SARGS[@]+"${SARGS[@]}"}" "$@"
    "$PY" "$HERE/scripts/calibrate_and_optimize.py" --session "$SESSION" --smooth
    echo
    "$0" status
    cat <<MSG

Ready. Run on the robot (./buzzwire_run.sh uses this active session automatically):
  ./buzzwire_run.sh goto L                    # park at the start post
  ./buzzwire_run.sh lr --speed 0.35           # L -> R  raw (demonstrated contours)
  ./buzzwire_run.sh rl --speed 0.35           # R -> L  raw
  ./buzzwire_run.sh lr -smooth --speed 0.35   # L -> R  smooth
  ./buzzwire_run.sh rl -smooth --speed 0.35   # R -> L  smooth
MSG
    ;;
  run)
    exec "$HERE/buzzwire_run.sh" "$@"
    ;;
  calibrate)
    need_session
    exec "$PY" "$HERE/scripts/calibrate_and_optimize.py" "${SARGS[@]+"${SARGS[@]}"}" "$@"
    ;;
  replay)
    if [[ " $* " == *" --dry-run "* || " $* " == *" --sim "* ]]; then exec "$PY" "$HERE/scripts/replay_trajectory.py" "${SARGS[@]+"${SARGS[@]}"}" "$@"; fi
    source_ros; exec "$PY" "$HERE/scripts/replay_trajectory.py" "${SARGS[@]+"${SARGS[@]}"}" "$@" ;;
  test)
    for t in selftest_solver selftest_teleop selftest_pipeline; do echo "=== $t ==="; "$PY" "$HERE/scripts/$t.py" | tail -3; done
    ;;
  clean)
    S="${1:-$SESSION}"
    [[ -z "$S" ]] && { echo "no session found"; exit 1; }
    rm -f "$S"/merged_trajectory.csv "$S"/merge_report.json "$S"/replay_state.json "$S"/calibrated_wire_*.csv "$S"/calibration_metrics.json
    echo "cleaned $S (segments kept)" ;;
  help|-h|--help|*) sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
