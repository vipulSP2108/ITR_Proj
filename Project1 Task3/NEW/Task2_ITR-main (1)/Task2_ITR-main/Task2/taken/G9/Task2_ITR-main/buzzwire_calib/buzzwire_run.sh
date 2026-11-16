#!/usr/bin/env bash
# =============================================================================
# buzzwire_run.sh - Single-Command Runner for Calibrated Buzz Wire LR and RL
#
# USAGE:
#   ./buzzwire_run.sh goto L                     # Drive arm to Post L (Start) from ANY orientation
#   ./buzzwire_run.sh goto R                     # Drive arm to Post R (Goal) from ANY orientation
#   ./buzzwire_run.sh goto home                  # Drive arm to Home position (all zeros)
#   ./buzzwire_run.sh rl [-smooth] [--speed 0.35]    # Safe wire-guided return R -> L
#   ./buzzwire_run.sh lr [-smooth] [--speed 0.70]    # Wire-guided traversal L -> R
#   ./buzzwire_run.sh lr [show|balanced|fast]        # Wire traversal with named preset
#   ./buzzwire_run.sh calibrate [-smooth]            # Re-run full calibration & averaging
#   ./buzzwire_run.sh dry-run [lr|rl]                # Dry-run speed audit without motion
#   ./buzzwire_run.sh sim [lr|rl] [-smooth]          # Practice on simulated virtual arm
#
# SESSION: uses data/ACTIVE_SESSION (set by ./buzzwire_teach.sh new / use).
#          Override with --session DIR anywhere on the line, or BUZZ_SESSION=DIR.
#
# SPEED LIMITS & RECOMMENDATIONS:
#   Global speed cap: v_max = 60 mm/s (speed=1.0)
#   --speed 0.25 : Safe return / trial      (15 mm/s cruise, ~134s duration)
#   --speed 0.40 : Show / Demo profile      (24 mm/s cruise, ~69s duration)
#   --speed 0.65 : Balanced operational    (39 mm/s cruise, ~55s duration)
#   --speed 1.00 : Fast competition limit   (60 mm/s cruise, ~42s duration)
#   ADVISED MAX  : 1.0 (Speeds > 1.0 saturate joint limits: max_joint_vel=0.5 rad/s)
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-python3}"
# ---- which session? 1) --session DIR   2) $BUZZ_SESSION   3) data/ACTIVE_SESSION (set by ./buzzwire_teach.sh new/use)
ARGS_IN=("$@"); FILTERED=(); DATA_DIR=""
for ((i=0; i<${#ARGS_IN[@]}; i++)); do
  if [[ "${ARGS_IN[$i]}" == "--session" ]]; then DATA_DIR="${ARGS_IN[$((i+1))]:-}"; i=$((i+1))
  else FILTERED+=("${ARGS_IN[$i]}"); fi
done
set -- "${FILTERED[@]+"${FILTERED[@]}"}"
[[ -z "$DATA_DIR" ]] && DATA_DIR="${BUZZ_SESSION:-}"
if [[ -z "$DATA_DIR" && -f "$HERE/data/ACTIVE_SESSION" ]]; then DATA_DIR="$(head -1 "$HERE/data/ACTIVE_SESSION")"; fi
if [[ -z "$DATA_DIR" ]]; then
  echo "[buzzwire_run] ERROR: no session selected. Run ./buzzwire_teach.sh new (then teach + build), or pass --session DIR." >&2
  exit 1
fi
[[ "$DATA_DIR" != /* ]] && { [[ -d "$DATA_DIR" ]] && DATA_DIR="$(cd "$DATA_DIR" && pwd)" || DATA_DIR="$HERE/$DATA_DIR"; }
[[ -d "$DATA_DIR" ]] || { echo "[buzzwire_run] ERROR: session folder $DATA_DIR not found." >&2; exit 1; }
echo "[buzzwire_run] Session: $DATA_DIR"
TRAJ_LR="$DATA_DIR/calibrated_wire_LR.csv"
TRAJ_RL="$DATA_DIR/calibrated_wire_RL.csv"

arc_mm() {   # arc length of a trajectory CSV in mm (for the banner)
  [[ -f "$1" ]] || { echo "?"; return; }
  "$PY" -c "import csv,math,sys;r=[(float(d['x']),float(d['y']),float(d['z'])) for d in csv.DictReader(open(sys.argv[1]))];print(round(1000*sum(math.dist(a,b) for a,b in zip(r,r[1:]))))" "$1" 2>/dev/null || echo "?"
}

# Function to source ROS 2 workspace if real hardware
source_ros() {
  local ws="${WS:-$HOME/Debojit_WS/addverb_ws}"
  set +u
  if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/humble/setup.bash ]]; then 
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "$ws/install/setup.bash" ]]; then 
    source "$ws/install/setup.bash"
    export WS="$ws"
  fi
  set -u
  if ! "$PY" -c "import rclpy" 2>/dev/null; then
    echo "[buzzwire_run] ERROR: rclpy not importable. Source ROS 2 or set WS=..." >&2
    exit 1
  fi
}

# Verify calibrated trajectories exist, or run calibration
ensure_calibrated() {
  if [[ ! -f "$DATA_DIR/calibrated_wire_LR.csv" || ! -f "$DATA_DIR/calibrated_wire_LR_smooth.csv" ]]; then
    if [[ ! -f "$DATA_DIR/merged_trajectory.csv" ]]; then
      echo "[buzzwire_run] No merged_trajectory.csv in $DATA_DIR. Run ./buzzwire_teach.sh build first." >&2; exit 1
    fi
    echo "[buzzwire_run] Calibrated files not found. Running calibration & smoothing pipeline..."
    "$PY" "$HERE/scripts/calibrate_and_optimize.py" --session "$DATA_DIR" --smooth
  fi
}

DIRECTION="${1:-help}"
[[ $# -gt 0 ]] && shift

# Handle goto L / goto R / goto home
if [[ "$DIRECTION" == "goto" || "$DIRECTION" == "GOTO" ]]; then
  DEST="${1:-L}"
  [[ $# -gt 0 ]] && shift
  if [[ " $@ " != *" --sim "* ]]; then source_ros; fi
  exec "$PY" "$HERE/scripts/goto_target.py" "$DEST" --session "$DATA_DIR" "$@"
fi

# Parse flags
SPEED="0.65" # default balanced
DO_SMOOTH="false"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -smooth|--smooth)
      DO_SMOOTH="true"
      shift
      ;;
    --speed)
      SPEED="$2"
      shift 2
      ;;
    show)
      SPEED="0.40"
      shift
      ;;
    balanced)
      SPEED="0.65"
      shift
      ;;
    fast)
      SPEED="1.00"
      shift
      ;;
    *)
      # Numeric float argument
      if [[ "$1" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
        SPEED="$1"
        shift
      else
        EXTRA_ARGS+=("$1")
        shift
      fi
      ;;
  esac
done

# Speed limit advisory check
speed_check=$("$PY" -c "s=float('$SPEED'); print('HIGH' if s > 1.0 else ('VERY_LOW' if s < 0.1 else 'OK'))")
if [[ "$speed_check" == "HIGH" ]]; then
  echo -e "\033[1;33m[buzzwire_run] WARNING: Speed ($SPEED) exceeds advised competition limit (1.0 = 60 mm/s)!\033[0m"
  echo "  Higher speeds risk joint velocity clamp saturation (max_joint_vel = 0.50 rad/s) on steep legs."
fi

# Select trajectory based on -smooth
if [[ "$DO_SMOOTH" == "true" ]]; then
  TRAJ_LR="$DATA_DIR/calibrated_wire_LR_smooth.csv"
  TRAJ_RL="$DATA_DIR/calibrated_wire_RL_smooth.csv"
  SMOOTH_LABEL="Active (CAD-Grade Multi-Stage Smoothing, $(arc_mm "$TRAJ_LR") mm arc)"
else
  TRAJ_LR="$DATA_DIR/calibrated_wire_LR.csv"
  TRAJ_RL="$DATA_DIR/calibrated_wire_RL.csv"
  SMOOTH_LABEL="Standard (SVD Planar Projection, Preserves Demonstrated Contours, $(arc_mm "$TRAJ_LR") mm arc)"
fi

case "$DIRECTION" in
  rl|RL)
    ensure_calibrated
    echo "==========================================================================="
    echo "[buzzwire_run] EXECUTING RETURN: Right Goal (R) -> Left Start (L)"
    echo "  Trajectory : $(basename "$TRAJ_RL")"
    echo "  Speed      : $SPEED  (Advised return speed: 0.25 - 0.35)"
    echo "  Smoothing  : $SMOOTH_LABEL"
    echo "==========================================================================="
    source_ros
    exec "$PY" "$HERE/scripts/replay_trajectory.py" --traj "$TRAJ_RL" --speed "$SPEED" "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"
    ;;

  lr|LR)
    ensure_calibrated
    echo "==========================================================================="
    echo "[buzzwire_run] EXECUTING FORWARD: Left Start (L) -> Right Goal (R)"
    echo "  Trajectory : $(basename "$TRAJ_LR")"
    echo "  Speed      : $SPEED"
    echo "  Smoothing  : $SMOOTH_LABEL"
    echo "==========================================================================="
    source_ros
    exec "$PY" "$HERE/scripts/replay_trajectory.py" --traj "$TRAJ_LR" --speed "$SPEED" "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"
    ;;

  calibrate)
    echo "[buzzwire_run] Re-running SVD plane fitting, multi-stage smoothing & auto-tangent calibration..."
    exec "$PY" "$HERE/scripts/calibrate_and_optimize.py" --session "$DATA_DIR" --smooth
    ;;

  dry-run)
    ensure_calibrated
    TARGET_TRAJ="$TRAJ_LR"
    for arg in "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"; do
      if [[ "$arg" == "rl" || "$arg" == "RL" ]]; then TARGET_TRAJ="$TRAJ_RL"; fi
    done
    echo "[buzzwire_run] Dry-run profile audit for $TARGET_TRAJ (speed scale $SPEED):"
    exec "$PY" "$HERE/scripts/replay_trajectory.py" --traj "$TARGET_TRAJ" --speed "$SPEED" --dry-run
    ;;

  sim)
    ensure_calibrated
    TARGET_TRAJ="$TRAJ_LR"
    for arg in "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"; do
      if [[ "$arg" == "rl" || "$arg" == "RL" ]]; then TARGET_TRAJ="$TRAJ_RL"; fi
    done
    echo "[buzzwire_run] Simulation playback on virtual arm (no hardware):"
    exec "$PY" "$HERE/scripts/replay_trajectory.py" --traj "$TARGET_TRAJ" --sim --speed "$SPEED"
    ;;

  help|-h|--help|*)
    echo "========================================================================"
    echo "Buzz Wire Autonomous Runner - Single Command Interface"
    echo "========================================================================"
    echo "Usage:"
    echo "  ./buzzwire_run.sh goto L [--yes]                 # Move arm to Start Post L"
    echo "  ./buzzwire_run.sh goto R [--yes]                 # Move arm to Goal Post R"
    echo "  ./buzzwire_run.sh goto home                      # Move arm to Home (0,0,0,0,0,0)"
    echo "  ./buzzwire_run.sh rl [-smooth] [--speed 0.35]    # Wire-guided return R -> L"
    echo "  ./buzzwire_run.sh lr [-smooth] [--speed 0.65]    # Wire-guided traversal L -> R"
    echo "  ./buzzwire_run.sh lr [show|balanced|fast]        # Presets"
    echo "  ./buzzwire_run.sh dry-run [lr|rl]                # Inspect profile"
    echo "  ./buzzwire_run.sh sim [lr|rl]                    # Practice on simulated arm"
    echo "  ./buzzwire_run.sh calibrate [-smooth]            # Re-run calibration pipeline"
    echo "========================================================================"
    ;;
esac
