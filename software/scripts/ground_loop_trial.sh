#!/usr/bin/env bash
# Run one ground_loop_trial on the robot. This publishes /cmd_vel and moves it.
# Keep the robot in a clear test area and stop it if anything looks unexpected.
# This entrypoint never starts, stops, or reconfigures m3.sh or the micro-ROS agent.
#
# Usage:
#   bash software/scripts/ground_loop_trial.sh --run
#   bash software/scripts/ground_loop_trial.sh --run --csv-prefix dorm_retry
#
# --run is mandatory. Without it, this script exits before loading ROS or
# checking the robot. CSVs are written under field_data/ with a UTC timestamp.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PACKAGE_DIR="$REPO_ROOT/software/ros2/m3pro_nav"
ROS_INSTALL="$REPO_ROOT/software/ros2/install/setup.bash"
FIELD_DATA="$REPO_ROOT/field_data"
RUN=0
CSV_PREFIX="ground_loop_trial"

usage() {
    cat <<'USAGE'
Usage: ground_loop_trial.sh --run [--csv-prefix PREFIX]

WARNING: --run launches a live ground trial. The node publishes /cmd_vel and
the robot will move. Use only in a clear test area with an operator ready to
stop the run. This script does not start or stop m3.sh or the micro-ROS agent.

Options:
  --run                 Explicitly authorize this live motion run (required)
  --csv-prefix PREFIX   CSV filename prefix (default: ground_loop_trial)
  -h, --help            Show this help without loading ROS or moving the robot
USAGE
}

die() { echo "[ground_loop_trial] ERROR: $*" >&2; exit 1; }
log() { echo "[ground_loop_trial] $*"; }

while (($#)); do
    case "$1" in
        --run) RUN=1; shift ;;
        --csv-prefix)
            (($# >= 2)) || die '--csv-prefix requires a value'
            CSV_PREFIX="$2"
            shift 2
            ;;
        -h|--help) usage; exit 0 ;;
        *) die "unknown argument: $1" ;;
    esac
done

(( RUN == 1 )) || {
    usage >&2
    die 'live motion is disabled unless --run is supplied explicitly'
}
[[ "$CSV_PREFIX" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] \
    || die '--csv-prefix must start with a letter or digit and contain only letters, digits, dot, underscore, or hyphen'

# Match field_scan_session.sh: ROS is already expected to be running on the car.
# Do not call m3.sh up/down or touch the agent, proxy, or other runtime services.
[[ -f /opt/ros/humble/setup.bash ]] || die 'ROS Humble setup not found on robot'
set +u
source /opt/ros/humble/setup.bash
[[ -f "$HOME/yahboomcar_ws/install/setup.bash" ]] \
    && source "$HOME/yahboomcar_ws/install/setup.bash"
[[ -f "$HOME/M3Pro_ws/install/setup.bash" ]] \
    && source "$HOME/M3Pro_ws/install/setup.bash"
[[ -f "$ROS_INSTALL" ]] \
    || die 'm3pro_nav install missing; run prepare_field_car.sh from the Mac first'
source "$ROS_INSTALL"
set -u
export ROS_DOMAIN_ID=30

command -v ros2 >/dev/null 2>&1 || die 'ros2 not found after sourcing the robot workspaces'
[[ -f "$PACKAGE_DIR/setup.py" ]] || die "ROS package not found: $PACKAGE_DIR"
command -v timeout >/dev/null 2>&1 || die 'timeout not found; cannot perform bounded read-only preflight'

topic_info="$(timeout 8 ros2 topic info -v /odom_raw 2>/dev/null || true)"
[[ "$topic_info" == *'Type: nav_msgs/msg/Odometry'* ]] \
    || die '/odom_raw is missing or is not nav_msgs/msg/Odometry'
odom_frame="$(timeout 5 ros2 topic echo --once --field header.frame_id /odom_raw 2>/dev/null | sed -n '1p' || true)"
base_frame="$(timeout 5 ros2 topic echo --once --field child_frame_id /odom_raw 2>/dev/null | sed -n '1p' || true)"
[[ "$odom_frame" == odom ]] \
    || die "expected /odom_raw header.frame_id=odom, got '${odom_frame:-unread}'"
[[ "$base_frame" == base_footprint ]] \
    || die "expected /odom_raw child_frame_id=base_footprint, got '${base_frame:-unread}'"

# Refuse to take command ownership when another /cmd_vel publisher is visible.
cmd_info="$(timeout 8 ros2 topic info -v /cmd_vel 2>/dev/null || true)"
[[ "$cmd_info" =~ Publisher\ count:\ ([0-9]+) ]] \
    || die 'cannot verify /cmd_vel publisher ownership; refusing to start motion'
publisher_count="${BASH_REMATCH[1]}"
[[ "$publisher_count" == 0 ]] \
    || die "/cmd_vel already has $publisher_count publisher(s); stop and resolve ownership before this trial"

mkdir -p "$FIELD_DATA"
stamp="$(date -u +%Y%m%dT%H%M%S%N)"
csv_path="$FIELD_DATA/${CSV_PREFIX}_${stamp}.csv"
[[ ! -e "$csv_path" ]] || die "refusing to overwrite existing CSV: $csv_path"

log 'LIVE MOTION RUN: publishing /cmd_vel; operator must remain ready to stop the robot'
log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID; odom frame=$odom_frame; base frame=$base_frame"
log "CSV=$csv_path"
exec ros2 run m3pro_nav ground_loop_trial \
    --run \
    --expected-odom-frame odom \
    --expected-base-frame base_footprint \
    --odom-synchronous-control \
    --yaw-source odom \
    --csv "$csv_path"
