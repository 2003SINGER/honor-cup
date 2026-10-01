#!/usr/bin/env bash
# field_scan_session.sh —— 现场一键采集 (只做实验, 不做软件开发)
#
# 用法:
#   bash ~/honor-cup/software/scripts/field_scan_session.sh --cell 3 2 --heading N --label dead_end_2cells
#   bash ~/honor-cup/software/scripts/field_scan_session.sh --config configs/scan_experiments/dead_end_2cells.yaml \
#       --cell 3 2 --heading N
#
# 自动完成: 环境检查 / topic·type·frame 核对 / maze anchor / scan_debug
#           / rosbag / git SHA 与参数记录 / 结束后生成 summary
# 图形环境下可传 --rviz 启动 RViz; 默认不额外启动 GUI。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PACKAGE_DIR="$REPO_ROOT/software/ros2/m3pro_nav"
FIELD_DATA="$REPO_ROOT/field_data"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_multi}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom_raw}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data_raw}"
JOY_TOPIC="${JOY_TOPIC:-/joy}"
EXTRINSIC_YAML="${EXTRINSIC_YAML:-}"
CELL_X="" CELL_Y="" HEADING="N" LABEL="manual" CONFIG="" START_RVIZ=0

log() { echo "[field_session] $*"; }
die() { echo "[field_session] ERROR: $*" >&2; exit 1; }

# This session runs on the car, where the ROS graph and sensors are available.
[[ -f /opt/ros/humble/setup.bash ]] || die "ROS Humble setup not found on car"
set +u
source /opt/ros/humble/setup.bash
[[ -f "$HOME/yahboomcar_ws/install/setup.bash" ]] && source "$HOME/yahboomcar_ws/install/setup.bash"
[[ -f "$HOME/M3Pro_ws/install/setup.bash" ]] && source "$HOME/M3Pro_ws/install/setup.bash"
[[ -f "$REPO_ROOT/software/ros2/install/setup.bash" ]] \
    || die "m3pro_nav install missing; run prepare_field_car.sh from the Mac first"
source "$REPO_ROOT/software/ros2/install/setup.bash"
set -u
export ROS_DOMAIN_ID=30

while [[ $# -gt 0 ]]; do
    case "$1" in
        --cell) CELL_X="$2"; CELL_Y="$3"; shift 3 ;;
        --heading) HEADING="$2"; shift 2 ;;
        --label) LABEL="$2"; shift 2 ;;
        --config) CONFIG="$2"; shift 2 ;;
        --rviz) START_RVIZ=1; shift ;;
        --scan-topic) SCAN_TOPIC="$2"; shift 2 ;;
        --odom-topic) ODOM_TOPIC="$2"; shift 2 ;;
        *) die "unknown argument: $1" ;;
    esac
done
[[ -n "$CELL_X" && -n "$CELL_Y" ]] || die "--cell X Y is required"
[[ "$HEADING" =~ ^[NESW]$ ]] || die "--heading must be N/E/S/W"

# ---- 环境检查 ----
command -v ros2 >/dev/null 2>&1 || die "ros2 not found: source the robot workspace first"
[[ -d "$PACKAGE_DIR" ]] || die "package dir missing: $PACKAGE_DIR"
log "ROS_DISTRO=${ROS_DISTRO:-unknown}"
if [[ -n "${ROS_LOCALHOST_ONLY:-}" ]]; then log "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"; fi

# ---- topic 存在性 / 类型 / frame 核对 ----
check_topic() {
    local topic="$1" expected_type="$2"
    local info
    info=$(timeout 8 ros2 topic info -v "$topic" 2>/dev/null || true)
    [[ -n "$info" ]] || die "topic $topic not found — start the robot drivers first"
    echo "$info" | grep -q "Type: ${expected_type}" \
        || die "topic $topic is not ${expected_type}: $(echo "$info" | grep 'Type:')"
    log "topic OK: $topic (${expected_type})"
}
check_topic "$SCAN_TOPIC" "sensor_msgs/msg/LaserScan"
check_topic "$ODOM_TOPIC" "nav_msgs/msg/Odometry"
if timeout 8 ros2 topic list 2>/dev/null | grep -q "^${IMU_TOPIC}$"; then
    check_topic "$IMU_TOPIC" "sensor_msgs/msg/Imu"
    IMU_RECORD="$IMU_TOPIC"
else
    log "imu topic $IMU_TOPIC absent — skipping imu record"
    IMU_RECORD=""
fi
if timeout 8 ros2 topic list 2>/dev/null | grep -q "^${JOY_TOPIC}$"; then
    check_topic "$JOY_TOPIC" "sensor_msgs/msg/Joy"
    JOY_VISIBLE_AT_START=true
    log "joy is visible at capture start"
else
    JOY_VISIBLE_AT_START=false
    log "joy topic $JOY_TOPIC absent at capture start — rosbag will keep discovering it if the controller starts later"
fi
JOY_RECORD="$JOY_TOPIC"
LASER_FRAME=$(timeout 5 ros2 topic echo --once --field header.frame_id "$SCAN_TOPIC" 2>/dev/null | sed -n '1p' || true)
log "scan frame_id: ${LASER_FRAME:-<unread>}"
ODOM_FRAME=$(timeout 5 ros2 topic echo --once --field header.frame_id "$ODOM_TOPIC" 2>/dev/null | sed -n '1p' || true)
[[ -n "$ODOM_FRAME" ]] || die "cannot read header.frame_id from $ODOM_TOPIC; verify the odometry driver is publishing"
BASE_FRAME=$(timeout 5 ros2 topic echo --once --field child_frame_id "$ODOM_TOPIC" 2>/dev/null | sed -n '1p' || true)
[[ -n "$BASE_FRAME" ]] || die "cannot read child_frame_id from $ODOM_TOPIC; verify the odometry driver is publishing"
log "odom frame_id: $ODOM_FRAME; base child_frame_id: $BASE_FRAME"

# ---- session 目录 ----
STAMP=$(date +%Y%m%d_%H%M%S)
SESSION_BASE="$FIELD_DATA/${STAMP}_${LABEL}"
SESSION_DIR="$SESSION_BASE"
SESSION_SUFFIX=1
while ! mkdir "$SESSION_DIR" 2>/dev/null; do
    SESSION_DIR="${SESSION_BASE}_$(printf '%02d' "$SESSION_SUFFIX")"
    SESSION_SUFFIX=$((SESSION_SUFFIX + 1))
done
mkdir -p "$SESSION_DIR/bag"
exec > >(tee -a "$SESSION_DIR/session.log") 2>&1

# --config is copied below for metadata only; it does not set topics or recording duration.
# The session runs until Ctrl-C; the template record_seconds value is descriptive only.

# ---- 元数据 ----
GIT_SHA=$(cd "$REPO_ROOT" && git rev-parse HEAD 2>/dev/null || echo "unknown")
DEPLOYED_GIT_SHA="$(cat "$REPO_ROOT/software/ros2/install/deployed_git_sha" 2>/dev/null || true)"
[[ -n "$DEPLOYED_GIT_SHA" ]] || DEPLOYED_GIT_SHA="$GIT_SHA"
cat > "$SESSION_DIR/session.yaml" << EOF
label: ${LABEL}
timestamp: ${STAMP}
git_sha: ${GIT_SHA}
deployed_git_sha: ${DEPLOYED_GIT_SHA}
ros_distro: ${ROS_DISTRO:-unknown}
cell: [${CELL_X}, ${CELL_Y}]
heading: ${HEADING}
scan_topic: ${SCAN_TOPIC}
odom_topic: ${ODOM_TOPIC}
imu_topic: ${IMU_RECORD}
joy_topic: ${JOY_RECORD}
joy_topic_visible_at_start: ${JOY_VISIBLE_AT_START}
laser_frame: ${LASER_FRAME:-unknown}
odom_frame: ${ODOM_FRAME}
base_frame: ${BASE_FRAME}
extrinsic_yaml: ${EXTRINSIC_YAML:-none}
config: ${CONFIG:-none}
EOF
cp "${CONFIG:-/dev/null}" "$SESSION_DIR/experiment.yaml" 2>/dev/null || true
timeout 8 ros2 topic list -v > "$SESSION_DIR/topics.txt" 2>/dev/null || true
log "session dir: $SESSION_DIR"
CMD_VEL_INFO=$(timeout 8 ros2 topic info -v /cmd_vel 2>/dev/null || true)
if [[ -z "$CMD_VEL_INFO" ]]; then
    log "WARNING: /cmd_vel is not visible yet; recording odometry/scan and waiting for any /cmd_vel publisher that appears"
elif ! grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$CMD_VEL_INFO"; then
    log "WARNING: /cmd_vel has no publisher at capture start; recording anyway (manual motion will still be measured by odometry and scan)"
fi

# ---- 采集主题 ----
RECORD_TOPICS="$SCAN_TOPIC $ODOM_TOPIC /cmd_vel /tf /tf_static /scan_debug/markers"
[[ -n "$IMU_RECORD" ]] && RECORD_TOPICS="$RECORD_TOPICS $IMU_RECORD"
RECORD_TOPICS="$RECORD_TOPICS $JOY_RECORD"

cleanup() {
    local exit_status=$?
    [[ "${CLEANUP_DONE:-0}" == 1 ]] && return
    CLEANUP_DONE=1
    trap - EXIT INT TERM
    if [[ "${INTERRUPTED:-0}" != 1 && -n "${LAUNCH_PID:-}" ]] \
            && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        LAUNCH_FAILED=1
    fi
    [[ "${LAUNCH_FAILED:-0}" == 1 ]] && exit_status=1
    log "stopping..."
    [[ -n "${BAG_PID:-}" ]] && kill -INT "$BAG_PID" 2>/dev/null || true
    [[ -n "${LAUNCH_PID:-}" ]] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
    [[ -n "${RVIZ_PID:-}" ]] && kill -INT "$RVIZ_PID" 2>/dev/null || true
    wait 2>/dev/null || true
    if [[ -s "$SESSION_DIR/frames.jsonl" ]]; then
        log "generating summary..."
        python3 "$REPO_ROOT/software/tools/scan_session_summary.py" "$SESSION_DIR" || true
        if [[ "$exit_status" -eq 0 ]]; then
            log "done: $SESSION_DIR"
        else
            log "session stopped with status $exit_status; partial data: $SESSION_DIR" >&2
        fi
    else
        log "ERROR: frames.jsonl is missing or empty; scan_debug did not produce observations. Session is incomplete: $SESSION_DIR" >&2
        exit_status=1
    fi
    exit "$exit_status"
}
CLEANUP_DONE=0
INTERRUPTED=0
LAUNCH_FAILED=0
trap cleanup EXIT
trap 'INTERRUPTED=1; exit 0' INT
trap 'exit 143' TERM

# ---- 启动: scan_debug + RViz + rosbag ----
EXTRA_ARGS=()
[[ -n "$EXTRINSIC_YAML" ]] && EXTRA_ARGS+=(laser_extrinsic_yaml:="$EXTRINSIC_YAML")
[[ -n "$LASER_FRAME" ]] && EXTRA_ARGS+=(expected_laser_frame:="$LASER_FRAME")
EXTRA_ARGS+=(expected_odom_frame:="$ODOM_FRAME" expected_base_frame:="$BASE_FRAME")

ros2 launch m3pro_nav scan_debug.launch.py \
    cell_x:="$CELL_X" cell_y:="$CELL_Y" heading:="$HEADING" \
    scan_topic:="$SCAN_TOPIC" odom_topic:="$ODOM_TOPIC" \
    session_dir:="$SESSION_DIR" \
    "${EXTRA_ARGS[@]}" &
LAUNCH_PID=$!

if [[ "$START_RVIZ" == 1 ]] && command -v rviz2 >/dev/null 2>&1; then
    rviz2 -d "$PACKAGE_DIR/config/scan_debug.rviz" || true &
    RVIZ_PID=$!
elif [[ "$START_RVIZ" == 1 ]]; then
    log "rviz2 not found — continuing without RViz"
else
    log "RViz disabled; pass --rviz to start it when a display is available"
fi

ros2 bag record -o "$SESSION_DIR/bag/record" $RECORD_TOPICS &
BAG_PID=$!

log "recording — Ctrl-C to stop"
while kill -0 "$BAG_PID" 2>/dev/null; do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        launch_status=0
        wait "$LAUNCH_PID" || launch_status=$?
        LAUNCH_FAILED=1
        die "scan_debug launch exited while recording (status=$launch_status)"
    fi
    sleep 1
done
wait "$BAG_PID"
