#!/usr/bin/env bash
# field_scan_session.sh —— 现场一键采集 (只做实验, 不做软件开发)
#
# 用法:
#   ./scripts/field_scan_session.sh --cell 3 2 --heading N --label dead_end_2cells
#   ./scripts/field_scan_session.sh --config configs/scan_experiments/dead_end_2cells.yaml \
#       --cell 3 2 --heading N
#
# 自动完成: 环境检查 / topic·type·frame 核对 / maze anchor / scan_debug
#           / RViz / rosbag / git SHA 与参数记录 / 结束后生成 summary
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_DIR="$REPO_ROOT/software/ros2/m3pro_nav"
FIELD_DATA="$REPO_ROOT/field_data"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom_raw}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data_raw}"
EXTRINSIC_YAML="${EXTRINSIC_YAML:-}"
CELL_X="" CELL_Y="" HEADING="N" LABEL="manual" CONFIG=""

log() { echo "[field_session] $*"; }
die() { echo "[field_session] ERROR: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --cell) CELL_X="$2"; CELL_Y="$3"; shift 3 ;;
        --heading) HEADING="$2"; shift 2 ;;
        --label) LABEL="$2"; shift 2 ;;
        --config) CONFIG="$2"; shift 2 ;;
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
    info=$(ros2 topic info -v "$topic" 2>/dev/null || true)
    [[ -n "$info" ]] || die "topic $topic not found — start the robot drivers first"
    echo "$info" | grep -q "Type: ${expected_type}" \
        || die "topic $topic is not ${expected_type}: $(echo "$info" | grep 'Type:')"
    log "topic OK: $topic (${expected_type})"
}
check_topic "$SCAN_TOPIC" "sensor_msgs/msg/LaserScan"
check_topic "$ODOM_TOPIC" "nav_msgs/msg/Odometry"
if ros2 topic list 2>/dev/null | grep -q "^${IMU_TOPIC}$"; then
    check_topic "$IMU_TOPIC" "sensor_msgs/msg/Imu"
    IMU_RECORD="$IMU_TOPIC"
else
    log "imu topic $IMU_TOPIC absent — skipping imu record"
    IMU_RECORD=""
fi
LASER_FRAME=$(ros2 topic echo -n1 --field header.frame_id "$SCAN_TOPIC" 2>/dev/null | tr -d '\n' || true)
log "scan frame_id: ${LASER_FRAME:-<unread>}"

# ---- session 目录 ----
STAMP=$(date +%Y%m%d_%H%M%S)
SESSION_DIR="$FIELD_DATA/${STAMP}_${LABEL}"
mkdir -p "$SESSION_DIR/bag"

# ---- 元数据 ----
GIT_SHA=$(cd "$REPO_ROOT" && git rev-parse HEAD 2>/dev/null || echo "unknown")
cat > "$SESSION_DIR/session.yaml" << EOF
label: ${LABEL}
timestamp: ${STAMP}
git_sha: ${GIT_SHA}
ros_distro: ${ROS_DISTRO:-unknown}
cell: [${CELL_X}, ${CELL_Y}]
heading: ${HEADING}
scan_topic: ${SCAN_TOPIC}
odom_topic: ${ODOM_TOPIC}
imu_topic: ${IMU_RECORD}
laser_frame: ${LASER_FRAME:-unknown}
extrinsic_yaml: ${EXTRINSIC_YAML:-none}
config: ${CONFIG:-none}
EOF
cp "${CONFIG:-/dev/null}" "$SESSION_DIR/experiment.yaml" 2>/dev/null || true
ros2 topic list -v > "$SESSION_DIR/topics.txt" 2>/dev/null || true
log "session dir: $SESSION_DIR"

# ---- 采集主题 ----
RECORD_TOPICS="$SCAN_TOPIC $ODOM_TOPIC /tf /tf_static /scan_debug/markers"
[[ -n "$IMU_RECORD" ]] && RECORD_TOPICS="$RECORD_TOPICS $IMU_RECORD"

cleanup() {
    log "stopping..."
    [[ -n "${BAG_PID:-}" ]] && kill -INT "$BAG_PID" 2>/dev/null || true
    [[ -n "${LAUNCH_PID:-}" ]] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
    [[ -n "${RVIZ_PID:-}" ]] && kill -INT "$RVIZ_PID" 2>/dev/null || true
    wait 2>/dev/null || true
    log "generating summary..."
    python3 "$REPO_ROOT/software/tools/scan_session_summary.py" "$SESSION_DIR" || true
    log "done: $SESSION_DIR"
}
trap cleanup EXIT INT TERM

# ---- 启动: scan_debug + RViz + rosbag ----
EXTRA_ARGS=()
[[ -n "$EXTRINSIC_YAML" ]] && EXTRA_ARGS+=(laser_extrinsic_yaml:="$EXTRINSIC_YAML")
[[ -n "$LASER_FRAME" ]] && EXTRA_ARGS+=(expected_laser_frame:="$LASER_FRAME")

ros2 launch m3pro_nav scan_debug.launch.py \
    cell_x:="$CELL_X" cell_y:="$CELL_Y" heading:="$HEADING" \
    scan_topic:="$SCAN_TOPIC" odom_topic:="$ODOM_TOPIC" \
    session_dir:="$SESSION_DIR" \
    "${EXTRA_ARGS[@]}" &
LAUNCH_PID=$!

if command -v rviz2 >/dev/null 2>&1; then
    rviz2 -d "$PACKAGE_DIR/config/scan_debug.rviz" --fullscreen || true &
    RVIZ_PID=$!
else
    log "rviz2 not found — headless session"
fi

ros2 bag record -o "$SESSION_DIR/bag/record" $RECORD_TOPICS &
BAG_PID=$!

log "recording — Ctrl-C to stop"
wait $BAG_PID
