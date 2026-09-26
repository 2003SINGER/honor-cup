#!/usr/bin/env bash
# replay_scan_session.sh —— 离线重放 (与现场同一条算法链, 零第二套代码)
#
# 用法:
#   ./scripts/replay_scan_session.sh field_data/20260926_143011_dead_end_2cells
#
# 重放 = 读 session.yaml 的同参数启动 scan_debug (session_dir 指向重放目录)
#        + ros2 bag play 喂同一批消息 → 同一 ScanAdapter/FrameProjector/
#        GridAssociation/FrameAccumulator → 重生成 summary。
# 同一 bag 重放两次, 与时间无关的统计必须一致。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_DIR="$REPO_ROOT/software/ros2/m3pro_nav"

log() { echo "[replay] $*"; }
die() { echo "[replay] ERROR: $*" >&2; exit 1; }

SESSION_DIR="${1:-}"
[[ -n "$SESSION_DIR" && -d "$SESSION_DIR" ]] || die "usage: replay_scan_session.sh <session_dir>"
command -v ros2 >/dev/null 2>&1 || die "ros2 not found: source the workspace first"

BAG_DIR=$(find "$SESSION_DIR/bag" -maxdepth 2 -name '*.db3' -exec dirname {} \; | head -1)
[[ -n "$BAG_DIR" ]] || die "no rosbag database under $SESSION_DIR/bag"

# 从 session.yaml 读取同参数 (yaml 不含复杂结构, 逐行解析足够)
read_yaml() { grep -E "^$1:" "$SESSION_DIR/session.yaml" | head -1 | sed "s/^$1:[[:space:]]*//" | tr -d '"' ; }
CELL=$(read_yaml cell)                  # "[3, 2]"
CELL_X=$(echo "$CELL" | tr -d '[]' | cut -d, -f1 | tr -d ' ')
CELL_Y=$(echo "$CELL" | tr -d '[]' | cut -d, -f2 | tr -d ' ')
HEADING=$(read_yaml heading)
SCAN_TOPIC=$(read_yaml scan_topic)
ODOM_TOPIC=$(read_yaml odom_topic)
EXTRINSIC_YAML=$(read_yaml extrinsic_yaml)
[[ "$EXTRINSIC_YAML" == "none" ]] && EXTRINSIC_YAML=""
log "replaying: cell=($CELL_X,$CELL_Y) heading=$HEADING scan=$SCAN_TOPIC bag=$BAG_DIR"

REPLAY_DIR="$SESSION_DIR/replay_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$REPLAY_DIR"
cp "$SESSION_DIR/session.yaml" "$REPLAY_DIR/session.yaml"

cleanup() {
    [[ -n "${PLAY_PID:-}" ]] && kill -INT "$PLAY_PID" 2>/dev/null || true
    [[ -n "${LAUNCH_PID:-}" ]] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait 2>/dev/null || true
    log "generating replay summary..."
    python3 "$REPO_ROOT/software/tools/scan_session_summary.py" "$REPLAY_DIR" || true
    log "replay done: $REPLAY_DIR"
    if [[ -f "$SESSION_DIR/summary.json" && -f "$REPLAY_DIR/summary.json" ]]; then
        log "diff vs online summary (time-dependent fields excluded):"
        python3 - "$SESSION_DIR/summary.json" "$REPLAY_DIR/summary.json" << 'PYEOF' || true
import json, sys
a = json.load(open(sys.argv[1])); b = json.load(open(sys.argv[2]))
for k in ('n_frames', 'n_rays', 'n_valid', 'unique_rate',
          'ambiguous_rate', 'outcomes', 'reject_reasons',
          'by_range_bin', 'per_edge_counts', 'open_edge_counts'):
    if a.get(k) != b.get(k):
        print(f'  MISMATCH {k}: online={a.get(k)} replay={b.get(k)}')
        sys.exit(1)
print('  IDENTICAL (all time-independent statistics)')
PYEOF
    fi
}
trap cleanup EXIT INT TERM

EXTRA_ARGS=()
[[ -n "$EXTRINSIC_YAML" ]] && EXTRA_ARGS+=(laser_extrinsic_yaml:="$EXTRINSIC_YAML")
ros2 launch m3pro_nav scan_debug.launch.py \
    cell_x:="$CELL_X" cell_y:="$CELL_Y" heading:="$HEADING" \
    scan_topic:="$SCAN_TOPIC" odom_topic:="$ODOM_TOPIC" \
    session_dir:="$REPLAY_DIR" \
    "${EXTRA_ARGS[@]}" &
LAUNCH_PID=$!
sleep 2

ros2 bag play "$BAG_DIR" --clock &
PLAY_PID=$!
log "replaying bag — Ctrl-C to stop"
wait $PLAY_PID
