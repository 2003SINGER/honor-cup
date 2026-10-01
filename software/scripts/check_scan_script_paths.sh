#!/usr/bin/env bash
# ROS-free smoke check for repo-relative paths used by field scan/replay scripts.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PACKAGE_DIR="$REPO_ROOT/software/ros2/m3pro_nav"
SUMMARY_TOOL="$REPO_ROOT/software/tools/scan_session_summary.py"

for script in field_scan_session.sh replay_scan_session.sh; do
    grep -Fq '$(dirname "${BASH_SOURCE[0]}")/../..' "$SCRIPT_DIR/$script" || {
        echo "wrong REPO_ROOT expression in $script" >&2
        exit 1
    }
done
grep -Fq 'SCAN_TOPIC="${SCAN_TOPIC:-/scan_multi}"' "$SCRIPT_DIR/field_scan_session.sh" || {
    echo "field session default scan topic is not /scan_multi" >&2
    exit 1
}
grep -Fq 'RECORD_TOPICS="$SCAN_TOPIC $ODOM_TOPIC /cmd_vel /tf /tf_static /scan_debug/markers"' "$SCRIPT_DIR/field_scan_session.sh" || {
    echo "field session rosbag must include /cmd_vel for motion correlation" >&2
    exit 1
}
grep -Fq 'RECORD_TOPICS="$RECORD_TOPICS $JOY_RECORD"' "$SCRIPT_DIR/field_scan_session.sh" || {
    echo "field session rosbag must retain the /joy discovery request" >&2
    exit 1
}
grep -Fq 'field_scan_cleanup_watchdog.sh' "$SCRIPT_DIR/field_scan_session.sh" \
    && grep -Fq 'WATCHDOG_REQUEST="$SESSION_DIR/cleanup.request"' "$SCRIPT_DIR/field_scan_session.sh" \
    && grep -Fq 'WATCHDOG_DONE="$SESSION_DIR/cleanup.done"' "$SCRIPT_DIR/field_scan_session.sh" \
    && grep -Fq 'WATCHDOG_READY="$SESSION_DIR/cleanup.ready"' "$SCRIPT_DIR/field_scan_session.sh" \
    && grep -Fq "trap 'exit 0' INT" "$SCRIPT_DIR/field_scan_session.sh" \
    && grep -Fq "trap 'exit 129' HUP" "$SCRIPT_DIR/field_scan_session.sh" \
    && grep -Fq "trap 'exit 143' TERM" "$SCRIPT_DIR/field_scan_session.sh" || {
    echo "field session must delegate signal/exit cleanup to the detached watchdog" >&2
    exit 1
}
grep -Fq 'stop_group "$SESSION_DIR/bag.pid" "rosbag"' "$SCRIPT_DIR/field_scan_cleanup_watchdog.sh" \
    && grep -Fq 'setsid bash -c' "$SCRIPT_DIR/field_scan_session.sh" || {
    echo "field session must isolate rosbag and let watchdog finalize it before summarizing" >&2
    exit 1
}
[[ -x "$SCRIPT_DIR/field_scan_cleanup_watchdog.sh" ]] || {
    echo "field session cleanup watchdog must be executable" >&2
    exit 1
}
grep -Fq 'field_scan_cleanup_watchdog.sh' "$SCRIPT_DIR/prepare_field_car.sh" || {
    echo "field car preparation must sync the cleanup watchdog" >&2
    exit 1
}
[[ -d "$PACKAGE_DIR" ]] || { echo "package path missing: $PACKAGE_DIR" >&2; exit 1; }
[[ -f "$SUMMARY_TOOL" ]] || { echo "summary tool missing: $SUMMARY_TOOL" >&2; exit 1; }
printf 'scan script paths OK\nrepo: %s\npackage: %s\nsummary: %s\n' "$REPO_ROOT" "$PACKAGE_DIR" "$SUMMARY_TOOL"
