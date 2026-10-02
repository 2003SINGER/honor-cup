#!/usr/bin/env bash
# Detached owner for field_scan_session cleanup. Survives loss of the SSH/PTTY shell.
set -u

SESSION_DIR="$1"
REPO_ROOT="$2"
OWNER_PID="$3"
REQUEST_FILE="$4"
DONE_FILE="$5"
READY_FILE="$6"
LOG_FILE="$SESSION_DIR/session.log"

log() {
    printf '[field_session] %s\n' "$*" >> "$LOG_FILE"
}

: > "$READY_FILE"

# The parent asks for cleanup on normal exit/signals. If it vanishes first,
# take over automatically. Allow its last child-pid file write to finish.
while [[ ! -e "$REQUEST_FILE" ]] && kill -0 "$OWNER_PID" 2>/dev/null; do
    sleep 0.25
done
if [[ ! -e "$REQUEST_FILE" ]]; then sleep 0.5; fi

exit_status=0
if [[ -s "$REQUEST_FILE" ]]; then
    read -r exit_status < "$REQUEST_FILE" || exit_status=1
else
    exit_status=0
    log "parent shell disappeared; watchdog is finalizing the capture"
fi
log "stopping..."

stop_group() {
    local pid_file="$1" label="$2" pid remaining
    [[ -s "$pid_file" ]] || return 0
    read -r pid < "$pid_file" || return 0
    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    kill -0 -- "-$pid" 2>/dev/null || return 0
    kill -INT -- "-$pid" 2>/dev/null || true
    for remaining in {1..10}; do
        kill -0 -- "-$pid" 2>/dev/null || return 0
        sleep 1
    done
    if kill -0 -- "-$pid" 2>/dev/null; then
        log "$label did not stop after SIGINT; sending SIGTERM"
        kill -TERM -- "-$pid" 2>/dev/null || true
        for remaining in {1..3}; do
            kill -0 -- "-$pid" 2>/dev/null || return 0
            sleep 1
        done
    fi
    if kill -0 -- "-$pid" 2>/dev/null; then
        log "$label did not stop after SIGTERM; sending SIGKILL"
        kill -KILL -- "-$pid" 2>/dev/null || true
    fi
}

# rosbag must close its storage metadata while scan_debug is still alive.
stop_group "$SESSION_DIR/bag.pid" "rosbag"
stop_group "$SESSION_DIR/launch.pid" "scan_debug launch"
stop_group "$SESSION_DIR/rviz.pid" "RViz"

# Rosbag can exit successfully while a requested topic produced no messages.
# Check the finalized SQLite bag after rosbag closes, and keep all original
# bag files even when this diagnostic marks the session incomplete.
if ! python3 - "$SESSION_DIR" "$REPO_ROOT" >> "$LOG_FILE" 2>&1 <<'PY'
import json
import pathlib
import sys

session = pathlib.Path(sys.argv[1])
sys.path.insert(0, str(pathlib.Path(sys.argv[2]) / 'software' / 'tools'))
from scan_session_summary import audit_capture_topics

report = audit_capture_topics(session)
(session / 'raw_scan_capture_check.json').write_text(
    json.dumps(report, indent=2) + '\n', encoding='utf-8')
for name, entry in report['required_topics'].items():
    print(f"[field_session] bag topic {name}: type={entry['type'] or '<missing>'} "
          f"messages={entry['message_count']}")
bag_dir = session / 'bag' / 'record'
if not report['complete']:
    print('[field_session] WARNING: CAPTURE INCOMPLETE; required bag topics '
          f"missing/empty={report['missing_or_empty_topics']}; "
          f"raw bag retained at {bag_dir}")
    if report['errors']:
        print(f"[field_session] bag verification errors: {report['errors']}")
    sys.exit(1)
print('[field_session] required raw and motion topics have recorded messages')
PY
then
    log "WARNING: bag topic verification failed; capture is incomplete and raw bag is retained at $SESSION_DIR/bag/record"
    exit_status=1
fi

if [[ -s "$SESSION_DIR/frames.jsonl" ]]; then
    log "generating summary..."
    if ! python3 "$REPO_ROOT/software/tools/scan_session_summary.py" "$SESSION_DIR" \
            >> "$LOG_FILE" 2>&1; then
        log "summary generation failed; raw partial data retained: $SESSION_DIR"
        exit_status=1
    fi
    if [[ "$exit_status" -eq 0 ]]; then
        log "done: $SESSION_DIR"
    else
        log "session stopped with status $exit_status; partial data: $SESSION_DIR"
    fi
else
    log "ERROR: frames.jsonl is missing or empty; scan_debug did not produce observations. Session is incomplete: $SESSION_DIR"
    exit_status=1
fi

if [[ -s "$SESSION_DIR/summary.json" && -s "$SESSION_DIR/raw_scan_capture_check.json" ]]; then
    if ! python3 - "$SESSION_DIR/summary.json" "$SESSION_DIR/raw_scan_capture_check.json" <<'PY' >> "$LOG_FILE" 2>&1
import json
import pathlib
import sys

summary_path, check_path = map(pathlib.Path, sys.argv[1:])
summary = json.loads(summary_path.read_text(encoding='utf-8'))
summary['raw_scan_capture_check'] = json.loads(
    check_path.read_text(encoding='utf-8'))
summary_path.write_text(json.dumps(summary, indent=2) + '\n',
                         encoding='utf-8')
PY
    then
        log "WARNING: could not add raw topic counts to summary.json; standalone raw_scan_capture_check.json retained"
        exit_status=1
    fi
fi

printf '%s\n' "$exit_status" > "$DONE_FILE"
exit "$exit_status"
