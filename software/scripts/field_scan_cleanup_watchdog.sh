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

printf '%s\n' "$exit_status" > "$DONE_FILE"
exit "$exit_status"
