#!/usr/bin/env bash
# GNOME Applications launcher for a manual, read-only full-maze data capture.
set -euo pipefail

REPO_ROOT="${HONOR_CUP_REPO:-$HOME/honor-cup}"
SESSION_SCRIPT="$REPO_ROOT/software/scripts/field_scan_session.sh"

if [[ "${1:-}" == "--in-terminal" ]]; then
    if [[ ! -f "$SESSION_SCRIPT" ]]; then
        echo "Recorder missing: $SESSION_SCRIPT"
        read -r -p 'Press Enter to close...'
        exit 1
    fi
    cd "$REPO_ROOT"
    echo "Manual maze capture: anchor=(3,0), heading=N"
    echo "Drive the full maze with the joystick. Press Ctrl-C in this terminal to stop and save the session."
    echo "This launcher records data only; it does not start joystick control or publish /cmd_vel."
    set +e
    bash "$SESSION_SCRIPT" --cell 3 0 --heading N --label joystick_full_maze
    result=$?
    set -e
    echo
    echo "Capture process ended (status $result). Session files are under: $REPO_ROOT/field_data/"
    read -r -p 'Press Enter to close...'
    exit "$result"
fi

command -v gnome-terminal >/dev/null 2>&1 || {
    echo "gnome-terminal is required to open the visible capture session." >&2
    exit 1
}
gnome-terminal --wait --title="Joystick maze data capture" -- bash -lc '"$HOME/honor-cup/software/scripts/joystick_map_capture_app.sh" --in-terminal'
