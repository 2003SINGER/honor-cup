#!/usr/bin/env bash
# GNOME Applications launcher for a manual, read-only full-maze data capture.
set -euo pipefail

REPO_ROOT="${HONOR_CUP_REPO:-$HOME/honor-cup}"
SESSION_SCRIPT="$REPO_ROOT/software/scripts/field_scan_session.sh"
LAUNCHER_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

hold_terminal() {
    [[ -t 0 ]] || return 0
    read -r -p 'Press Enter to close this terminal...'
}

fail() {
    echo "Joystick maze capture: ERROR: $*" >&2
    hold_terminal
    exit 1
}

if [[ "${1:-}" == "--in-terminal" ]]; then
    [[ -f "$SESSION_SCRIPT" ]] || fail "recorder missing: $SESSION_SCRIPT"
    [[ -f /opt/ros/humble/setup.bash ]] || fail 'ROS Humble is missing at /opt/ros/humble/setup.bash'
    [[ -f "$REPO_ROOT/software/ros2/install/setup.bash" ]] \
        || fail "project ROS install is missing: $REPO_ROOT/software/ros2/install/setup.bash; prepare the car first"
    cd "$REPO_ROOT"
    export ROS_DOMAIN_ID=30
    echo "Manual maze capture: anchor=(3,0), heading=N"
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    echo "Drive the full maze with the joystick. Press Ctrl-C in this terminal to stop and save the session."
    echo "Closing this terminal also stops the capture; the cleanup watchdog finalizes the bag and summary."
    echo "This launcher records data only; it does not start joystick control or publish /cmd_vel."
    set +e
    bash "$SESSION_SCRIPT" --cell 3 0 --heading N --label joystick_full_maze
    result=$?
    set -e
    echo
    echo "Capture process ended (status $result). Check the session log and summary under: $REPO_ROOT/field_data/"
    hold_terminal
    exit "$result"
fi

command -v gnome-terminal >/dev/null 2>&1 || fail 'gnome-terminal is required to open the visible capture session'
gnome-terminal --wait --title="Joystick maze data capture" -- \
    bash -lc 'exec bash "$1" --in-terminal' _ "$LAUNCHER_SCRIPT" \
    || fail 'GNOME Terminal could not launch the capture window'
