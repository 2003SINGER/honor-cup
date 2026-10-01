#!/usr/bin/env bash
# Prepare the robot's ROS runtime from the Mac. This starts sensor/runtime
# services but never publishes a motion command or creates field_data.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PACKAGE_DIR="$REPO_ROOT/software/ros2/m3pro_nav"
CAR_HOST="${CAR_SSH_HOST:-${1:-}}"
CAR_REPO_PATH="${CAR_REPO_PATH:-}"
MAX_CLOCK_SKEW="${MAX_CLOCK_SKEW_SECONDS:-3}"

die() { echo "[prepare_field_car] ERROR: $*" >&2; exit 1; }
[[ -n "$CAR_HOST" ]] || die 'set CAR_SSH_HOST (SSH config alias) or pass host as first argument'
[[ "$MAX_CLOCK_SKEW" =~ ^[0-9]+$ ]] || die 'MAX_CLOCK_SKEW_SECONDS must be a nonnegative integer'
command -v ssh >/dev/null 2>&1 || die 'ssh not found'
command -v rsync >/dev/null 2>&1 || die 'rsync not found on Mac'
[[ -f "$PACKAGE_DIR/setup.py" ]] || die "ROS package not found: $PACKAGE_DIR"
SOURCE_SHA="$(cd "$REPO_ROOT" && git rev-parse HEAD)" \
    || die 'could not determine the Mac repository commit SHA'

echo "[prepare_field_car] checking SSH access to $CAR_HOST"
if [[ -z "$CAR_REPO_PATH" ]]; then
    CAR_REPO_PATH="$(ssh -o BatchMode=yes "$CAR_HOST" 'printf %s "$HOME/honor-cup"')" \
        || die 'SSH failed; power on the car and make sure the configured host is reachable'
fi
[[ "$CAR_REPO_PATH" == /* ]] || die 'CAR_REPO_PATH must be an absolute path on the car'

# Read-only clock comparison. Fail closed because stale ROS timestamps can
# invalidate TF/odom observations. No clock adjustment is attempted here.
MAC_EPOCH="$(date +%s)"
CAR_EPOCH="$(ssh -o BatchMode=yes "$CAR_HOST" 'date +%s')" \
    || die 'could not read the car clock over SSH'
[[ "$CAR_EPOCH" =~ ^[0-9]+$ ]] || die "invalid car epoch: $CAR_EPOCH"
SKEW=$(( MAC_EPOCH > CAR_EPOCH ? MAC_EPOCH - CAR_EPOCH : CAR_EPOCH - MAC_EPOCH ))
echo "[prepare_field_car] Mac/car clock skew: ${SKEW}s (limit ${MAX_CLOCK_SKEW}s)"
(( SKEW <= MAX_CLOCK_SKEW )) || die "clock skew exceeds limit; sync the car clock to network time or set it from the Mac, then rerun"

echo "[prepare_field_car] syncing ROS package to $CAR_HOST:$CAR_REPO_PATH/software/ros2/m3pro_nav"
ssh -o BatchMode=yes "$CAR_HOST" "mkdir -p '$CAR_REPO_PATH/software/ros2/m3pro_nav'"
rsync -a --exclude='__pycache__/' --exclude='*.pyc' \
    "$PACKAGE_DIR/" "$CAR_HOST:$CAR_REPO_PATH/software/ros2/m3pro_nav/"

# The session script records bags and metadata on the car, where ROS is running.
ssh -o BatchMode=yes "$CAR_HOST" "mkdir -p '$CAR_REPO_PATH/software/scripts' '$CAR_REPO_PATH/software/tools'"
rsync -a "$REPO_ROOT/software/scripts/field_scan_session.sh" \
    "$CAR_HOST:$CAR_REPO_PATH/software/scripts/field_scan_session.sh"
rsync -a "$REPO_ROOT/software/tools/scan_session_summary.py" \
    "$CAR_HOST:$CAR_REPO_PATH/software/tools/scan_session_summary.py"
rsync -a "$REPO_ROOT/software/tools/m3.sh" "$CAR_HOST:/tmp/codex-m3.sh"
ssh -o BatchMode=yes "$CAR_HOST" 'install -m 755 /tmp/codex-m3.sh "$HOME/m3.sh" && rm -f /tmp/codex-m3.sh'

ssh -o BatchMode=yes "$CAR_HOST" bash -s -- "$CAR_REPO_PATH" "$SOURCE_SHA" <<'REMOTE'
set -euo pipefail
repo="$1"
source_sha="$2"
cd "$repo/software/ros2"
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=30
colcon build --packages-select m3pro_nav --symlink-install
printf '%s\n' "$source_sha" > "$repo/software/ros2/install/deployed_git_sha"
set +u
source install/setup.bash
set -u
ros2 run m3pro_nav control_probe --help >/dev/null
echo '[prepare_field_car] installed entrypoint: ros2 run m3pro_nav control_probe --help OK (no ROS node or motion started)'

# m3.sh up starts services only. Its implementation contains no /cmd_vel
# publisher; it does start the micro-ROS agent, laser, camera, IMU filter and EKF.
bash "$HOME/m3.sh" up

set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/yahboomcar_ws/install/setup.bash" ] && source "$HOME/yahboomcar_ws/install/setup.bash"
[ -f "$HOME/M3Pro_ws/install/setup.bash" ] && source "$HOME/M3Pro_ws/install/setup.bash"
source "$repo/software/ros2/install/setup.bash"
set -u
export ROS_DOMAIN_ID=30
for topic in /scan_multi /odom_raw; do
  timeout 8 ros2 topic list | grep -Fxq "$topic" || { echo "[prepare_field_car] missing required topic: $topic" >&2; exit 1; }
done
type="$(timeout 8 ros2 topic type /odom_raw)"
[ "$type" = nav_msgs/msg/Odometry ] || { echo "[prepare_field_car] unexpected /odom_raw type: $type" >&2; exit 1; }
base="$(timeout 4 ros2 topic echo --once --field child_frame_id /odom_raw 2>/dev/null | sed -n '1p')"
[ "$base" = base_footprint ] || { echo "[prepare_field_car] expected /odom_raw child_frame_id=base_footprint, got '${base:-unread}'" >&2; exit 1; }
echo '[prepare_field_car] required drivers/topics ready; odom base frame is base_footprint'
REMOTE

echo "[prepare_field_car] ready. Run the recorder on the car: bash '$CAR_REPO_PATH/software/scripts/field_scan_session.sh' --cell X Y --heading N --label LABEL"
echo "[prepare_field_car] after recording, pull from the Mac with: rsync -a '$CAR_HOST:$CAR_REPO_PATH/field_data/<session_dir>/' './field_data/<session_dir>/'"
