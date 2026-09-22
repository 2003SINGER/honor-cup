#!/bin/bash
# ============================================================
#  M3Pro 小车  一键启动 / 状态检查
#  用法:   bash ~/car.sh up      启动全部（代理 + 雷达/TF + 相机）
#          bash ~/car.sh check   检查状态（默认）
#          bash ~/car.sh arm     机械臂回竖直
#          bash ~/car.sh stop    停止
#  说明:   SSH 下也能用；脚本自己处理 DISPLAY，日志在 ~/m3pro_log/
# ============================================================

WS1="$HOME/yahboomcar_ws"
WS2="$HOME/M3Pro_ws"
LOG="$HOME/m3pro_log"
export DISPLAY=:0          # SSH 下 start_agent.sh 需要，否则报 Cannot open display

src_env() {
  source /opt/ros/humble/setup.bash
  [ -f "$WS1/install/setup.bash" ] && source "$WS1/install/setup.bash"
  [ -f "$WS2/install/setup.bash" ] && source "$WS2/install/setup.bash"
}

has() { ros2 topic list 2>/dev/null | grep -qx "$1"; }

waitfor() {                # waitfor <话题> <最多等几秒>
  for i in $(seq 1 "$2"); do has "$1" && return 0; sleep 1; done
  return 1
}

mark() { if has "$1"; then printf '✅%s  ' "$1"; else printf '❌%s  ' "$1"; fi; }

src_env
mkdir -p "$LOG"

case "${1:-check}" in

up)
  echo "[1/3] 通讯代理（上位机 ↔ 底层 STM32 的桥）"
  if has /cmd_vel; then
    echo "      已在运行，跳过"
  else
    nohup bash "$HOME/start_agent.sh" > "$LOG/agent.log" 2>&1 &
    if waitfor /cmd_vel 30; then echo "      ✅ 起来了"
    else echo "      ❌ 30 秒没起来，看 $LOG/agent.log"; exit 1; fi
  fi

  echo "[2/3] 雷达 + TF"
  if has /scan_multi; then
    echo "      已在运行，跳过"
  else
    nohup ros2 launch yahboom_M3Pro_laser laser_driver.launch.py > "$LOG/laser.log" 2>&1 &
    if waitfor /scan_multi 30; then echo "      ✅ 起来了（/tf 一起出来了）"
    else echo "      ❌ 看 $LOG/laser.log"; fi
  fi

  echo "[3/3] 相机 + 机械臂解算"
  if has /rgb; then
    echo "      已在运行，跳过"
  else
    nohup ros2 launch M3Pro_demo camera_arm_kin.launch.py > "$LOG/camera.log" 2>&1 &
    for i in $(seq 1 30); do
      ros2 topic list 2>/dev/null | grep -qiE "camera|rgb|depth" && break; sleep 1
    done
    echo "      已启动（相机话题见 check）"
  fi

  echo
  echo "启动流程结束。看状态： bash ~/car.sh check"
  echo "机械臂竖直： bash ~/car.sh arm"
  ;;

check)
  echo "=============== M3Pro 状态 ==============="
  echo -n "[代理/底层] "
  for t in /cmd_vel /odom_raw /imu/data_raw /battery /arm6_joints; do mark "$t"; done
  echo

  echo -n "[机械臂订阅] "
  if has /arm6_joints; then
    ros2 topic info /arm6_joints 2>/dev/null | grep "Subscription count" | sed 's/^ *//'
  else
    echo "❌ /arm6_joints 不存在 → 代理没起来，先 bash ~/car.sh up"
  fi

  echo -n "[雷达/TF]   "
  for t in /scan0 /scan1 /scan_multi /tf /tf_static; do mark "$t"; done
  echo

  echo -n "[相机]      "
  cam=$(ros2 topic list 2>/dev/null | grep -iE "camera|rgb|depth" | tr '\n' ' ')
  if [ -n "$cam" ]; then echo "$cam"; else echo "❌ 没有相机话题（没启动或没起来）"; fi
  echo "=========================================="
  echo "PS: 只有 ✅/❌ 不够时，日志在 $LOG/"
  ;;

arm)
  if ! has /arm6_joints; then
    echo "❌ /arm6_joints 不存在 —— 代理没起来，先 bash ~/car.sh up"
    exit 1
  fi
  echo "机械臂 → 竖直"
  ros2 topic pub /arm6_joints arm_msgs/msg/ArmJoints \
    "{joint1: 90, joint2: 90, joint3: 90, joint4: 90, joint5: 90, joint6: 180, time: 1500}" --once
  ;;

stop)
  pkill -f laser_driver.launch.py 2>/dev/null
  pkill -f camera_arm_kin 2>/dev/null
  echo "已停雷达/相机。（代理是开机自启的，不建议手动停；要彻底复位就重启小车）"
  ;;

*)
  echo "用法: bash ~/car.sh {up|check|arm|stop}"
  ;;
esac
