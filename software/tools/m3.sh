#!/bin/bash
# ================================================================
#  M3Pro 小车 一键脚本 —— 在【车上 Ubuntu】运行
#
#    bash ~/m3.sh           查看状态
#    bash ~/m3.sh up        ★ 重启小车后跑这一条：自动修复代理 + 起雷达/TF + 相机
#                           （失败时自动打印原因和日志）
#    bash ~/m3.sh arm       机械臂回竖直
#    bash ~/m3.sh stop      停雷达 / 相机
#
#  远程用法（Mac 上）:  ssh car "bash ~/m3.sh up"
# ================================================================

export DISPLAY=:0
export ROS_DOMAIN_ID=30

source /opt/ros/humble/setup.bash
[ -f ~/yahboomcar_ws/install/setup.bash ] && source ~/yahboomcar_ws/install/setup.bash
[ -f ~/M3Pro_ws/install/setup.bash ] && source ~/M3Pro_ws/install/setup.bash
[ -f ~/mircoROS_agent/install/setup.bash ] && source ~/mircoROS_agent/install/setup.bash

LOG=/tmp/m3_log
mkdir -p "$LOG"

has() { ros2 topic list 2>/dev/null | grep -qx "$1"; }
yn()  { has "$1" && printf '✅' || printf '❌'; }
flow() { timeout 2 ros2 topic hz "$1" 2>/dev/null | grep -q "average rate"; }
agent_running() { pgrep -f micro_ros_agent > /dev/null; }

# 重启代理：官方 start_agent.sh 用 gnome-terminal 开窗口，日志看不到；
# 这里直接跑它里面那一行命令（domain 30 + serial /dev/myserial 2000000），日志落盘
start_agent() {
  pkill -f micro_ros_agent 2>/dev/null
  sleep 3
  nohup ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/myserial -b 2000000 \
    > "$LOG/agent.log" 2>&1 < /dev/null &
}

wait_agent() { for i in $(seq 1 $(($2/2))); do has /cmd_vel && return 0; sleep 2; done; return 1; }

dump_agent_diag() {
  echo "    ── 自动诊断（失败原因）──"
  echo "    代理进程: $(pgrep -af micro_ros_agent 2>/dev/null | head -2 | tr '\n' ' | ')"
  [ -z "$(pgrep -f micro_ros_agent 2>/dev/null)" ] && echo "    ⚠️ 代理进程已死 —— 上面日志最后一行就是死因"
  echo "    串口设备: $(ls -l /dev/myserial 2>/dev/null | awk '{print $9, $10, $11}')"
  echo "    代理日志尾部（$LOG/agent.log）:"
  tail -8 "$LOG/agent.log" 2>/dev/null | sed 's/^/      /'
}

case "${1:-check}" in

up)
  echo "[1/3] 通讯代理（上位机 ↔ 底层 STM32 的桥）"
  if has /cmd_vel && agent_running; then
    echo "      ✅ 正常，跳过"
  else
    [ -n "$(pgrep -f micro_ros_agent 2>/dev/null)" ] && echo "      ⚠️ 代理进程在但底板没挂上 → 杀掉重起"
    start_agent
    if wait_agent 60; then
      echo "      ✅ 底板挂上了"
    else
      echo "      ❌ 60 秒底板没挂上。失败原因："
      dump_agent_diag
      exit 1
    fi
  fi

  echo "[2/3] 雷达 + TF"
  if has /scan_multi; then
    echo "      ✅ 已在运行"
  else
    nohup ros2 launch yahboom_M3Pro_laser laser_driver.launch.py > "$LOG/laser.log" 2>&1 < /dev/null &
    for i in $(seq 1 15); do has /scan_multi && break; sleep 2; done
    has /scan_multi && echo "      ✅ 好了（/tf 一起出来）" || echo "      ❌ 看 $LOG/laser.log"
  fi

  echo "[3/3] 相机 + 机械臂解算"
  if has /camera/color/image_raw || has /rgb; then
    echo "      ✅ 已在运行"
  else
    nohup ros2 launch M3Pro_demo camera_arm_kin.launch.py > "$LOG/camera.log" 2>&1 < /dev/null &
    for i in $(seq 1 20); do has /camera/color/image_raw && break; sleep 2; done
    has /camera/color/image_raw && echo "      ✅ 好了" || echo "      ❌ 看 $LOG/camera.log"
  fi

  echo
  bash ~/m3.sh check
  ;;

check)
  SUB=$(ros2 topic info /arm6_joints 2>/dev/null | sed -n 's/.*Subscription count: \([0-9]*\).*/\1/p')
  echo
  echo "╔════════ M3Pro 小车状态 ════════╗"
  echo "  主机   $(hostname)   $(hostname -I 2>/dev/null | awk '{print $1}')"
  echo "  运行   $(uptime -p 2>/dev/null | sed 's/^up //')"
  echo "╠════════════════════════════════╣"

  echo "  [通讯代理 / 底层]   ← 上位机 ↔ 底层 STM32 的桥，它断则下面全断"
  if has /cmd_vel; then
    printf "    %-15s %s\n" "/cmd_vel"      "$(yn /cmd_vel)"
    printf "    %-15s %s\n" "/odom_raw"     "$(yn /odom_raw)"
    printf "    %-15s %s\n" "/imu/data_raw" "$(yn /imu/data_raw)"
    printf "    %-15s %s\n" "/battery"      "$(yn /battery)"
    printf "    %-15s %s\n" "/arm6_joints"  "$(yn /arm6_joints)"
  else
    echo "    ❌ 断了。自动诊断——"
    dump_agent_diag
  fi

  echo "  [机械臂]   底板订阅数 ${SUB:-0}   $([ "${SUB:-0}" = "1" ] && echo '✅ 可发角度指令' || echo '❌ 底板未挂上')"

  echo "  [雷达]"
  printf "    %-15s %s\n" "/scan0" "$(yn /scan0)"
  printf "    %-15s %s\n" "/scan1" "$(yn /scan1)"
  if has /scan_multi; then
    printf "    %-15s ✅ %s\n" "/scan_multi" "$(flow /scan_multi && echo '有数据流' || echo '⚠️ 话题在但无数据流')"
  else
    printf "    %-15s ❌\n" "/scan_multi"
  fi

  echo "  [TF]       /tf $(yn /tf)    /tf_static $(yn /tf_static)"

  cam=$(ros2 topic list 2>/dev/null | grep -iE 'rgb|camera|depth' | tr '\n' ' ')
  echo "  [相机]     ${cam:-❌ 没启动}"

  echo "╠════════════════════════════════╣"
  echo "  缺雷达/相机 → bash ~/m3.sh up     （只起这两个）"
  echo "  代理断了    → bash ~/m3.sh up     （自动杀旧起新 + 打印失败原因）"
  echo "  机械臂不直  → bash ~/m3.sh arm"
  echo "  日志        → $LOG/"
  echo "╚════════════════════════════════╝"
  ;;

arm)
  if ! has /arm6_joints; then
    echo "❌ /arm6_joints 不存在 → 代理没起来，先跑: bash ~/m3.sh up"
    exit 1
  fi
  echo "机械臂 → 竖直"
  ros2 topic pub /arm6_joints arm_msgs/msg/ArmJoints \
    "{joint1: 90, joint2: 90, joint3: 90, joint4: 90, joint5: 90, joint6: 180, time: 1500}" --once
  ;;

stop)
  pkill -f laser_driver.launch.py 2>/dev/null
  pkill -f camera_arm_kin 2>/dev/null
  echo "已停雷达 / 相机"
  ;;

*)
  echo "用法: bash ~/m3.sh {up|check|arm|stop}"
  ;;
esac
