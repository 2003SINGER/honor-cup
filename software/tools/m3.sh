#!/bin/bash
# ================================================================
#  M3Pro 小车 一键脚本   —— 在【车上 Ubuntu】运行
#
#  用法（在车上）:
#      bash ~/m3.sh up      启动：代理 + 雷达/TF + 相机
#      bash ~/m3.sh check   看状态
#      bash ~/m3.sh arm     机械臂回竖直
#      bash ~/m3.sh stop    停雷达 / 相机
#
#  用法（在 Mac 上，ssh 远程开）:
#      ssh car "bash ~/m3.sh up"
#      ssh car "bash ~/m3.sh check"
# ================================================================

export DISPLAY=:0          # SSH 下 start_agent.sh 需要，否则报 Cannot open display

source /opt/ros/humble/setup.bash
[ -f ~/yahboomcar_ws/install/setup.bash ] && source ~/yahboomcar_ws/install/setup.bash
[ -f ~/M3Pro_ws/install/setup.bash ] && source ~/M3Pro_ws/install/setup.bash

has() { ros2 topic list 2>/dev/null | grep -qx "$1"; }
yn()  { has "$1" && printf '✅' || printf '❌'; }
waitfor() { for i in $(seq 1 "$2"); do has "$1" && return 0; sleep 1; done; return 1; }

case "${1:-check}" in

up)
  echo "[1/3] 通讯代理（上位机 ↔ 底层 STM32 的桥）"
  if has /cmd_vel; then
    echo "      已在运行，跳过"
  else
    nohup bash ~/start_agent.sh > /tmp/m3_agent.log 2>&1 &
    waitfor /cmd_vel 30 && echo "      ✅ 好了" || echo "      ❌ 30 秒没起来，看 /tmp/m3_agent.log"
  fi

  echo "[2/3] 雷达 + TF"
  if has /scan_multi; then
    echo "      已在运行，跳过"
  else
    nohup ros2 launch yahboom_M3Pro_laser laser_driver.launch.py > /tmp/m3_laser.log 2>&1 &
    waitfor /scan_multi 30 && echo "      ✅ 好了（/tf 一起出来）" || echo "      ❌ 看 /tmp/m3_laser.log"
  fi

  echo "[3/3] 相机 + 机械臂解算"
  if has /rgb; then
    echo "      已在运行，跳过"
  else
    nohup ros2 launch M3Pro_demo camera_arm_kin.launch.py > /tmp/m3_cam.log 2>&1 &
    for i in $(seq 1 30); do
      ros2 topic list 2>/dev/null | grep -qiE 'rgb|camera|depth' && break; sleep 1
    done
    echo "      已启动"
  fi

  echo
  bash ~/m3.sh check
  ;;

check)
  echo "=========== M3Pro 状态 ==========="
  echo "代理/底层  /cmd_vel $(yn /cmd_vel)   /odom_raw $(yn /odom_raw)   /imu/data_raw $(yn /imu/data_raw)   /battery $(yn /battery)   /arm6_joints $(yn /arm6_joints)"
  echo "机械臂订阅 $(ros2 topic info /arm6_joints 2>/dev/null | sed -n 's/.*Subscription count: \([0-9]*\).*/\1/p')"
  echo "雷达       /scan0 $(yn /scan0)   /scan1 $(yn /scan1)   /scan_multi $(yn /scan_multi)"
  echo "TF         /tf $(yn /tf)   /tf_static $(yn /tf_static)"
  echo -n "相机       "
  cam=$(ros2 topic list 2>/dev/null | grep -iE 'rgb|camera|depth' | tr '\n' ' ')
  [ -n "$cam" ] && echo "$cam" || echo "❌ 没启动"
  echo "日志       /tmp/m3_*.log"
  echo "=================================="
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
