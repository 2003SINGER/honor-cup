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
export ROS_DOMAIN_ID=30    # 车的 domain（check_sensor.desktop 里读到的），SSH 非交互 shell 不读 .bashrc

source /opt/ros/humble/setup.bash
[ -f ~/yahboomcar_ws/install/setup.bash ] && source ~/yahboomcar_ws/install/setup.bash
[ -f ~/M3Pro_ws/install/setup.bash ] && source ~/M3Pro_ws/install/setup.bash

has() { ros2 topic list 2>/dev/null | grep -qx "$1"; }
yn()  { has "$1" && printf '✅' || printf '❌'; }
flow() { timeout 2 ros2 topic hz "$1" 2>/dev/null | grep -q "average rate"; }
waitfor() { for i in $(seq 1 "$2"); do has "$1" && return 0; sleep 1; done; return 1; }

case "${1:-check}" in

up)
  echo "[1/3] 通讯代理（开机自启 —— 这里只检查，绝不重复启动）"
  if has /cmd_vel; then
    echo "      ✅ 正常"
  else
    echo "      ❌ 断了。不重复启动：两个代理会抢同一个串口，只会越搞越坏。"
    echo "         先跑  bash ~/m3.sh check  看自动诊断结果"
    exit 1
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
    echo "      代理进程  $(pgrep -af micro_ros_agent 2>/dev/null | head -2 | tr '\n' '|')"
    echo "      会话日志  $(tail -3 /tmp/m3_agent.log 2>/dev/null | tr '\n' '|')"
    echo "      串口设备  $(ls /dev/myserial /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
    echo "      自启项    $(ls ~/.config/autostart/ 2>/dev/null | tr '\n' ' ')"
    echo "      ---- start_agent.sh 的内容 ----"
    sed 's/^/       /' ~/start_agent.sh 2>/dev/null | head -30
    echo "      --------------------------------"
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
  echo "  代理断了    → 上面已自动诊断，贴出来看"
  echo "  机械臂不直  → bash ~/m3.sh arm"
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
