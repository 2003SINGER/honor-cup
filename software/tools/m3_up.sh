#!/bin/bash
# ================================================================
#  M3Pro 开机自启服务 —— 登录桌面后由 autostart 自动调用
#  幂等：已在跑的服务自动跳过；代理没起来会先等它（雷达数据经代理）
# ================================================================

export DISPLAY=:0
export ROS_DOMAIN_ID=30

source /opt/ros/humble/setup.bash
[ -f ~/yahboomcar_ws/install/setup.bash ] && source ~/yahboomcar_ws/install/setup.bash
[ -f ~/M3Pro_ws/install/setup.bash ] && source ~/M3Pro_ws/install/setup.bash

mkdir -p /tmp/m3_log

has() { ros2 topic list 2>/dev/null | grep -qx "$1"; }

# ---- 0. 等通讯代理起来（雷达数据经它转发，最多等 2 分钟）----
for i in $(seq 1 60); do has /cmd_vel && break; sleep 2; done

# ---- 1. 雷达 + TF ----
if has /scan_multi; then
  echo "$(date '+%F %T') laser: 已在运行" >> /tmp/m3_log/boot.log
else
  nohup ros2 launch yahboom_M3Pro_laser laser_driver.launch.py > /tmp/m3_log/laser.log 2>&1 &
  echo "$(date '+%F %T') laser: 启动" >> /tmp/m3_log/boot.log
fi

# ---- 2. 相机 + 机械臂解算 ----
if has /camera/color/image_raw || has /rgb; then
  echo "$(date '+%F %T') camera: 已在运行" >> /tmp/m3_log/boot.log
else
  nohup ros2 launch M3Pro_demo camera_arm_kin.launch.py > /tmp/m3_log/camera.log 2>&1 &
  echo "$(date '+%F %T') camera: 启动" >> /tmp/m3_log/boot.log
fi

# ---- 2.5 IMU 滤波（/imu/data_raw → /imu/data，EKF 融合的输入之一）----
if ! pgrep -f imu_filter_madgwick > /dev/null; then
  nohup ros2 launch imu_filter_madgwick imu_filter.launch.py > /tmp/m3_log/imu.log 2>&1 &
  echo "$(date '+%F %T') imu_filter: 启动" >> /tmp/m3_log/boot.log
fi

# ---- 3. URDF 静态 TF（base_link / imu_frame 等坐标系）----
if ! pgrep -f robot_state_publisher > /dev/null; then
  nohup ros2 launch M3Pro display.launch.py > /tmp/m3_log/display.log 2>&1 &
  echo "$(date '+%F %T') display: 启动" >> /tmp/m3_log/boot.log
fi

# ---- 4. EKF（发 odom→base_footprint TF + /odom 融合里程计）----
if ! pgrep -f ekf_filter_node > /dev/null; then
  nohup ros2 launch ekf_bringup ekf.launch.py > /tmp/m3_log/ekf.log 2>&1 &
  echo "$(date '+%F %T') ekf: 启动" >> /tmp/m3_log/boot.log
fi

sleep 8   # 给 TF/EKF 一点时间
