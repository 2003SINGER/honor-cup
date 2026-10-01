#ifndef ROS2_HANDLE_H_
#define ROS2_HANDLE_H_

#include "stdint.h"
#include "stdio.h"



// 定时器间隔时间，单位:ms, 频率=1000/timeout
#define PUBLISHER_TIMEOUT_1            (1000)
#define PUBLISHER_TIMEOUT_2            (800)
#define PUBLISHER_TIMEOUT_3            (500)

// ROS运行超时时间
#define ROS2_SPIN_TIMEOUT_MS         (10)


#define ROS_NAMESPACE_LEN_MAX           (10)
#define DEF_ROS_NAMESPACE               ""
#define DEF_ROS_DOMAIN_ID               (30)


void Ros2_Handle_Task(void);


#endif /* ROS2_HANDLE_H_ */
