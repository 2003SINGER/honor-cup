#ifndef ROS2_HANDLE_H_
#define ROS2_HANDLE_H_

#include "stdint.h"
#include "stdio.h"




// ROS运行超时时间
#define ROS2_SPIN_TIMEOUT_MS         (10)


#define ROS_NAMESPACE_LEN_MAX           (10)
#define DEF_ROS_NAMESPACE               ""
#define DEF_ROS_DOMAIN_ID               (30)


void Ros2_Handle_Task(void);


#endif /* ROS2_HANDLE_H_ */
