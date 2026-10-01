#ifndef ROS2_HANDLE_H_
#define ROS2_HANDLE_H_

#include "stdint.h"
#include "stdio.h"



// 定时器间隔时间，单位:ms, 频率=1000/timeout
#define ODOM_PUBLISHER_TIMEOUT       (90)

// ROS运行超时时间
#define ROS2_SPIN_TIMEOUT_MS         (10)

// ROS时间同步错误次数
#define ROS2_TIME_SYNC_ERROR_COUNT   (5)
// ROS时间同步超时ms
#define ROS2_TIME_SYNC_TIMEOUT       (1000)

#define ENABLE_ROS2_MOTION_IMU       (0)
#define ENABLE_ROS_SERIAL_TIMEOUT    (0)
#define ENABLE_ROS2_CHECK_ERROR      (0)


#define ROS_NAMESPACE_LEN_MAX           (10)
#define DEF_ROS_NAMESPACE               ""
#define DEF_ROS_DOMAIN_ID               (30)



typedef enum agent_state {
    AGENT_DISCONNECTED = 0,
    AGENT_CONNECTED = 1,
    AGENT_STATE_MAX
} agent_state_t;

typedef enum agent_transport
{
    AGENT_WIFI_UDP = 0,
    AGENT_SERIAL = 1,
} agent_transport_t;


typedef struct _timespec {
	int64_t	tv_sec;		/* seconds */
	long	tv_nsec;	/* and nanoseconds */
} timespec_t;


int64_t get_millisecond(void);
void Ros2_Handle_Task(void);


#endif /* ROS2_HANDLE_H_ */
