#include "ros2_handle.h"

#include "string.h"
#include "stdlib.h"
#include "math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"



#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>


#include <micro_ros_utilities/string_utilities.h>
#include <geometry_msgs/msg/twist.h>


#include "app.h"
#include "app_motion.h"



int ros_error = 0;


#define RCCHECK(fn)                                                                  \
{                                                                                    \
    rcl_ret_t temp_rc = fn;                                                          \
    if ((temp_rc != RCL_RET_OK))                                                     \
    {                                                                                \
        printf("Error on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);         \
        ros_error = 5;                                                               \
        vTaskDelete(NULL);                                                           \
    }                                                                                \
}


#define RCSOFTCHECK(fn)                                                                \
{                                                                                      \
    rcl_ret_t temp_rc = fn;                                                            \
    if ((temp_rc != RCL_RET_OK))                                                       \
    {                                                                                  \
        ros_error++;                                                                   \
        printf("Failed on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);        \
    }                                                                                  \
    else                                                                               \
    {                                                                                  \
        ros_error = 0;                                                                 \
    }                                                                                  \
}


rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;


int64_t time_offset = 0;
uint8_t executor_count = 0;

static uint8_t ros2_domain_id = DEF_ROS_DOMAIN_ID;
static char ros2_namespace[ROS_NAMESPACE_LEN_MAX] = DEF_ROS_NAMESPACE;

float x_pos_ = 0.0;
float y_pos_ = 0.0;
float heading_ = 0.0;


int32_t set_microros_serial_transports(void);
int32_t set_microros_serial_transports_with_options(rmw_init_options_t * rmw_options);


// 返回系统开机到当前的时间戳，单位为毫秒（ms）
int64_t get_millisecond(void)
{
    return HAL_GetTick();
}


static void Ros2_Sync_Time(void)
{
    int64_t now = get_millisecond();
    uint8_t count_sync = 0;
    while (count_sync < ROS2_TIME_SYNC_ERROR_COUNT)
    {
        // 同步时间
        rmw_uros_sync_session(ROS2_TIME_SYNC_TIMEOUT);
        count_sync++;
        if (rmw_uros_epoch_synchronized())
        {
            count_sync = ROS2_TIME_SYNC_ERROR_COUNT;
        }
    }
    // 判断时间是否同步
    if (!rmw_uros_epoch_synchronized())
    {
        // 同步时间, 如果失败退出ROS任务
        RCCHECK(rmw_uros_sync_session(ROS2_TIME_SYNC_TIMEOUT));
    }
    int64_t ros_time_ms = rmw_uros_epoch_millis();
    now = get_millisecond();
    // ROS_agent和MCU的时间差
    time_offset = ros_time_ms - now;
}

timespec_t get_ros2_timestamp(void)
{
    timespec_t tp = {0, 0};
    //同步时间
    int64_t now = get_millisecond() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


int set_microros_freeRTOS_allocator(void)
{
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;
    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
        return -1;
    }
    return 0;
}

int32_t set_microros_serial_transports(void)
{
    int32_t ret = 0;
    ret = rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart1,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);
    return ret;
}


int32_t set_microros_serial_transports_with_options(rmw_init_options_t * rmw_options)
{
    int32_t ret = 0;
    ret = rmw_uros_options_set_custom_transport(
        true,
	    (void *) &huart1,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read,
        rmw_options
    );
    return ret;
}

int32_t ping_microros_agent(void)
{
    static int ping_count = 0;
    ping_count++;
    if(ping_count > 100)
    {
        ping_count = 0;
        return rmw_uros_ping_agent(100, 1);
    }
    return RMW_RET_OK;
}


void twist_Callback(const void *msgin)
{
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist *)msgin;
    printf("cmd_vel:%.2f, %.2f, %.2f\n", msg->linear.x, msg->linear.y, msg->angular.z);
    Motion_Ctrl_Car(msg->linear.x, msg->linear.y, msg->angular.z);
}


void Ros2_Handle_Task(void)
{
    int state = 0;
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ros2_domain_id));
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    osDelay(50);
    printf("ROS Domain ID:%d\n", ros2_domain_id);
    set_microros_serial_transports_with_options(rmw_options);
    set_microros_freeRTOS_allocator();
    while (1)
    {
        osDelay(500);
        state = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        if (state == RCL_RET_OK) break;
        printf("Reconnecting agent...\n");
    }
    
    printf("Start YB_Example_Node\n");
    Ros2_Sync_Time();

    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "YB_Example_Node", (char*)ros2_namespace, &support));


    executor_count++;
    // 创建订阅者 cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    printf("executor_count:%d\n", executor_count);
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, executor_count, &allocator));

    // 向执行器添加订阅者twist
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twist_Callback,
        ON_NEW_DATA));

    LED_ROS_ON();
    uint32_t lastWakeTime = xTaskGetTickCount();
    while (ros_error < 3)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ROS2_SPIN_TIMEOUT_MS));
        if (ping_microros_agent() != RMW_RET_OK) break;
        vTaskDelayUntil(&lastWakeTime, 10);
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
    LED_ROS_OFF();
    Motion_Stop(STOP_FREE);
    // ROS代理结束
    printf("ROS Task End\n");
    printf("Restart System!!!\n");
    vTaskDelay(pdMS_TO_TICKS(10));
    HAL_NVIC_SystemReset();
}
