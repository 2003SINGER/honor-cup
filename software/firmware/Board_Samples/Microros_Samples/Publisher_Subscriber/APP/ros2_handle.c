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


#include <std_msgs/msg/int32.h>

#include "app.h"



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


rcl_publisher_t publisher_1;
rcl_publisher_t publisher_2;
rcl_publisher_t publisher_3;
std_msgs__msg__Int32 publisher_msg_1;
std_msgs__msg__Int32 publisher_msg_2;
std_msgs__msg__Int32 publisher_msg_3;
rcl_timer_t publisher_timer_1;
rcl_timer_t publisher_timer_2;
rcl_timer_t publisher_timer_3;


rcl_subscription_t subscriber_1;
rcl_subscription_t subscriber_2;
rcl_subscription_t subscriber_3;
std_msgs__msg__Int32 subscriber_msg_1;
std_msgs__msg__Int32 subscriber_msg_2;
std_msgs__msg__Int32 subscriber_msg_3;

rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;


static uint8_t ros2_domain_id = DEF_ROS_DOMAIN_ID;
static char ros2_namespace[ROS_NAMESPACE_LEN_MAX] = DEF_ROS_NAMESPACE;
static int executor_count = 0;

int32_t set_microros_serial_transports(void);
int32_t set_microros_serial_transports_with_options(rmw_init_options_t * rmw_options);


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



void publisher_callback_1(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        printf("Publishing_1: %d\n", (int) publisher_msg_1.data);
		RCSOFTCHECK(rcl_publish(&publisher_1, &publisher_msg_1, NULL));
		publisher_msg_1.data++;
    }
}
void publisher_callback_2(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        printf("Publishing_2: %d\n", (int) publisher_msg_2.data);
		RCSOFTCHECK(rcl_publish(&publisher_2, &publisher_msg_2, NULL));
		publisher_msg_2.data++;
    }
}
void publisher_callback_3(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        printf("Publishing_3: %d\n", (int) publisher_msg_3.data);
		RCSOFTCHECK(rcl_publish(&publisher_3, &publisher_msg_3, NULL));
		publisher_msg_3.data++;
    }
}


void subscriber_callback_1(const void *msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int32_t msg_data = msg->data;
    printf("subscriber_1 data:%ld\n", msg_data);
}
void subscriber_callback_2(const void *msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int32_t msg_data = msg->data;
    printf("subscriber_2 data:%ld\n", msg_data);
}
void subscriber_callback_3(const void *msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int32_t msg_data = msg->data;
    printf("subscriber_3 data:%ld\n", msg_data);
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

    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "YB_Example_Node", (char*)ros2_namespace, &support));

    
    executor_count++;
    // 创建发布者
    RCCHECK(rclc_publisher_init_default(
        &publisher_1,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int32_publisher_1"));
    
    executor_count++;
    // 创建发布者
    RCCHECK(rclc_publisher_init_default(
        &publisher_2,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int32_publisher_2"));
    
    executor_count++;
    // 创建发布者
    RCCHECK(rclc_publisher_init_default(
        &publisher_3,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int32_publisher_3"));
    
    // 创建定时器
    RCCHECK(rclc_timer_init_default(
        &publisher_timer_1,
        &support,
        RCL_MS_TO_NS(PUBLISHER_TIMEOUT_1),
        publisher_callback_1));

    RCCHECK(rclc_timer_init_default(
        &publisher_timer_2,
        &support,
        RCL_MS_TO_NS(PUBLISHER_TIMEOUT_2),
        publisher_callback_2));

    RCCHECK(rclc_timer_init_default(
        &publisher_timer_3,
        &support,
        RCL_MS_TO_NS(PUBLISHER_TIMEOUT_3),
        publisher_callback_3));

    // 创建订阅者
    executor_count++;
    RCCHECK(rclc_subscription_init_default(
        &subscriber_1,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int32_subscriber_1"));
    executor_count++;
    RCCHECK(rclc_subscription_init_default(
        &subscriber_2,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int32_subscriber_2"));
    executor_count++;
    RCCHECK(rclc_subscription_init_default(
        &subscriber_3,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "int32_subscriber_3"));

    printf("executor_count:%d\n", executor_count);
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, executor_count, &allocator));

    // 向执行器添加计时器
    RCCHECK(rclc_executor_add_timer(&executor, &publisher_timer_1));
    RCCHECK(rclc_executor_add_timer(&executor, &publisher_timer_2));
    RCCHECK(rclc_executor_add_timer(&executor, &publisher_timer_3));

    // 向执行器添加订阅者
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &subscriber_1,
        &subscriber_msg_1,
        &subscriber_callback_1,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &subscriber_2,
        &subscriber_msg_2,
        &subscriber_callback_2,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &subscriber_3,
        &subscriber_msg_3,
        &subscriber_callback_3,
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

    // ROS代理结束
    printf("ROS Task End\n");
    printf("Restart System!!!\n");
    vTaskDelay(pdMS_TO_TICKS(10));
    HAL_NVIC_SystemReset();
}
