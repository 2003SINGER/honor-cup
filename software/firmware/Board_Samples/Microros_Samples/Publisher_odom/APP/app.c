#include "app.h"
#include "app_motion.h"
#include "app_encoder.h"
#include "app_pid.h"
#include "app_motor.h"
#include "usart.h"
#include "ros2_handle.h"


#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"


// LED显示当前运行状态，每10毫秒调用一次，LED灯每200毫秒闪烁一次。
// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.
static void App_Led_Mcu_Flash_200ms(void)
{
	static uint8_t led_count = 0;
	led_count++;
	if (led_count > 20)
	{
		led_count = 0;
		LED_MCU_TOGGLE();
	}
}


void Car_Handle(void)
{
    uint32_t lastWakeTime;
    osDelay(500);
    Motor_Init();
	Encoder_Init();
	PID_Param_Init();

    lastWakeTime = xTaskGetTickCount();
    while (1)
	{
        Motion_Handle();
		vTaskDelayUntil(&lastWakeTime, 10);
	}
}


void App_Handle(void)
{
    uint32_t lastWakeTime = xTaskGetTickCount();

    printf("start APP Handle\n");
    while (1)
	{
		App_Led_Mcu_Flash_200ms();
		vTaskDelayUntil(&lastWakeTime, 10);
	}
    
}

