#include "app.h"
#include "app_motion.h"
#include "app_encoder.h"
#include "app_pid.h"
#include "app_motor.h"
#include "usart.h"



// LED显示当前运行状态，每10毫秒调用一次，LED灯每200毫秒闪烁一次。
// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.
void App_Led_Mcu_Handle(void)
{
	static uint8_t led_count = 0;
	led_count++;
	if (led_count > 20)
	{
		led_count = 0;
		LED_MCU_TOGGLE();
	}
}


void App_Handle(void)
{
    Motor_Init();
	Encoder_Init();
	PID_Param_Init();

    HAL_Delay(1000);
    Motion_Set_Speed(300, 300, 300, 300);

    while (1)
	{
        App_Led_Mcu_Handle();
        Motion_Handle();
        HAL_Delay(10);
	}
}

