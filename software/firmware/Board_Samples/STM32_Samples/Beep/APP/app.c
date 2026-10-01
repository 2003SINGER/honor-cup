#include "app.h"
#include "app_key.h"
#include "app_beep.h"

int led_ros_state = 0;
int led_ros_count = 0;

// LED显示当前运行状态，每10毫秒调用一次，LED灯每200毫秒闪烁一次。
// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.
void App_Led_Mcu_Handle(void)
{
	static uint8_t led_count = 0;
	led_count++;
	if (led_count >= 20)
	{
		led_count = 0;
		LED_MCU_TOGGLE();
	}
}



// main.c中调用此函数，避免多次修改main.c文件。
// This function is called in main.c to avoid multiple modifications to the main.c file
void App_Handle(void)
{

	while (1)
	{
        if (Key1_State())
        {
            Beep_On_Time(100);
        }
        Beep_Handle();
		App_Led_Mcu_Handle();
		HAL_Delay(10);
	}
}
