#include "app.h"
#include "app_sbus.h"


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
    SBUS_Init();

    while (1)
	{
        SBUS_Handle();
		App_Led_Mcu_Handle();
		HAL_Delay(10);
	}
}


