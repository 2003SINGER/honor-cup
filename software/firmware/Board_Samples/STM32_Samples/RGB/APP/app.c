#include "app.h"
#include "app_rgb.h"
#include "stdio.h"



int rgb_count = 0;
int rgb_color = 0;

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
    RGB_Init();

	while (1)
	{
		rgb_count++;
		if (rgb_count > 100)
		{
			rgb_count = 0;
			rgb_color = (rgb_color + 1) % 5;
            printf("color:%d\n", rgb_color);
			if (rgb_color == 0)
			{
				RGB_Clear();
				RGB_Update();
			}
			else if (rgb_color == 1)
			{
				RGB_Clear();
				RGB_Set_Color(RGB_CTRL_ALL, 0xFF, 0x00, 0x00);
				RGB_Update();
			}
			else if (rgb_color == 2)
			{
				RGB_Clear();
				RGB_Set_Color(RGB_CTRL_ALL, 0x00, 0xFF, 0x00);
				RGB_Update();
			}
			else if (rgb_color == 3)
			{
				RGB_Clear();
				RGB_Set_Color(RGB_CTRL_ALL, 0x00, 0x00, 0xFF);
				RGB_Update();
			}
			else if (rgb_color == 4)
			{
				RGB_Clear();
				RGB_Set_Color(RGB_CTRL_ALL, 0xFF, 0xFF, 0xFF);
				RGB_Update();
			}
		}
		App_Led_Mcu_Handle();
		HAL_Delay(10);
	}
}
