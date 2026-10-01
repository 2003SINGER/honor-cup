#include "app.h"
#include "app_oled.h"


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
    int print_count = 0;
    HAL_Delay(100);
	char text_1[] = "Hello Yahboom";
	SSD1306_Init();
	OLED_Draw_Line(text_1, 1, 1, 1);
    printf("Hello Yahboom\n");
    
    while (1)
	{
        print_count++;
        if (print_count % 100 == 0)
        {
            char text_2[20] = {0};
            sprintf(text_2, "count:%d", print_count/100);
            OLED_Draw_Line(text_1, 1, 1, 0);
            OLED_Draw_Line(text_2, 2, 0, 1);
            printf(text_2);
            printf("\n");
        }
		App_Led_Mcu_Handle();
		HAL_Delay(10);
	}
}


