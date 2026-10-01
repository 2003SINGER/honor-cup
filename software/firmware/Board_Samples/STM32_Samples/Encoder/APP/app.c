#include "app.h"
#include "app_encoder.h"
#include "tim.h"




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
    uint8_t print_count = 0;
    int g_Encoder_Now[4] = {0};
    Encoder_Init();
    HAL_Delay(100);

    while (1)
	{
        Encoder_Update_Count();
        Encoder_Get_ALL(g_Encoder_Now);
        print_count++;
        if (print_count >= 30)
        {
            print_count = 0;
            printf("count:%d, %d, %d, %d\n", g_Encoder_Now[0], g_Encoder_Now[1], g_Encoder_Now[2], g_Encoder_Now[3]);
        }
		App_Led_Mcu_Handle();
		HAL_Delay(10);
	}
}


