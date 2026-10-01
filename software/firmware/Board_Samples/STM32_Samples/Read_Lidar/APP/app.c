#include "app.h"

#include "usart.h"
#include "app_lidar.h"



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




void App_Handle(void)
{
    uint32_t lastTick = HAL_GetTick();

    printf("start APP Handle\n");
    Lidar_Init();

    while (1)
	{
        Lidar_Handle();
        if (HAL_GetTick() - lastTick >= 200)
        {
            lastTick = HAL_GetTick();
            App_Led_Mcu_Flash_200ms();

            printf("Lidar range:%ldmm\n", Lidar_Ranges[0]);
        }
	}
}

