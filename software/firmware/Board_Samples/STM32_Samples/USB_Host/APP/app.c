#include "app.h"
#include "usb_host.h"

int print_count = 0;

// LED显示当前运行状态，每10毫秒调用一次，LED灯每200毫秒闪烁一次。
// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.
void App_Led_Mcu_Handle(void)
{
	static uint8_t led_count = 0;
	led_count++;
	if (led_count > 30)
	{
		led_count = 0;
		LED_MCU_TOGGLE();
	}
}

void App_Loop_10ms(void)
{
    print_count++;
    // if (print_count % 100 == 0)
    // {
    //     printf("count:%d\n", print_count/100);
    // }
	App_Led_Mcu_Handle();
}

void App_Handle(void)
{
    uint32_t lastTick = HAL_GetTick();
    while (1)
	{
        MX_USB_HOST_Process();

        if (HAL_GetTick() - lastTick >= 10)
        {
            lastTick = HAL_GetTick();
            App_Loop_10ms();
        }
	}
}


