#include "app.h"
#include "app_battery.h"


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
    uint16_t adc = 0;
    float voltage = 0.0;
    float battery = 0.0;
    while (1)
	{
        print_count++;
        if (print_count % 100 == 0)
        {
            adc = Bat_Get_Adc(BAT_ADC_CHANNEL);
            voltage = Bat_Get_GPIO_Volotage();
            battery = Bat_Get_Battery_Volotage();
            printf("voltage:%d, %f, %f\n", adc, voltage, battery);
        }
		App_Led_Mcu_Handle();
		HAL_Delay(10);
	}
}


