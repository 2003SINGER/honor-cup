#include "app.h"
#include "app_pwm_servo.h"

uint8_t RxTemp = 0;

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
    int servo_count = 0;
    int servo_angle = 0;
    PwmServo_Init();
    while (1)
	{
        servo_count++;
        if (servo_count >= 100)
        {
            servo_count = 0;
            if (servo_angle == 0)
            {
                servo_angle = 180;
            }
            else
            {
                servo_angle = 0;
            }
            PwmServo_Set_Angle(PWM_SERVO_ID_MAX, servo_angle);
        }
		App_Led_Mcu_Handle();
		HAL_Delay(10);
	}
}


