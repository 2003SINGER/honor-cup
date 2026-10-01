#include "app.h"
#include "app_motor.h"
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
    Motor_Init();
    HAL_Delay(1000);

    while (1)
	{
        Motor_Set_Pwm(MOTOR_ID_M1, MOTOR_MAX_PULSE/3);
        Motor_Set_Pwm(MOTOR_ID_M2, MOTOR_MAX_PULSE/3);
        Motor_Set_Pwm(MOTOR_ID_M3, MOTOR_MAX_PULSE/3);
        Motor_Set_Pwm(MOTOR_ID_M4, MOTOR_MAX_PULSE/3);
        HAL_Delay(1000);
        Motor_Stop(MOTOR_STOP);
        HAL_Delay(1000);

        Motor_Set_Pwm(MOTOR_ID_M1, -MOTOR_MAX_PULSE/3);
        Motor_Set_Pwm(MOTOR_ID_M2, -MOTOR_MAX_PULSE/3);
        Motor_Set_Pwm(MOTOR_ID_M3, -MOTOR_MAX_PULSE/3);
        Motor_Set_Pwm(MOTOR_ID_M4, -MOTOR_MAX_PULSE/3);
        HAL_Delay(1000);
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(1000);
	}
}


