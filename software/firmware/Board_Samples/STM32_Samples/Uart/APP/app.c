#include "app.h"
#include "usart.h"

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

// The serial port receiving is interrupted. Procedure  串口接收完成中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART_RxCpltCallback can be implemented in the user file
     */
    // 测试发送数据，实际应用中不应该在中断中发送数据
    // Test sending data. In practice, data should not be sent during interrupts  
    HAL_UART_Transmit(&huart1, (uint8_t *)&RxTemp, 1, 0xFFFF);

    // Continue receiving data  继续接收数据
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
}


void App_Handle(void)
{
    int print_count = 0;

    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);

    while (1)
	{
        print_count++;
        if (print_count % 100 == 0)
        {
            printf("count:%d\n", print_count/100);
        }
		App_Led_Mcu_Handle();
		HAL_Delay(10);
	}
}


