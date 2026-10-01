#include "app.h"
#include "usart.h"
#include "app_key.h"
#include "app_flash.h"



//要写入到STM32 FLASH的字符串数组
#define FLASH_LEN     32


// uint8_t TEXT_Buffer[FLASH_LEN]={"STM32 FLASH TEST:0"};

#define FLASH_SAVE_ADDR  0x08120000 	//设置FLASH 保存地址(必须为4的倍数，且所在扇区,要大于本代码所占用到的扇区.
										//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.



void App_Key1_Handle(void)
{
    static int state = 0;
    if (Key1_State() == KEY_PRESS)
    {
        state++;
        char TEXT_Buffer[FLASH_LEN];
        sprintf((char*)TEXT_Buffer, "STM32 FLASH TEST:%d", state);
        Flash_Write(FLASH_SAVE_ADDR, (uint32_t*)TEXT_Buffer, 8);
        printf("STM32 FLASH TEST:%d\n", state);
    }
}


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
    uint8_t datatemp0[FLASH_LEN] = {0};	
    HAL_Delay(100);
    Flash_Read(FLASH_SAVE_ADDR, (uint32_t*)datatemp0, FLASH_LEN/4);
    printf("System Start\n");
    printf("Read Flash: %s\n", datatemp0);
    
    while (1)
	{
        App_Key1_Handle();
		App_Led_Mcu_Handle();
		HAL_Delay(10);
	}
}

