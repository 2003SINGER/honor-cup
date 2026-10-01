#include "app_key.h"
#include "app.h"

// 判断按键是否被按下，按下返回KEY_PRESS，松开返回KEY_RELEASE
static uint8_t Key1_is_Press(void)
{
	if (!HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin))
	{
		return KEY_PRESS; // 如果按键被按下，则返回KEY_PRESS
	}
	return KEY_RELEASE;   // 如果按键是松开状态，则返回KEY_RELEASE
}


// 读取按键K1的状态，需要每10毫秒调用一次。按下返回一次KEY_PRESS.
uint8_t Key1_State(void)
{
	static uint16_t key1_state = 0;

	if (Key1_is_Press() == KEY_PRESS)
	{
		if (key1_state < 4)
		{
			key1_state++;
		}
	}
	else
	{
		key1_state = 0;
	}
	if (key1_state == 2)
	{
		return KEY_PRESS;
	}
	return KEY_RELEASE;
}

