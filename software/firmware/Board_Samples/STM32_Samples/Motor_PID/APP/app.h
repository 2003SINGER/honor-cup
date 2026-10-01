#ifndef APP_H_
#define APP_H_

#include "main.h"
#include "stdint.h"
#include "stdio.h"



#define LED_MCU_ON()      HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, SET)
#define LED_MCU_OFF()     HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, RESET)
#define LED_MCU_TOGGLE()  HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin)


void App_Handle(void);

#endif /* APP_H_ */
