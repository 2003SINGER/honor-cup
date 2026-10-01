#ifndef APP_H_
#define APP_H_

#include "main.h"
#include "stdint.h"
#include "stdio.h"



#define LED_MCU_ON()      HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, SET)
#define LED_MCU_OFF()     HAL_GPIO_WritePin(LED_MCU_GPIO_Port, LED_MCU_Pin, RESET)
#define LED_MCU_TOGGLE()  HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin)

#define LED_ROS_ON()      HAL_GPIO_WritePin(LED_ROS_GPIO_Port, LED_ROS_Pin, SET)
#define LED_ROS_OFF()     HAL_GPIO_WritePin(LED_ROS_GPIO_Port, LED_ROS_Pin, RESET)
#define LED_ROS_TOGGLE()  HAL_GPIO_TogglePin(LED_ROS_GPIO_Port, LED_ROS_Pin)

#define LED_SW_ON()       HAL_GPIO_WritePin(LED_SW_GPIO_Port, LED_SW_Pin, SET)
#define LED_SW_OFF()      HAL_GPIO_WritePin(LED_SW_GPIO_Port, LED_SW_Pin, RESET)
#define LED_SW_TOGGLE()   HAL_GPIO_TogglePin(LED_SW_GPIO_Port, LED_SW_Pin)


void App_Handle(void);


#endif /* APP_H_ */
