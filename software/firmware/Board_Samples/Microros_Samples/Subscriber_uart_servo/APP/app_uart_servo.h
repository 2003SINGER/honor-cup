#ifndef __APP_UART_SERVO_H__
#define __APP_UART_SERVO_H__

#include "stdint.h"

#define PTO_HEAD1            0xFF
#define PTO_HEAD2            0xF5

#define MAX_RX_SIZE          8
#define RX_TIMEOUT_MS        2


#define USERVO_MAX_VALUE     (4000)
#define USERVO_MIN_VALUE     (96)



void UartServo_Send_Data(uint8_t* data, uint16_t len);


/* 控制总线舵机 */
void UartServo_Set_Position(uint8_t id, uint16_t value, uint16_t time);

/* 设置总线舵机的扭矩开关,0为关闭，1为开启*/
void UartServo_Set_Torque(uint8_t enable);

/* 写入目标ID(1~250) */
void UartServo_Set_ID(uint8_t id);


/* 读取总线舵机当前位置 */
int16_t UartServo_Get_Position(uint8_t servo_id);

/* 设置舵机接收标识，数据接收完成时调用 */
void UartServo_Set_Rx_Flag(void);

#endif /* __APP_UART_SERVO_H__ */
