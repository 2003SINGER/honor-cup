#ifndef _APP_CAN_H_
#define _APP_CAN_H_

#include "fdcan.h"


typedef struct {
	FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t				Data[8];
} FDCAN_TxFrame_TypeDef;

typedef struct {
	FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
} FDCAN_RxFrame_TypeDef;


void Can_Init(void);
void Can_Test_Send(void);


#endif /* _APP_CAN_H_ */
