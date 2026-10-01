#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include "stdint.h"


// 轮子转一整圈，编码器获得的脉冲数:56*11*2*2
// One full turn of the wheel, the number of pulses picked up by the coder: 56*11*2*2
#define ENCODER_CIRCLE           (2464.0f)


typedef enum {
    ENCODER_ID_M1 = 0,
    ENCODER_ID_M2,
    ENCODER_ID_M3,
    ENCODER_ID_M4,
    MAX_ENCODER
} Encoder_ID;


void Encoder_Init(void);
void Encoder_Update_Count(void);
int32_t Encoder_Get_Count_Now(uint8_t encoder_id);
void Encoder_Get_ALL(int32_t* Encoder_all);


void Encoder_Send_Count_Now(void);
void Encoder_Debug_Handle(void);

#endif

