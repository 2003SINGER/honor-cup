#ifndef APP_ARM6_H_
#define APP_ARM6_H_
#include "stdint.h"



#define FLAG_RECV            1
#define FLAG_RXBUFF          2

#define MEDIAN_VALUE         2000

#define MID_ID5_MAX          3700
#define MID_ID5_MIN          380
// (uint16_t)((MID_ID5_MAX-MID_ID5_MIN)/3+MID_ID5_MIN)
#define MID_VAL_ID5          1486

#define MID_VAL_ID6          3100


#define ARM_READ_TO_UART     0
#define ARM_READ_TO_FLASH    1
#define ARM_READ_TO_ARM      2


#define FLAG_OFFSET_ERROR    0
#define FLAG_OFFSET_OK       1
#define FLAG_OFFSET_OVER     2
#define FLAG_OFFSET_INVALID  3


#define MAX_SERVO_NUM        6

#define ARM_MAX_VALUE        (4000)
#define ARM_MIN_VALUE        (96)


typedef enum _Arm_Joint_ID
{
    ARM_ID_1 = 1,
    ARM_ID_2,
    ARM_ID_3,
    ARM_ID_4,
    ARM_ID_5,
    ARM_ID_6
} arm_joint_id_t;



void Arm_Set_Angle(uint8_t id, int16_t angle, int16_t runtime);
void Arm_Set_Angle6(int16_t a1, int16_t a2, int16_t a3, int16_t a4, int16_t a5, int16_t a6, int16_t runtime);

void Arm_Set_Snyc_Buffer(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6);
void Arm_Sync_Write(uint16_t sync_time);


/* 读取舵机当前角度 */
int16_t Arm_Get_Angle(uint8_t id);

int Arm_Cali_Median_Value(uint8_t id);
int Arm_Cali_Median_Value_All(void);

void Arm_Set_Median_Value(uint8_t id, uint16_t value);

void Arm_Read_All_Median_Value(void);
void Arm_Offset_Reset(void);


#endif /* APP_ARM6_H_ */
