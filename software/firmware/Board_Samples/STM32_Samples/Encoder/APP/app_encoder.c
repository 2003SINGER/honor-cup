#include "app_encoder.h"
#include "app.h"
#include "tim.h"


int g_Encoder_M1_Now = 0;
int g_Encoder_M2_Now = 0;
int g_Encoder_M3_Now = 0;
int g_Encoder_M4_Now = 0;

/**
 * @Brief: To read the encoder count, call every 10 milliseconds  读取编码器计数，需每10毫秒调用一次
 * @Note: 
 * @Parm: Motor id：电机的ID号:ENCODER_ID_M1, ENCODER_ID_M2, ENCODER_ID_M3, ENCODER_ID_M4
 * @Retval: Returns encoder count data  返回编码器计数数据
 */
static int Encoder_Read_CNT(uint8_t Encoder_id)
{
	short Encoder_TIM = 0;
	switch(Encoder_id)
	{
	case ENCODER_ID_M1:  Encoder_TIM = (short)TIM3 -> CNT; TIM3 -> CNT = 0; break;
	case ENCODER_ID_M2:  Encoder_TIM = (short)TIM2 -> CNT; TIM2 -> CNT = 0; break;
	case ENCODER_ID_M3:  Encoder_TIM = (short)TIM5 -> CNT; TIM5 -> CNT = 0; break;
	case ENCODER_ID_M4:  Encoder_TIM = (short)TIM4 -> CNT; TIM4 -> CNT = 0; break;
	default:  break;
	}
	return Encoder_TIM;
}


// 返回开机到现在总共统计的编码器的计数（单路）。
// Returns the total count of encoders from boot up to now (single channel)
int Encoder_Get_Count_Now(uint8_t Encoder_id)
{
	if (Encoder_id == ENCODER_ID_M1) return g_Encoder_M1_Now;
	if (Encoder_id == ENCODER_ID_M2) return g_Encoder_M2_Now;
	if (Encoder_id == ENCODER_ID_M3) return g_Encoder_M3_Now;
	if (Encoder_id == ENCODER_ID_M4) return g_Encoder_M4_Now;
	return 0;
}

// 获取开机到现在总共的四路编码器计数。
// Get the total four - way encoder count up to now
void Encoder_Get_ALL(int* Encoder_all)
{
	Encoder_all[0] = g_Encoder_M1_Now;
	Encoder_all[1] = g_Encoder_M2_Now;
	Encoder_all[2] = g_Encoder_M3_Now;
	Encoder_all[3] = g_Encoder_M4_Now;
}

// 更新编码器的计数总值。需每10毫秒调用一次
// Update the count value of the encoder. call every 10 milliseconds
void Encoder_Update_Count(void)
{
    // g_Encoder_M1_Now -= Encoder_Read_CNT(ENCODER_ID_M1);
	// g_Encoder_M2_Now -= Encoder_Read_CNT(ENCODER_ID_M2);
	// g_Encoder_M3_Now += Encoder_Read_CNT(ENCODER_ID_M3);
	// g_Encoder_M4_Now += Encoder_Read_CNT(ENCODER_ID_M4);

	g_Encoder_M1_Now += Encoder_Read_CNT(ENCODER_ID_M1);
	g_Encoder_M2_Now += Encoder_Read_CNT(ENCODER_ID_M2);
	g_Encoder_M3_Now -= Encoder_Read_CNT(ENCODER_ID_M3);
	g_Encoder_M4_Now -= Encoder_Read_CNT(ENCODER_ID_M4);
}

// Initializing timer  初始化定时器
void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}

