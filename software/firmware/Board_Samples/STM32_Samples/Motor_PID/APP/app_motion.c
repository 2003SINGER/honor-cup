#include "app_motion.h"
#include "app.h"
#include "app_motor.h"
#include "app_pid.h"
#include "app_encoder.h"


int g_Encoder_All_Now[MOTOR_ID_MAX] = {0};
int g_Encoder_All_Last[MOTOR_ID_MAX] = {0};

int g_Encoder_All_Offset[MOTOR_ID_MAX] = {0};

uint8_t g_start_ctrl = 0;

int16_t speed_data[4] = {0};
motor_data_t motor_data;
int debug_count = 0;


// 控制小车运动，Motor_X=[-1000, 1000]，超过范围则无效。
// Control car movement, Motor_X=[-1000, 1000], beyond the range is invalid. 
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4)
{
    int16_t max_value = MOTOR_MAX_SPEED;
    if (Motor_1 >= -max_value && Motor_1 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M1, Motor_1);
    }
    if (Motor_2 >= -max_value && Motor_2 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M2, Motor_2);
    }
    if (Motor_3 >= -max_value && Motor_3 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M3, Motor_3);
    }
    if (Motor_4 >= -max_value && Motor_4 <= max_value)
    {
        Motor_Set_Pwm(MOTOR_ID_M4, Motor_4);
    }
}

// The car stopped  小车停止
void Motion_Stop(uint8_t brake)
{
    Motion_Set_Speed(0, 0, 0, 0);
    PID_Clear_Motor(MOTOR_ID_MAX);
    Motor_Stop(brake);
    g_start_ctrl = 0;
}


// 设置速度 speed_mX=[-700, 700], 单位为：mm/s
// Set speed speed mX=[-700, 700], unit: mm/s
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4)
{
    g_start_ctrl = 1;
    motor_data.speed_set[0] = speed_m1;
    motor_data.speed_set[1] = speed_m2;
    motor_data.speed_set[2] = speed_m3;
    motor_data.speed_set[3] = speed_m4;
    for (uint8_t i = 0; i < MOTOR_ID_MAX; i++)
    {
        PID_Set_Motor_Target(i, motor_data.speed_set[i]*1.0);
    }
}


// 从编码器读取当前各轮子速度，单位mm/s
// Read the current speed of each wheel from the encoder in mm/s
void Motion_Get_Speed(int16_t* speed_motors)
{
    Motion_Get_Encoder();

    float circle_m = Motion_Get_Circle_M();

    float speed_m1 = (g_Encoder_All_Offset[0]) * 100 * circle_m / ENCODER_CIRCLE;
    float speed_m2 = (g_Encoder_All_Offset[1]) * 100 * circle_m / ENCODER_CIRCLE;
    float speed_m3 = (g_Encoder_All_Offset[2]) * 100 * circle_m / ENCODER_CIRCLE;
    float speed_m4 = (g_Encoder_All_Offset[3]) * 100 * circle_m / ENCODER_CIRCLE;

    speed_motors[0] = speed_m1 * 1000;
    speed_motors[1] = speed_m2 * 1000;
    speed_motors[2] = speed_m3 * 1000;
    speed_motors[3] = speed_m4 * 1000;

    if (g_start_ctrl)
    {
        motor_data.speed_mm_s[0] = speed_m1*1000;
        motor_data.speed_mm_s[1] = speed_m2*1000;
        motor_data.speed_mm_s[2] = speed_m3*1000;
        motor_data.speed_mm_s[3] = speed_m4*1000;

        PID_Calc_Motor(&motor_data);
    }
}


float Motion_Get_Circle_M(void)
{
    return MECANUM_CIRCLE_M;
}

// Obtain encoder data and calculate the number of deviation pulses  获取编码器数据，并计算偏差脉冲数
void Motion_Get_Encoder(void)
{
    Encoder_Update_Count();
    Encoder_Get_ALL(g_Encoder_All_Now);

    for(uint8_t i = 0; i < MOTOR_ID_MAX; i++)
    {
        g_Encoder_All_Offset[i] = g_Encoder_All_Now[i] - g_Encoder_All_Last[i];
	    g_Encoder_All_Last[i] = g_Encoder_All_Now[i];
    }
}



// 运动控制句柄，每10ms调用一次，主要处理速度相关的数据
// Motion control handle, called every 10ms, mainly processing speed related data
void Motion_Handle(void)
{
    Motion_Get_Speed(speed_data);
    debug_count++;
    if (debug_count >= 30)
    {
        debug_count = 0;
        printf("motor speed:%d, %d, %d, %d\n", speed_data[0], speed_data[1], speed_data[2], speed_data[3]);
    }


    if (g_start_ctrl)
    {
        Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1],
        		motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
    }
}

