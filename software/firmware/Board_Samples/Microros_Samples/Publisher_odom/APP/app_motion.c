#include "app_motion.h"
#include "app.h"
#include "app_motor.h"
#include "app_pid.h"
#include "app_encoder.h"
#include "usart.h"



int32_t g_Encoder_All_Now[MOTOR_ID_MAX] = {0};
int32_t g_Encoder_All_Last[MOTOR_ID_MAX] = {0};

int32_t g_Encoder_All_Offset[MOTOR_ID_MAX] = {0};

uint8_t g_start_ctrl = 0;
uint16_t g_speed_setup = 0;

int speed_L1_setup = 0;
int speed_L2_setup = 0;
int speed_R1_setup = 0;
int speed_R2_setup = 0;

car_data_t car_data;
motor_data_t motor_data;

static float Limit_Input_Speed(uint8_t dir, float speed)
{
    if (dir == MECANUM_DIR_X)
    {
        if (speed > MECANUM_MAX_SPEED_X)
        {
            speed = MECANUM_MAX_SPEED_X;
        }
        if (speed < -MECANUM_MAX_SPEED_X)
        {
            speed = -MECANUM_MAX_SPEED_X;
        }
    }
    else if (dir == MECANUM_DIR_Y)
    {
        if (speed > MECANUM_MAX_SPEED_Y)
        {
            speed = MECANUM_MAX_SPEED_Y;
        }
        if (speed < -MECANUM_MAX_SPEED_Y)
        {
            speed = -MECANUM_MAX_SPEED_Y;
        }
    }
    else if (dir == MECANUM_DIR_Z)
    {
        if (speed > MECANUM_MAX_SPEED_Z)
        {
            speed = MECANUM_MAX_SPEED_Z;
        }
        if (speed < -MECANUM_MAX_SPEED_Z)
        {
            speed = -MECANUM_MAX_SPEED_Z;
        }
    }
    else
    {
        speed = 0.0;
    }
    return speed;
}



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
void Motion_Get_Speed(car_data_t* car)
{
    Motion_Get_Encoder();

    float circle_m = Motion_Get_Circle_M();

    float speed_m1 = (g_Encoder_All_Offset[0]) * 100 * circle_m / ENCODER_CIRCLE;
    float speed_m2 = (g_Encoder_All_Offset[1]) * 100 * circle_m / ENCODER_CIRCLE;
    float speed_m3 = (g_Encoder_All_Offset[2]) * 100 * circle_m / ENCODER_CIRCLE;
    float speed_m4 = (g_Encoder_All_Offset[3]) * 100 * circle_m / ENCODER_CIRCLE;
    float robot_APB = Motion_Get_APB();

    car->v_x = (speed_m1 + speed_m2 + speed_m3 + speed_m4) / 4.0;
    car->v_y = (-speed_m1 + speed_m2 + speed_m3 - speed_m4) / 4.0;
    car->v_z = (-speed_m1 - speed_m2 + speed_m3 + speed_m4) / 4.0f / robot_APB;

    if (g_start_ctrl)
    {
        motor_data.speed_mm_s[0] = speed_m1*1000;
        motor_data.speed_mm_s[1] = speed_m2*1000;
        motor_data.speed_mm_s[2] = speed_m3*1000;
        motor_data.speed_mm_s[3] = speed_m4*1000;
        PID_Calc_Motor(&motor_data);
    }
}

// Returns half of the sum of the current cart wheel axles  返回当前小车轮子轴间距和的一半
float Motion_Get_APB(void)
{
    return MECANUM_APB;
}

// Returns the number of millimeters at which the current wheel has been turned  返回当前小车轮子转一圈多少毫米
float Motion_Get_Circle_MM(void)
{
    return MECANUM_CIRCLE_MM;
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

// Control car movement  控制小车运动
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z)
{
    float robot_APB = Motion_Get_APB();
    float speed_lr = -V_y;
    float speed_fb = V_x;
    float speed_spin = -V_z * robot_APB;
    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        g_speed_setup = 0;
        Motion_Stop(STOP_BRAKE);
        return;
    }
    
    speed_L1_setup = speed_fb + speed_lr + speed_spin;
    speed_L2_setup = speed_fb - speed_lr + speed_spin;
    speed_R1_setup = speed_fb - speed_lr - speed_spin;
    speed_R2_setup = speed_fb + speed_lr - speed_spin;

    if (speed_L1_setup > MECANUM_LIMIT_SPEED) speed_L1_setup = MECANUM_LIMIT_SPEED;
    if (speed_L1_setup < -MECANUM_LIMIT_SPEED) speed_L1_setup = -MECANUM_LIMIT_SPEED;
    if (speed_L2_setup > MECANUM_LIMIT_SPEED) speed_L2_setup = MECANUM_LIMIT_SPEED;
    if (speed_L2_setup < -MECANUM_LIMIT_SPEED) speed_L2_setup = -MECANUM_LIMIT_SPEED;
    if (speed_R1_setup > MECANUM_LIMIT_SPEED) speed_R1_setup = MECANUM_LIMIT_SPEED;
    if (speed_R1_setup < -MECANUM_LIMIT_SPEED) speed_R1_setup = -MECANUM_LIMIT_SPEED;
    if (speed_R2_setup > MECANUM_LIMIT_SPEED) speed_R2_setup = MECANUM_LIMIT_SPEED;
    if (speed_R2_setup < -MECANUM_LIMIT_SPEED) speed_R2_setup = -MECANUM_LIMIT_SPEED;
    Motion_Set_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
}

void Motion_Ctrl_Car(float v_x, float v_y, float v_z)
{
    v_x = Limit_Input_Speed(MECANUM_DIR_X, v_x);
    v_y = Limit_Input_Speed(MECANUM_DIR_Y, v_y);
    v_z = Limit_Input_Speed(MECANUM_DIR_Z, v_z);
    Motion_Ctrl(v_x*1000, v_y*1000, v_z*1000);
}

// 速度控制：speed=0~0.7。
void Motion_Ctrl_State_Car(uint8_t state, float speed)
{
    uint16_t speed_mm_s = Limit_Input_Speed(MECANUM_DIR_X, speed) * 1000;
    Motion_Ctrl_State(state, speed_mm_s);
}

// 速度控制：speed=0~700。
void Motion_Ctrl_State(uint8_t state, uint16_t speed)
{
    if (speed > 700)
    {
        speed = 700;
    }
    g_speed_setup = speed;
    switch (state)
    {
    case MOTION_STOP:
        Motion_Stop(speed==0?STOP_FREE:STOP_BRAKE);
        g_speed_setup = 0;
        break;
    case MOTION_RUN:
        Motion_Ctrl(speed, 0, 0);
        break;
    case MOTION_BACK:
        Motion_Ctrl(-speed, 0, 0);
        break;
    case MOTION_LEFT:
        Motion_Ctrl(0, speed, 0);
        break;
    case MOTION_RIGHT:
        Motion_Ctrl(0, -speed, 0);
        break;
    case MOTION_SPIN_LEFT:
        Motion_Ctrl(0, 0, speed*6);
        g_speed_setup = 0;
        break;
    case MOTION_SPIN_RIGHT:
        Motion_Ctrl(0, 0, -speed*6);
        g_speed_setup = 0;
        break;
    case MOTION_BRAKE:
        Motion_Stop(STOP_BRAKE);
        g_speed_setup = 0;
        break;
    default:
        break;
    }
}


// 运动控制句柄，每10ms调用一次，主要处理速度相关的数据
// Motion control handle, called every 10ms, mainly processing speed related data
void Motion_Handle(void)
{
    Motion_Get_Speed(&car_data);

    if (g_start_ctrl)
    {
        Motion_Set_Pwm(motor_data.speed_pwm[0], motor_data.speed_pwm[1],
        		motor_data.speed_pwm[2], motor_data.speed_pwm[3]);
    }
}

