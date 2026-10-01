#ifndef BSP_MOTION_H_
#define BSP_MOTION_H_

#include "stdint.h"

// 轮子一整圈的位移
// The displacement of a wheel in one complete turn, Unit: mm 
#define MECANUM_CIRCLE_MM            (251.33f)

#define MECANUM_CIRCLE_M             (0.2513f)
// 底盘电机间距之和的一半, 单位是米
// Half of the sum of the chassis motor spacing
#define MECANUM_APB                  (0.167f)

#define MECANUM_DIR_X                (1)
#define MECANUM_DIR_Y                (2)
#define MECANUM_DIR_Z                (3)


#define MECANUM_MAX_SPEED_X          (0.7f)
#define MECANUM_MAX_SPEED_Y          (0.7f)
#define MECANUM_MAX_SPEED_Z          (4.2f)

#define MECANUM_LIMIT_SPEED          (750)


// 停止模式，STOP_FREE表示自由停止，STOP_BRAKE表示刹车。
// Stop mode, STOP_FREE: stop freely, STOP_BRAKE: brake
typedef enum _stop_mode {
    STOP_FREE = 0,
    STOP_BRAKE
} stop_mode_t;

// The speed structure of the car  小车的速度结构体
typedef struct _car_data
{
    float v_x;
    float v_y;
    float v_z;
} car_data_t;

typedef enum _motion_state {
    MOTION_STOP = 0,
    MOTION_RUN,
    MOTION_BACK,
    MOTION_LEFT,
    MOTION_RIGHT,
    MOTION_SPIN_LEFT,
    MOTION_SPIN_RIGHT,
    MOTION_BRAKE,

    MOTION_MAX_STATE
} motion_state_t;



extern car_data_t car_data;

void Motion_Stop(uint8_t brake);
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4);
void Motion_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z);
void Motion_Ctrl_Car(float v_x, float v_y, float v_z);
void Motion_Ctrl_State(uint8_t state, uint16_t speed);
void Motion_Ctrl_State_Car(uint8_t state, float speed);

void Motion_Get_Encoder(void);
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4);
void Motion_Handle(void);

void Motion_Get_Speed(car_data_t* car);
float Motion_Get_Circle_MM(void);
float Motion_Get_Circle_M(void);
float Motion_Get_APB(void);


void Motion_Send_Data(void);


#endif /* BSP_MOTION_H_ */
