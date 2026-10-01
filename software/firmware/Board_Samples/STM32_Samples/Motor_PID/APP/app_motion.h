#ifndef BSP_MOTION_H_
#define BSP_MOTION_H_

#include "stdint.h"

// 轮子一整圈的位移，单位为米
// The displacement of a wheel in one complete turn, Unit: m 
#define MECANUM_CIRCLE_M        (0.2513f)

// 轮子最大速度
#define MECANUM_MAX_SPEED       (700)


// 停止模式，STOP_FREE表示自由停止，STOP_BRAKE表示刹车。
// Stop mode, STOP_FREE: stop freely, STOP_BRAKE: brake
typedef enum _stop_mode {
    STOP_FREE = 0,
    STOP_BRAKE
} stop_mode_t;




void Motion_Stop(uint8_t brake);
void Motion_Set_Pwm(int16_t Motor_1, int16_t Motor_2, int16_t Motor_3, int16_t Motor_4);

void Motion_Get_Encoder(void);
void Motion_Set_Speed(int16_t speed_m1, int16_t speed_m2, int16_t speed_m3, int16_t speed_m4);
void Motion_Get_Speed(int16_t* speed_motors);
float Motion_Get_Circle_M(void);
void Motion_Handle(void);



#endif /* BSP_MOTION_H_ */
