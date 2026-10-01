#include "app_pwm_servo.h"
#include "main.h"
#include "tim.h"


#define Servo_Set_Pulse_S1(value)    TIM12->CCR2=value
#define Servo_Set_Pulse_S2(value)    TIM12->CCR1=value

// 角度转成PWM脉冲数  Convert the angle to the number of PWM pulses
static uint32_t Angle_To_Pulse(uint8_t angle)
{
    return (uint32_t)angle * 11 + 500;
}


void PwmServo_Init(void)
{
    // 开启PWM通道  Enable the PWM channel
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    // 设置初始化角度为90度 Set the initial angle to 90 degrees.
    PwmServo_Set_Angle(PWM_SERVO_ID_MAX, 90);
}

// 设置舵机角度 Set the servo angle
void PwmServo_Set_Angle(uint8_t id, uint8_t angle)
{
    uint16_t pulse = Angle_To_Pulse(angle);
    if (id == PWM_SERVO_ID_1)
    {
        Servo_Set_Pulse_S1(pulse);
        return;
    }
    if (id == PWM_SERVO_ID_2)
    {
        Servo_Set_Pulse_S2(pulse);
        return;
    }
    if (id == PWM_SERVO_ID_MAX)
    {
        Servo_Set_Pulse_S1(pulse);
        Servo_Set_Pulse_S2(pulse);
        return;
    }
}



