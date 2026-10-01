#ifndef _APP_PWM_SERVO_H_
#define _APP_PWM_SERVO_H_
#include "stdint.h"


typedef enum _PwmServo_ID
{
    PWM_SERVO_ID_1,
    PWM_SERVO_ID_2,
    PWM_SERVO_ID_MAX
} PwmServo_ID_t;


void PwmServo_Init(void);
void PwmServo_Set_Angle(uint8_t id, uint8_t angle);



#endif /* _APP_PWM_SERVO_H_ */
