#ifndef __APP_BATTERY_H__
#define __APP_BATTERY_H__
#include "adc.h"
#include "stdint.h"


#define BAT_ADC_CHANNEL ADC_CHANNEL_10



uint16_t Bat_Get_Adc(uint32_t ch);
float Bat_Get_GPIO_Volotage(void);
float Bat_Get_Battery_Volotage(void);




#endif /* __APP_BATTERY_H__ */
