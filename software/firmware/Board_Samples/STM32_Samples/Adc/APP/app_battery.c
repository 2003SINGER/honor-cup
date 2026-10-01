#include "app_battery.h"
#include "app.h"


// 获得 ADC 值, ch:通道值：ADC_CHANNEL_0~ADC_CHANNEL_19
uint16_t Bat_Get_Adc(uint32_t ch)
{
    ADC_ChannelConfTypeDef ADC1_ChanConf;

    ADC1_ChanConf.Channel = ch;
    ADC1_ChanConf.Rank = ADC_REGULAR_RANK_1;
    ADC1_ChanConf.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    ADC1_ChanConf.SingleDiff = ADC_SINGLE_ENDED;
    ADC1_ChanConf.OffsetNumber = ADC_OFFSET_NONE;
    ADC1_ChanConf.Offset = 0;
    ADC1_ChanConf.OffsetSignedSaturation = DISABLE;
    HAL_ADC_ConfigChannel(&hadc1, &ADC1_ChanConf); // 通道配置

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    return (uint16_t)HAL_ADC_GetValue(&hadc1);
}

// 获得测得IO口电压值
float Bat_Get_GPIO_Volotage(void)
{
    uint16_t adcx;
    float temp;
    adcx = Bat_Get_Adc(BAT_ADC_CHANNEL);
    temp = (float)adcx * (3.30f / 4096);
    return temp;
}

// 获得实际电池分压前电压
float Bat_Get_Battery_Volotage(void)
{
    float temp;
    temp = Bat_Get_GPIO_Volotage();
    temp = temp * 4.03f; // temp*(10+3.3)/3.3;
    return temp;
}
