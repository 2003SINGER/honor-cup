#include <stdio.h>
#include <string.h>

#include "inv_mems_hw_config.h"
#include "invn_types.h"

#include "icm20948.h"
#include "spi.h"
#include "app.h"
#include "cmsis_os2.h"




static void inv_enable_cs(void)
{
    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, RESET);
    
}

static void inv_disable_cs(void)
{
    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, SET);
}


static uint8_t inv_read_write_data(uint8_t TxData, uint8_t* RxData)
{
    HAL_SPI_TransmitReceive(&hspi2, &TxData, RxData, 1, 1000);
    return 0;
}


int inv_serial_interface_write_hook(uint16_t reg, uint32_t length, uint8_t *data)
{
    unsigned char rx;
    int result = 0, i = 0;
    inv_enable_cs();
    result |= inv_read_write_data((unsigned char)reg, &rx);
    for(; i < length; i++)
    {
        result |= inv_read_write_data( data[i], &rx);
    }
    inv_disable_cs();
    return result;
}

int inv_serial_interface_read_hook(uint16_t reg, uint32_t length, uint8_t *data)
{
    unsigned char rx;
    int result = 0, i = 0;
    inv_enable_cs();
    reg = reg | 0x80;
    result |= inv_read_write_data((unsigned char)reg, &rx);
    for(; i < length; i++)
    {
        result |= inv_read_write_data(0xff, data++);
    }
    inv_disable_cs();
    return result;
}

//外部中断服务程序
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == IMU_INT_Pin)
    {
        gyro_data_ready_cb();
    }
}


/**
 *  @brief  Sleep function.
**/
void inv_sleep(unsigned long mSecs)
{
    osDelay(mSecs);
}

void inv_sleep_100us(unsigned long nHowMany100MicroSecondsToSleep)
{
    osDelay(nHowMany100MicroSecondsToSleep);
}

/**
 *  @brief  get system's internal tick count.
 *          Used for time reference.
 *  @return current tick count.
**/
long long inv_get_tick_count(void)
{
    long long count;

    get_tick_count(&count);

    return (long long)count;
}

