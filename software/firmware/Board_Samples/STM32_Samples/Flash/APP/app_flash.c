#include "app_flash.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "tim.h"

#define ENABLE_FLASH_DEBUG               0


static uint16_t time_out = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM6)
	{
		time_out = 1;
	}
}


void delay_us(uint32_t nus)
{
    time_out = 0;
    HAL_TIM_Base_Start_IT(&htim6);
    while (time_out == 0);
    HAL_TIM_Base_Stop_IT(&htim6);
}


// 从flash读取一个字（四个字节）
uint32_t Flash_Read_Word(uint32_t addr)
{
    return *(__IO uint32_t *)addr;
}


// 从flash读取数据
void Flash_Read(uint32_t addr, uint32_t *output, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++)
    {
        output[i] = Flash_Read_Word(addr); // 读取4个字节.
        addr += 4;                         // 偏移4个字节.
    }
}


// 得到FLASH的错误状态
// bankx:0,获取bank1的状态
//       1,获取bank2的状态
// 返回值:
// 0,无错误
// 其他,错误编号
uint8_t Flash_Get_Error_Status(uint8_t bankx)
{
    uint32_t res = 0;
    if (bankx == 0)
        res = FLASH->SR1;
    else
        res = FLASH->SR2;
    if (res & (1 << 17))
        return 1; // WRPERR=1,写保护错误
    else if (res & (1 << 18))
        return 2; // PGSERR=1,编程序列错误
    else if (res & (1 << 19))
        return 3; // STRBERR=1,复写错误
    else if (res & (1 << 21))
        return 4; // INCERR=1,数据一致性错误
    else if (res & (1 << 22))
        return 5; // OPERR=1,写/擦除错误
    else if (res & (1 << 23))
        return 6; // RDPERR=1,读保护错误
    else if (res & (1 << 24))
        return 7; // RDSERR=1,非法访问加密区错误
    else if (res & (1 << 25))
        return 8; // SNECCERR=1,1bit ecc校正错误
    else if (res & (1 << 26))
        return 9; // DBECCERR=1,2bit ecc错误
    return 0;     // 没有任何状态/操作完成.
}

uint8_t Flash_Get_Sector(uint32_t addr)
{
    if (addr < BANK1_FLASH_SECTOR_1)
        return FLASH_SECTOR_0;
    else if (addr < BANK1_FLASH_SECTOR_2)
        return FLASH_SECTOR_1;
    else if (addr < BANK1_FLASH_SECTOR_3)
        return FLASH_SECTOR_2;
    else if (addr < BANK1_FLASH_SECTOR_4)
        return FLASH_SECTOR_3;
    else if (addr < BANK1_FLASH_SECTOR_5)
        return FLASH_SECTOR_4;
    else if (addr < BANK1_FLASH_SECTOR_6)
        return FLASH_SECTOR_5;
    else if (addr < BANK1_FLASH_SECTOR_7)
        return FLASH_SECTOR_6;
    else if (addr < BANK2_FLASH_SECTOR_0)
        return FLASH_SECTOR_7;
    else if (addr < BANK2_FLASH_SECTOR_1)
        return FLASH_SECTOR_0;
    else if (addr < BANK2_FLASH_SECTOR_2)
        return FLASH_SECTOR_1;
    else if (addr < BANK2_FLASH_SECTOR_3)
        return FLASH_SECTOR_2;
    else if (addr < BANK2_FLASH_SECTOR_4)
        return FLASH_SECTOR_3;
    else if (addr < BANK2_FLASH_SECTOR_5)
        return FLASH_SECTOR_4;
    else if (addr < BANK2_FLASH_SECTOR_6)
        return FLASH_SECTOR_5;
    else if (addr < BANK2_FLASH_SECTOR_7)
        return FLASH_SECTOR_6;
    return FLASH_SECTOR_7;
}

// 等待操作完成
// bankx:0,bank1; 1,bank2
// time:要延时的长短(单位:10us)
// 返回值:
// 0,完成
// 1~9,错误代码.
// 0XFF,超时
uint8_t Flash_Wait_Done(uint8_t bankx, uint32_t time)
{
    uint8_t res = 0;
    uint32_t tempreg = 0;
    while (1)
    {
        if (bankx == 0)
            tempreg = FLASH->SR1;
        else
            tempreg = FLASH->SR2;
        if ((tempreg & 0X07) == 0)
            break; // BSY=0,WBNE=0,QW=0,则操作完成
        delay_us(10);
        time--;
        if (time == 0)
            return 0XFF;
    }
    res = Flash_Get_Error_Status(bankx);
    if (res)
    {
        if (bankx == 0)
            FLASH->CCR1 = 0X07EE0000; // 清所有错误标志
        else
            FLASH->CCR2 = 0X07EE0000; // 清所有错误标志
    }
    return res;
}



uint8_t Flash_Write_8Word(uint32_t addr, uint32_t *data)
{
    uint8_t nword = 8; // 每次写8个字,256bit
    uint8_t res;
    uint8_t bankx = 0;
    if (addr < BANK2_FLASH_SECTOR_0)
        bankx = 0; // 判断地址是在bank0,还是在bank1
    else
        bankx = 1;
    res = Flash_Wait_Done(bankx, 0XFF);
    if (res == 0) // OK
    {
        if (bankx == 0) // BANK1	编程
        {
            FLASH->CR1 &= ~(3 << 4); // PSIZE1[1:0]=0,清除原来的设置
            FLASH->CR1 |= 2 << 4;    // 设置为32bit宽,确保VCC=2.7~3.6V之间!!
            FLASH->CR1 |= 1 << 1;    // PG1=1,编程使能
        }
        else // BANK2 编程
        {
            FLASH->CR2 &= ~(3 << 4); // PSIZE2[1:0]=0,清除原来的设置
            FLASH->CR2 |= 2 << 4;    // 设置为32bit宽,确保VCC=2.7~3.6V之间!!
            FLASH->CR2 |= 1 << 1;    // PG2=1,编程使能
        }
        while (nword)
        {
            *(uint32_t *)addr = *data;  // 写入数据
            addr += 4;                  // 写地址+4
            data++;                     // 偏移到下一个数据首地址
            nword--;
        }
        for (int i = 0; i < 10000; i++);    // 等待写入结束

        __DSB();                              // 写操作完成后,屏蔽数据同步,使CPU重新执行指令序列
        res = Flash_Wait_Done(bankx, 0XFF); // 等待操作完成,一个字编程,最多100us.

        if (bankx == 0)
            FLASH->CR1 &= ~(1 << 1); // PG1=0,清除扇区擦除标志
        else
            FLASH->CR2 &= ~(1 << 1); // PG2=0,清除扇区擦除标志
    }
    return res;
}


int Flash_Write(uint32_t addr, uint32_t *data, uint32_t len)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t SectorError = 0;
    uint32_t addr_start = 0;
    uint32_t addr_end = 0;
    if (addr < STM32_FLASH_BASE || addr > STM32_FLASH_END || addr % 4)
    {
        #if ENABLE_FLASH_DEBUG
        printf("Flash address invalid\n");
        #endif
        return HAL_ERROR;
    }
    HAL_FLASH_Unlock();                   // 解锁
    addr_start = addr;                    // 写入的起始地址
    addr_end = addr + len * 4;            // 写入的结束地址
    #if ENABLE_FLASH_DEBUG
    printf("addr_start:0x%lx, 0x%lx\n", addr_start, addr_end);
    #endif
    while (addr_start < addr_end)
    {
        // 判断是否需要擦除扇区
        if (Flash_Read_Word(addr_start) != 0XFFFFFFFF)
        {
            FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;                                 // 擦除类型
            FlashEraseInit.Sector = Flash_Get_Sector(addr_start);                               // 要擦除的扇区
            FlashEraseInit.Banks = addr_start >= BANK2_FLASH_SECTOR_0 ? FLASH_BANK_2 : FLASH_BANK_1; // 操作 BANK
            FlashEraseInit.NbSectors = 1;                                                       // 一次只擦除一个扇区
            FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;                                // 电压范围
            if (HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError) != HAL_OK)
            {
                #if ENABLE_FLASH_DEBUG
                printf("Flash erease error\n");
                #endif
                HAL_FLASH_Lock(); // 擦除异常，上锁后返回失败码
                return HAL_ERROR;
            }
        }
        else
            addr_start += 4;
    }

    printf("flash start write data\n");

    while (addr < addr_end) // 写数据
    {
        if (Flash_Write_8Word(addr, data))
        {
            #if ENABLE_FLASH_DEBUG
            printf("Flash write8 error\n");
            #endif
            HAL_FLASH_Lock(); // 写入异常，上锁后返回失败码
            return HAL_ERROR;
        }
        addr += 32;
        data += 8;
    }
    HAL_FLASH_Lock(); // 上锁
    return HAL_OK;
}
