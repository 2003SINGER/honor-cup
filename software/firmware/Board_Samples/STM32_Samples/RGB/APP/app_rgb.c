#include "app_rgb.h"
#include "main.h"
#include "stdint.h"
#include "spi.h"

// 硬件spi模拟ws2812时序（用spi的4位数据模拟ws2812的一位数据）
// 要求SPI的通信频率为3.75M，传输一位数据的时间约为267ns（纳秒）
//  ___
// |   |_|   0b1110  high level
//  _   
// | |___|   0b1000  low level
#define TIMING_ONE           0x0E
#define TIMING_ZERO          0x08


// Store the color information of the light bar  储存灯条的颜色信息
ws2812_t g_ws2812 = {0};

// transmitter data  发送数据
static void WS2812_Send_Data(uint8_t *buf, uint16_t buf_size)
{
	HAL_SPI_Transmit_DMA(&hspi4, buf, buf_size);
}

// 设置单个RGB灯颜色值，index=[0, MAX_RGB-1]，RGB=[0x00000000, 0x00FFFFFF]
// Set single RGB light color value, index=[0, MAX_RGB-1], RGB=[0x00000000, 0x00FFFFFF] 
static void WS2812_Set_Color_One(uint8_t index, uint32_t RGB)
{
    if (index >= MAX_RGB) return;
    uint8_t i;
    uint64_t TempR = 0, TempG = 0, TempB = 0;

    for(i = 0; i < 8; i++)
    {
        (RGB & 0x00010000) == 0 ? (TempR |= (TIMING_ZERO<<(i*RGB_BIT_WIDTH))) : (TempR |= (TIMING_ONE<<(i*RGB_BIT_WIDTH)));
        (RGB & 0x00000100) == 0 ? (TempG |= (TIMING_ZERO<<(i*RGB_BIT_WIDTH))) : (TempG |= (TIMING_ONE<<(i*RGB_BIT_WIDTH)));
        (RGB & 0x00000001) == 0 ? (TempB |= (TIMING_ZERO<<(i*RGB_BIT_WIDTH))) : (TempB |= (TIMING_ONE<<(i*RGB_BIT_WIDTH)));
        RGB >>= 1;
    }
    for (i = 0; i < RGB_BIT_WIDTH; i++)
    {
        g_ws2812.Strip[index].RGB.R[i] = TempR >> (8*(RGB_BIT_WIDTH-i-1));
        g_ws2812.Strip[index].RGB.G[i] = TempG >> (8*(RGB_BIT_WIDTH-i-1));
        g_ws2812.Strip[index].RGB.B[i] = TempB >> (8*(RGB_BIT_WIDTH-i-1));
    }
}


// Initializes the indicator bar  初始化灯条
void RGB_Init(void)
{
	RGB_Clear();
	RGB_Update();
}

// 刷新RGB灯条颜色。下方函数调用修改RGB颜色后，必须调用此函数更新显示。
// Refresh RGB light bar color. This function must be called to update the display after the RGB color is modified by the function call below.  
void RGB_Update(void)
{
    // WS2812_Send_Data((uint8_t*)&g_ws2812, RGB_BIT_SIZE*MAX_RGB+RGB_RESET_WIDTH);
    WS2812_Send_Data((uint8_t*)&g_ws2812, sizeof(g_ws2812));
}

// 设置颜色，index=[0, MAX_RGB-1]控制对应灯珠颜色, index=0xFF控制所有灯珠颜色。
// Set the color, index=[0, max_RGB-1] controls the corresponding bead color, index=0xFF controls all the bead color.
void RGB_Set_Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t color = r << 16 | g << 8 | b;
    RGB_Set_Color_U32(index, color);
}

// 设置RGB灯条颜色值，index=[0, MAX_RGB-1]控制对应灯珠颜色, index=255控制所有灯珠颜色。
// Set the RGB bar color value, index=[0, max_RGB-1] controls the corresponding bead color, index=255 controls all the bead color.
void RGB_Set_Color_U32(uint8_t index, uint32_t color)
{
    if (index < MAX_RGB)
    {
        WS2812_Set_Color_One(index, color);
        return;
    }
    if (index == RGB_CTRL_ALL)
    {
        for (uint16_t i = 0; i < MAX_RGB; i++)
        {
            WS2812_Set_Color_One(i, color);
        }
    }
}

// Clear color (off)  清除颜色（熄灭）
void RGB_Clear(void)
{
    for (uint8_t i = 0; i < MAX_RGB; i++)
    {
        WS2812_Set_Color_One(i, 0);
    }
}
