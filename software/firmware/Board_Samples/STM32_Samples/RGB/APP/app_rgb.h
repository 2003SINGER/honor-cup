#ifndef _APP_RGB_H_
#define _APP_RGB_H_

#include "stdint.h"

#define RGB_CTRL_ALL    0xFF
#define MAX_RGB         8


#define RGB_BIT_WIDTH   4
#define RGB_BIT_SIZE    (RGB_BIT_WIDTH*3)
#define RGB_RESET_WIDTH 100


typedef struct rgb_ws2812
{
    uint8_t reset[RGB_RESET_WIDTH];
    union 
    {
        uint8_t Buff[RGB_BIT_SIZE];
        struct 
        {
            uint8_t G[RGB_BIT_WIDTH]; // G First
            uint8_t R[RGB_BIT_WIDTH]; // R Second
            uint8_t B[RGB_BIT_WIDTH]; // B Third
        } RGB;
    } Strip[MAX_RGB];
} ws2812_t;


void RGB_Init(void);
void RGB_Update(void);

void RGB_Set_Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void RGB_Set_Color_U32(uint8_t index, uint32_t color);
void RGB_Clear(void);


#endif /* _APP_RGB_H_ */

/******************************** END *****************************************/
