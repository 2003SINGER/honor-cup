#ifndef BSP_OLED_H_
#define BSP_OLED_H_
#include "stdint.h"
#include "bsp_ssd1306.h"


/* OLED Clear 清除屏幕 */
void OLED_Clear(void);
/* OLED Refresh 刷新屏幕 */
void OLED_Refresh(void);
/* Draw String 写入字符 */
void OLED_Draw_String(char *data, uint8_t x, uint8_t y, uint8_t clear, uint8_t refresh);
/* Draw Line 写入一行字符 */
void OLED_Draw_Line(char *data, uint8_t line, uint8_t clear, uint8_t refresh);


#endif /* BSP_OLED_H_ */
