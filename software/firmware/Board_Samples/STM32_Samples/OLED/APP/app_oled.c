#include "app_oled.h"
#include "bsp_ssd1306.h"

/* OLED Clear 清除屏幕 */
void OLED_Clear(void)
{
    SSD1306_Fill(SSD1306_COLOR_BLACK);
}

/* OLED Refresh 刷新屏幕 */
void OLED_Refresh(void)
{
    SSD1306_UpdateScreen();
}

/* Draw String 写入字符 */
void OLED_Draw_String(char *data, uint8_t x, uint8_t y, uint8_t clear, uint8_t refresh)
{
    if (clear) OLED_Clear();
    SSD1306_GotoXY(x, y);
    SSD1306_Puts(data, &Font_7x10, SSD1306_COLOR_WHITE);
    if (refresh) OLED_Refresh();
}

/* Draw Line 写入一行字符 */
void OLED_Draw_Line(char *data, uint8_t line, uint8_t clear, uint8_t refresh)
{
    if (line > 0 && line <= 3)
    {
        OLED_Draw_String(data, 0, 10 * (line - 1), clear, refresh);
    }
}



