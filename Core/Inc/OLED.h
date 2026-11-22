#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

/* 0.96 寸 SSD1306 OLED（128x64），软件 I2C
   接线：VCC -> 3.3V，GND -> GND，SCL -> PB1，SDA -> PB0
   用法：OLED_Init(); 然后 OLED_ShowString(行, 列, "文本");
   行 1~4，列 1~16（8x16 字模，一屏 4 行 16 列）。
   驱动来源：江协科技 OLED 例程改写。 */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
uint8_t OLED_IsReady(void);   /* 1 = 上电自检时屏幕有应答 */

#endif
