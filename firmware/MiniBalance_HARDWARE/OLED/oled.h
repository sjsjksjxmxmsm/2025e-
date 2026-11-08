#ifndef __OLED_H
#define __OLED_H
#include "sys.h"

/* ===========================================================================
 * OLED：SH1106，4 线 SPI（无 CS）
 *   引脚（江协 32 核心板，从原理图读出）：
 *     SCLK -> PB3      DC  -> PB4
 *     SDIN -> PB5      RES -> PB8（板上有 10k 下拉）
 *   要换引脚只改 oled.c 顶部那 4 个宏。
 *   SH1106 是 132 列显存，比屏多 2 列，所以刷新时列地址要 +2。
 * ==========================================================================*/

extern u8 OLED_GRAM[128][8];

#define OLED_CMD  0
#define OLED_DATA 1

void OLED_WR_Byte(u8 dat,u8 cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);

#endif
