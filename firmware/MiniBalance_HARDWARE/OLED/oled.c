#include "oled.h"
#include <string.h>
#include "oledfont.h"

/* ===========================================================================
 * SH1106  4 线 SPI 驱动（软件模拟 SPI）
 *   SCLK -> PB3     DC -> PB4     SDIN -> PB5     RES -> PB8
 *   板子原理图：61 PB8=OLED_RES，56 PB4=OLED_DC，55 PB3=OLED_SCLK，57 PB5=OLED_SDIN
 * ==========================================================================*/
#define OLED_PORT        GPIOB
#define OLED_SCLK_PIN    GPIO_Pin_3
#define OLED_SDIN_PIN    GPIO_Pin_5
#define OLED_DC_PIN      GPIO_Pin_4
#define OLED_RES_PIN     GPIO_Pin_8

#define OLED_COL_OFFSET  4      /* SH1106 是 132 列显存，比 128 的屏多 2 列 */
#define OLED_FLIP_V      1      /* 上下反了就改成 1 */

#define OLED_SCLK_H()    GPIO_SetBits  (OLED_PORT, OLED_SCLK_PIN)
#define OLED_SCLK_L()    GPIO_ResetBits(OLED_PORT, OLED_SCLK_PIN)
#define OLED_SDIN_H()    GPIO_SetBits  (OLED_PORT, OLED_SDIN_PIN)
#define OLED_SDIN_L()    GPIO_ResetBits(OLED_PORT, OLED_SDIN_PIN)
#define OLED_DC_H()      GPIO_SetBits  (OLED_PORT, OLED_DC_PIN)
#define OLED_DC_L()      GPIO_ResetBits(OLED_PORT, OLED_DC_PIN)
#define OLED_RES_H()     GPIO_SetBits  (OLED_PORT, OLED_RES_PIN)
#define OLED_RES_L()     GPIO_ResetBits(OLED_PORT, OLED_RES_PIN)

u8 OLED_GRAM[128][8];

static u32 oled_pow(u8 m, u8 n)
{
	u32 r = 1;
	while(n--) r *= m;
	return r;
}

/* 写一个字节：cmd=1 为数据，0 为命令 */
void OLED_WR_Byte(u8 dat, u8 cmd)
{
	u8 i;

	if(cmd) OLED_DC_H();
	else    OLED_DC_L();

	for(i = 0; i < 8; i++)
	{
		OLED_SCLK_L();
		if(dat & 0x80) OLED_SDIN_H();
		else           OLED_SDIN_L();
		OLED_SCLK_H();
		dat <<= 1;
	}
	OLED_DC_H();
}

void OLED_Refresh_Gram(void)
{
	u8 i, n;
	u8 col;

	for(i = 0; i < 8; i++)
	{
		col = i + 0;                        /* 页地址 */
		OLED_WR_Byte(0xB0 | col, OLED_CMD);
		col = OLED_COL_OFFSET;              /* 列地址从 2 开始 */
		OLED_WR_Byte(0x00 | (col & 0x0F), OLED_CMD);
		OLED_WR_Byte(0x10 | ((col >> 4) & 0x0F), OLED_CMD);

		for(n = 0; n < 128; n++)
		{
			OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
		}
	}
}

void OLED_Clear(void)
{
	u8 i, n;
	for(i = 0; i < 8; i++) for(n = 0; n < 128; n++) OLED_GRAM[n][i] = 0x00;
	OLED_Refresh_Gram();
}

void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
	u8 pos, bx;
	u8 temp;

	if(x > 127 || y > 63) return;

	pos = 7 - y / 8;
	bx  = y % 8;
	temp = 1 << (7 - bx);

	if(t) OLED_GRAM[x][pos] |= temp;
	else   OLED_GRAM[x][pos] &= ~temp;
}

void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode)
{
	u8 temp, t, t1;
	u8 y0 = y;

	chr = chr - ' ';

	for(t = 0; t < size; t++)
	{
		if(size == 12) temp = oled_asc2_1206[chr][t];
		else           temp = oled_asc2_1608[chr][t];

		for(t1 = 0; t1 < 8; t1++)
		{
			if(temp & 0x80) OLED_DrawPoint(x, y, mode);
			else            OLED_DrawPoint(x, y, !mode);
			temp <<= 1;
			y++;
			if((y - y0) == size)
			{
				y = y0;
				x++;
				break;
			}
		}
	}
}

void OLED_ShowNumber(u8 x, u8 y, u32 num, u8 len, u8 size)
{
	u8 t, temp;
	u8 enshow = 0;

	for(t = 0; t < len; t++)
	{
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if(enshow == 0 && t < (len - 1))
		{
			if(temp == 0)
			{
				OLED_ShowChar(x + (size / 2) * t, y, ' ', size, 1);
				continue;
			}
			else enshow = 1;
		}
		OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 1);
	}
}

void OLED_ShowString(u8 x, u8 y, const u8 *p)
{
	while(*p != '\0')
	{
		if(x > 120) { x = 0; y += 16; }
		if(y > 56)  { y = x = 0; }
		OLED_ShowChar(x, y, *p, 12, 1);
		x += 6;
		p++;
	}
}

void OLED_Display_On(void)
{
	OLED_WR_Byte(0x8D, OLED_CMD);
	OLED_WR_Byte(0x14, OLED_CMD);
	OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_Display_Off(void)
{
	OLED_WR_Byte(0x8D, OLED_CMD);
	OLED_WR_Byte(0x10, OLED_CMD);
	OLED_WR_Byte(0xAE, OLED_CMD);
}

void OLED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = OLED_SCLK_PIN | OLED_SDIN_PIN | OLED_DC_PIN | OLED_RES_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OLED_PORT, &GPIO_InitStructure);

	OLED_DC_H();
	OLED_SCLK_H();

	/* 硬件复位：RES 低 -> 高 */
	OLED_RES_H();
	delay_ms(50);
	OLED_RES_L();
	delay_ms(50);
	OLED_RES_H();
	delay_ms(50);

	OLED_WR_Byte(0xAE, OLED_CMD);          /* 关显示 */

	OLED_WR_Byte(0xD5, OLED_CMD);          /* 时钟分频/振荡频率 */
	OLED_WR_Byte(0x80, OLED_CMD);

	OLED_WR_Byte(0xA8, OLED_CMD);          /* 多路复用 */
	OLED_WR_Byte(0x3F, OLED_CMD);

	OLED_WR_Byte(0xD3, OLED_CMD);          /* 显示偏移 */
	OLED_WR_Byte(0x00, OLED_CMD);

	OLED_WR_Byte(0x40, OLED_CMD);          /* 起始行 */

	OLED_WR_Byte(0xA1, OLED_CMD);          /* 左右方向：A1 正常 / A0 反置 */

	if(OLED_FLIP_V) OLED_WR_Byte(0xC0, OLED_CMD);   /* 上下方向：C8 正常 / C0 反置 */
	else            OLED_WR_Byte(0xC8, OLED_CMD);

	OLED_WR_Byte(0xDA, OLED_CMD);          /* COM 引脚配置 */
	OLED_WR_Byte(0x12, OLED_CMD);

	OLED_WR_Byte(0x81, OLED_CMD);          /* 对比度 */
	OLED_WR_Byte(0xCF, OLED_CMD);

	OLED_WR_Byte(0xD9, OLED_CMD);          /* 预充电周期 */
	OLED_WR_Byte(0xF1, OLED_CMD);

	OLED_WR_Byte(0xDB, OLED_CMD);          /* VCOMH */
	OLED_WR_Byte(0x30, OLED_CMD);

	OLED_WR_Byte(0xA4, OLED_CMD);          /* 全屏显示开 */
	OLED_WR_Byte(0xA6, OLED_CMD);          /* 正常显示（非反白） */

	OLED_WR_Byte(0x8D, OLED_CMD);          /* 电荷泵 */
	OLED_WR_Byte(0x14, OLED_CMD);

	OLED_WR_Byte(0xAF, OLED_CMD);          /* 开显示 */

	OLED_Clear();
}
