#include "bluetooth_usart.h"

/* ============================================================================
 * K230 视觉数据接收（USART3 默认引脚）
 *   接口：USART3，PB11 = RX（只接收，不需要 PB10）
 *   位置：板上 GX_SDA（温湿度芯片旁边那根），已用自发自收测试验证通过
 *   DMA ：DMA1_Channel3（USART3_RX 的固定映射）
 *   协议：8 字节定长帧  0x7B  XH XL  YH YL  AH AL  0x7D
 *
 * 为什么选 PB11：
 *   - 是 USART3 的【默认】RX 脚，不需要任何重映射
 *   - 标准外设：中断向量 39 号、DMA1_Channel3（13 号）都在中容量向量表里
 *   - 板上唯一挂的是温湿度芯片的 SDA（I2C 从机，平时高阻）+ 4.7k 上拉，
 *     没有任何芯片主动驱动这根线，实测收发 50fps 零错误
 *
 * 注意：PB10（USART3_TX / GX_SCL）故意不配置，保持输入。
 *   这样 SCL 一直是高电平，温湿度芯片不会认出 I2C 时序去拉低 SDA。
 * ==========================================================================*/

#define K230_FRAME_LEN   8
#define K230_HEAD        0x7B
#define K230_TAIL        0x7D

volatile u8  bluetooth_rx_data  = 0;
volatile u8  bluetooth_rx_flag  = 0;
volatile u16 bluetooth_rx_count = 0;
u8 show_data = 0;
u8 PID_Send  = 0;

volatile int16_t g_follow_x = 0;
volatile int16_t g_follow_y = 0;
volatile u32 g_follow_area = 0;
volatile u8  g_data_ready = 0;
volatile u8  g_vision_frame_flag = 0;
volatile u16 g_vision_frame_count = 0;

u8 Packet[K230_FRAME_LEN];

static void K230_DMA_Restart(void)
{
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel3, K230_FRAME_LEN);
	DMA_Cmd(DMA1_Channel3, ENABLE);
}

static void K230_ParsePacket(void)
{
	u16 cx_u;
	u16 cy_u;
	u16 area_u;

	if(Packet[0] != K230_HEAD || Packet[7] != K230_TAIL)
	{
		return;
	}

	cx_u = ((u16)Packet[1] << 8) | Packet[2];
	cy_u = ((u16)Packet[3] << 8) | Packet[4];
	area_u = ((u16)Packet[5] << 8) | Packet[6];

	/* 只有"有目标"的帧才更新坐标；没目标的帧保留最后一次有效位置。
	   否则 K230 丢目标时发的 (0,0) 会覆盖掉最后位置，
	   导致"回到最后位置"的逻辑朝错误方向跑。 */
	if(area_u != 0)
	{
		g_follow_x = (int16_t)(cx_u - 32768);
		g_follow_y = (int16_t)(cy_u - 32768);
	}

	g_follow_area = area_u * 10;
	g_data_ready = 1;
	g_vision_frame_flag = 1;
	g_vision_frame_count++;
	bluetooth_rx_flag = 1;
	bluetooth_rx_count++;
}

void Bluetooth_USART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* PB11 = USART3_RX，浮空输入（板上已有 4.7k 上拉） */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(USART3, &USART_InitStructure);

	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)Packet;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = K230_FRAME_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART3, ENABLE);
	K230_DMA_Restart();
}

/* DMA 收满 8 字节 */
void DMA1_Channel3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC3) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_IT_TC3);
		DMA_Cmd(DMA1_Channel3, DISABLE);
		K230_ParsePacket();
		K230_DMA_Restart();
	}
}

/* 空闲中断：半帧时重新对齐；溢出时清掉重来 */
void USART3_IRQHandler(void)
{
	u16 dummy;
	u16 dma_left;

	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		dummy = USART3->SR;
		dummy = USART3->DR;
		(void)dummy;

		dma_left = DMA_GetCurrDataCounter(DMA1_Channel3);
		if(dma_left != 0 && dma_left != K230_FRAME_LEN)
		{
			K230_DMA_Restart();
		}
	}

	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
	{
		bluetooth_rx_data = USART_ReceiveData(USART3);
		show_data = bluetooth_rx_data;
		K230_DMA_Restart();
	}
}

void Bluetooth_SendByte(u8 data)     { (void)data; }
void Bluetooth_SendString(char *str) { (void)str; }
u8 Bluetooth_Available(void)         { return g_data_ready; }
u8 Bluetooth_ReadByte(void)          { g_data_ready = 0; return 0; }

void Bluetooth_ClearRxBuffer(void)
{
	g_data_ready = 0;
	bluetooth_rx_flag = 0;
	K230_DMA_Restart();
}

void APP_Show(void)
{
}
