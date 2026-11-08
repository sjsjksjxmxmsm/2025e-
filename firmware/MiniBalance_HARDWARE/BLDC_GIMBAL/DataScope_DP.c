#include "DataScope_DP.h"
#include <string.h>

/* 云台串口：USART1 (PA9 = TX, PA10 = RX)，对应板上 H4 的 TXD1 / RXD1 */
#define BLDC_USART_BAUDRATE 115200

volatile BLDC_MotorData_t BLDC_Motor1 = {0};
volatile BLDC_MotorData_t BLDC_Motor2 = {0};

static uint8_t rx_buf[10];
static uint8_t rx_index = 0;
static uint8_t rx_state = 0;

void DataScope_DP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = BLDC_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

uint8_t Calc_BCC(uint8_t *data, uint8_t len)
{
	uint8_t bcc = 0;
	uint8_t i;

	for(i = 0; i < len; i++)
	{
		bcc ^= data[i];
	}

	return bcc;
}

static void BLDC_USART1_SendByte(uint8_t data)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, data);
}

void BLDC_USART1_SendArray(uint8_t *data, uint8_t len)
{
	uint8_t i;

	for(i = 0; i < len; i++)
	{
		BLDC_USART1_SendByte(data[i]);
	}
}

void BLDC_SendCmd(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t len)
{
	uint8_t tx_buf[20];
	uint8_t idx = 0;

	tx_buf[idx++] = BLDC_HEADER;
	tx_buf[idx++] = addr;
	tx_buf[idx++] = cmd;

	if(len > 0 && data != 0)
	{
		memcpy(&tx_buf[idx], data, len);
		idx += len;
	}

	tx_buf[idx++] = Calc_BCC(tx_buf, idx);
	tx_buf[idx++] = BLDC_TAIL;
	BLDC_USART1_SendArray(tx_buf, idx);
}

void BLDC_Enable(uint8_t addr)  { BLDC_SendCmd(addr, CMD_ENABLE, 0, 0); }
void BLDC_Disable(uint8_t addr) { BLDC_SendCmd(addr, CMD_DISABLE, 0, 0); }

void BLDC_SetSpeed(uint8_t addr, int16_t rpm)
{
	uint8_t data[2];

	data[0] = (rpm >> 8) & 0xFF;
	data[1] = rpm & 0xFF;
	BLDC_SendCmd(addr, CMD_SPEED, data, 2);
}

void BLDC_SetMode(uint8_t addr, uint16_t mode)
{
	uint8_t data[2];

	data[0] = (mode >> 8) & 0xFF;
	data[1] = mode & 0xFF;
	BLDC_SendCmd(addr, CMD_MODE, data, 2);
}

void BLDC_SetMultiAngle(uint8_t addr, int32_t angle_x10)
{
	uint8_t data[4];

	data[0] = (angle_x10 >> 24) & 0xFF;
	data[1] = (angle_x10 >> 16) & 0xFF;
	data[2] = (angle_x10 >> 8) & 0xFF;
	data[3] = angle_x10 & 0xFF;
	BLDC_SendCmd(addr, CMD_MULTI_POS, data, 4);
}

void BLDC_SetSingleAngle(uint8_t addr, uint16_t angle_x10)
{
	uint8_t data[2];

	if(angle_x10 > 3599)
	{
		angle_x10 = 3599;
	}

	data[0] = (angle_x10 >> 8) & 0xFF;
	data[1] = angle_x10 & 0xFF;
	BLDC_SendCmd(addr, CMD_SINGLE_POS, data, 2);
}

void BLDC_ReqFeedback(uint8_t addr, uint8_t type)
{
	uint8_t data[1];

	data[0] = type;
	BLDC_SendCmd(addr, CMD_FEEDBACK, data, 1);
}

void BLDC_SetAcc(uint8_t addr, uint16_t acc)
{
	uint8_t data[2];

	data[0] = (acc >> 8) & 0xFF;
	data[1] = acc & 0xFF;
	BLDC_SendCmd(addr, CMD_ACC, data, 2);
}

void BLDC_SaveParams(uint8_t addr)
{
	BLDC_SendCmd(addr, CMD_SAVE, 0, 0);
}

void BLDC_ClearMultiAngle(uint8_t addr)
{
	BLDC_SendCmd(addr, CMD_CLEAR_MULTI, 0, 0);
}

void BLDC_SetSingleAngleZero(uint8_t addr)
{
	BLDC_SendCmd(addr, CMD_SET_ZERO, 0, 0);
}

void BLDC_FactoryReset(uint8_t addr)
{
	BLDC_SendCmd(addr, CMD_FACTORY_RST, 0, 0);
}

void BLDC_SetAddress(uint8_t addr, uint8_t new_addr)
{
	uint8_t data[1];

	data[0] = new_addr;
	BLDC_SendCmd(addr, CMD_SET_ADDR, data, 1);
}

void BLDC_ParseRxData(uint8_t rx_byte)
{
	uint8_t calc_bcc;
	uint8_t addr;
	uint8_t type;
	int32_t value;
	volatile BLDC_MotorData_t *motor;

	switch(rx_state)
	{
		case 0:
			if(rx_byte == BLDC_HEADER)
			{
				rx_buf[0] = rx_byte;
				rx_index = 1;
				rx_state = 1;
			}
			break;

		case 1:
			if(rx_byte == BLDC_ADDR_1 || rx_byte == BLDC_ADDR_2)
			{
				rx_buf[rx_index++] = rx_byte;
				rx_state = 2;
			}
			else
			{
				rx_state = 0;
			}
			break;

		case 2:
			if(rx_byte <= FB_VOLTAGE)
			{
				rx_buf[rx_index++] = rx_byte;
				rx_state = 3;
			}
			else
			{
				rx_state = 0;
			}
			break;

		case 3:
			rx_buf[rx_index++] = rx_byte;
			if(rx_index >= 7)
			{
				rx_state = 4;
			}
			break;

		case 4:
			rx_buf[rx_index++] = rx_byte;
			rx_state = 5;
			break;

		case 5:
			if(rx_byte == BLDC_TAIL)
			{
				rx_buf[rx_index++] = rx_byte;
				calc_bcc = Calc_BCC(rx_buf, 7);

				if(calc_bcc == rx_buf[7])
				{
					addr = rx_buf[1];
					type = rx_buf[2];
					value = ((int32_t)rx_buf[3] << 24) |
					        ((int32_t)rx_buf[4] << 16) |
					        ((int32_t)rx_buf[5] << 8) |
					         (int32_t)rx_buf[6];
					motor = (addr == BLDC_ADDR_1) ? &BLDC_Motor1 : &BLDC_Motor2;

					switch(type)
					{
						case FB_SPEED:
							motor->speed = (int16_t)(value & 0xFFFF);
							break;
						case FB_MULTI_ANGLE:
							motor->multi_angle = value;
							break;
						case FB_SINGLE_ANGLE:
							motor->single_angle = (uint16_t)(value & 0xFFFF);
							break;
						case FB_ACC:
							motor->acc = (int16_t)(value & 0xFFFF);
							break;
						case FB_VOLTAGE:
							motor->voltage = (uint16_t)(value & 0xFFFF);
							break;
						default:
							break;
					}

					motor->data_ready = 1;
				}
			}
			rx_state = 0;
			rx_index = 0;
			break;

		default:
			rx_state = 0;
			rx_index = 0;
			break;
	}
}

void USART1_IRQHandler(void)
{
	uint16_t status;
	uint8_t rx_data;

	status = USART1->SR;
	if(status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
	{
		rx_data = (uint8_t)USART1->DR;
		rx_state = 0;
		rx_index = 0;
		(void)rx_data;
		return;
	}

	if(status & USART_SR_RXNE)
	{
		rx_data = (uint8_t)USART1->DR;
		BLDC_ParseRxData(rx_data);
	}
}
