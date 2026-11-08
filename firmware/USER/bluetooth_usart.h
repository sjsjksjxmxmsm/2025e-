#ifndef __BLUETOOTH_USART_H
#define __BLUETOOTH_USART_H

#include "sys.h"

#define BLUETOOTH_RX_BUF_SIZE 64

extern volatile u8 bluetooth_rx_data;
extern volatile u8 bluetooth_rx_flag;
extern volatile u16 bluetooth_rx_count;
extern u8 show_data;
extern u8 PID_Send;
extern volatile int16_t g_follow_x;
extern volatile int16_t g_follow_y;
extern volatile u32 g_follow_area;   /* u32：area_u*10 最大可到 655350，u16 会溢出 */
extern volatile u8 g_data_ready;
extern volatile u8 g_vision_frame_flag;
extern volatile u16 g_vision_frame_count;
extern u8 Packet[8];

void Bluetooth_USART2_Init(void);
void Bluetooth_SendByte(u8 data);
void Bluetooth_SendString(char *str);
u8 Bluetooth_Available(void);
u8 Bluetooth_ReadByte(void);
void Bluetooth_ClearRxBuffer(void);
void APP_Show(void);
#endif
