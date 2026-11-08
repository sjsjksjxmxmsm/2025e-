#ifndef __DATASCOPE_DP_H
#define __DATASCOPE_DP_H

#include "sys.h"
#include <stdint.h>

#define BLDC_HEADER     0x7A
#define BLDC_TAIL       0x7B

#define BLDC_ADDR_1     0x01
#define BLDC_ADDR_2     0x02
/*
 * 本机云台实测：2 号电机是 X 轴，1 号电机是 Y 轴（与手册默认相反）。
 * 下面按轴定义别名，业务代码只用这两个宏，不要再直接写 BLDC_ADDR_1/2。
 *   BLDC_ADDR_X_AXIS = 偏航轴（水平旋转，速度模式）
 *   BLDC_ADDR_Y_AXIS = 俯仰轴（上下转动，单圈角度模式）
 */
#define BLDC_ADDR_X_AXIS    BLDC_ADDR_2
#define BLDC_ADDR_Y_AXIS    BLDC_ADDR_1

#define CMD_ENABLE      0x06
#define CMD_DISABLE     0x05
#define CMD_MODE        0x00
#define CMD_SPEED       0x01
#define CMD_MULTI_POS   0x02
#define CMD_SINGLE_POS  0x03
#define CMD_FEEDBACK    0x0E
#define CMD_ACC         0x07
#define CMD_SAVE        0x08
#define CMD_CLEAR_MULTI 0x09
#define CMD_SET_ZERO    0x0A
#define CMD_FACTORY_RST 0x0B
#define CMD_SET_ADDR    0x0D

#define MODE_SPEED          0x0000
#define MODE_MULTI_POS      0x0001
#define MODE_SINGLE_POS     0x0002
#define MODE_MULTI_POS_L      0x0003
#define MODE_SINGLE_POS_L     0x0004

#define FB_SPEED        0x00
#define FB_MULTI_ANGLE  0x01
#define FB_SINGLE_ANGLE 0x02
#define FB_ACC          0x03
#define FB_VOLTAGE      0x04

typedef struct {
	int16_t  speed;
	int32_t  multi_angle;
	uint16_t single_angle;
	int16_t  acc;
	uint16_t voltage;
	uint8_t  data_ready;
} BLDC_MotorData_t;

extern volatile BLDC_MotorData_t BLDC_Motor1;
extern volatile BLDC_MotorData_t BLDC_Motor2;

void DataScope_DP_Init(void);
uint8_t Calc_BCC(uint8_t *data, uint8_t len);
void BLDC_USART3_SendArray(uint8_t *data, uint8_t len);

void BLDC_SendCmd(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t len);
void BLDC_Enable(uint8_t addr);
void BLDC_Disable(uint8_t addr);
void BLDC_SetSpeed(uint8_t addr, int16_t rpm);
void BLDC_SetMode(uint8_t addr, uint16_t mode);
void BLDC_ReqFeedback(uint8_t addr, uint8_t type);
void BLDC_SetMultiAngle(uint8_t addr, int32_t angle_x10);
void BLDC_SetSingleAngle(uint8_t addr, uint16_t angle_x10);
void BLDC_SetAcc(uint8_t addr, uint16_t acc);
void BLDC_SaveParams(uint8_t addr);
void BLDC_ClearMultiAngle(uint8_t addr);
void BLDC_SetSingleAngleZero(uint8_t addr);
void BLDC_FactoryReset(uint8_t addr);
void BLDC_SetAddress(uint8_t addr, uint8_t new_addr);
void BLDC_ParseRxData(uint8_t rx_byte);

#endif
