#include "h30_imu.h"

/* H30 模块挂在 I2C1 上，以下为设备地址、数据寄存器地址和各数据块长度。 */
#define H30_IMU_I2C                 I2C1
#define H30_IMU_I2C_ADDR            ((u8)0x6A)
#define H30_IMU_REG_ACC             ((u8)0x10)
#define H30_IMU_REG_GYRO            ((u8)0x20)
#define H30_IMU_REG_EULER           ((u8)0x40)
#define H30_IMU_REG_QUATERNION      ((u8)0x41)
#define H30_IMU_ACC_LEN             ((u8)12)
#define H30_IMU_GYRO_LEN            ((u8)12)
#define H30_IMU_EULER_LEN           ((u8)12)
#define H30_IMU_QUATERNION_LEN      ((u8)16)
#define H30_IMU_DATA_FACTOR         0.000001f
#define H30_IMU_TIMEOUT             ((u32)50000)

/* IMU 运行状态标志：data_ready 表示有新数据，last_error 表示最近一次 I2C 读取失败。 */
volatile u8 h30_imu_data_ready = 0;
volatile u8 h30_imu_last_error = 0;
volatile u16 h30_imu_sample_tick = 0;
volatile u16 h30_imu_sample_seconds = 0;

/* 全局 IMU 数据缓存，控制和显示模块直接读取这里的数据。 */
H30_IMU_Data h30_imu_data;

/* H30 输出为小端序 32 位有符号整数，这里把 4 个字节合成为 s32。 */
static s32 H30_IMU_ToS32(u8 *buf)
{
	return ((s32)buf[0]) |
	       ((s32)buf[1] << 8) |
	       ((s32)buf[2] << 16) |
	       ((s32)buf[3] << 24);
}

/* 等待 I2C 指定事件，超时返回 0，避免总线异常时死等。 */
static u8 H30_IMU_WaitEvent(u32 event)
{
	u32 timeout = H30_IMU_TIMEOUT;

	while(I2C_CheckEvent(H30_IMU_I2C, event) != SUCCESS)
	{
		if(timeout-- == 0)
		{
			return 0;
		}
	}

	return 1;
}

/* 等待 I2C 指定标志变为目标状态，超时返回 0。 */
static u8 H30_IMU_WaitFlag(FlagStatus status, u32 flag)
{
	u32 timeout = H30_IMU_TIMEOUT;

	while(I2C_GetFlagStatus(H30_IMU_I2C, flag) != status)
	{
		if(timeout-- == 0)
		{
			return 0;
		}
	}

	return 1;
}

/* 从 H30 指定寄存器连续读取 len 个字节，成功返回 1，失败返回 0。 */
static u8 H30_IMU_ReadReg(u8 reg, u8 *buf, u8 len)
{
	u8 index = 0;

	if(len == 0)
	{
		return 1;
	}
	if(len < 3)
	{
		return 0;
	}

	/* 等待总线空闲后开始一次“写寄存器地址 + 重启 + 读数据”的标准 I2C 读流程。 */
	if(!H30_IMU_WaitFlag(RESET, I2C_FLAG_BUSY))
	{
		goto error;
	}

	I2C_GenerateSTART(H30_IMU_I2C, ENABLE);
	if(!H30_IMU_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		goto error;
	}

	I2C_Send7bitAddress(H30_IMU_I2C, H30_IMU_I2C_ADDR, I2C_Direction_Transmitter);
	if(!H30_IMU_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		goto error;
	}

	I2C_SendData(H30_IMU_I2C, reg);
	if(!H30_IMU_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		goto error;
	}

	I2C_GenerateSTART(H30_IMU_I2C, ENABLE);
	if(!H30_IMU_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		goto error;
	}

	I2C_Send7bitAddress(H30_IMU_I2C, H30_IMU_I2C_ADDR, I2C_Direction_Receiver);
	if(!H30_IMU_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		goto error;
	}

	/* 前面的字节逐个 ACK 接收，最后 3 字节按 STM32 I2C 推荐时序单独处理。 */
	while(len > 3)
	{
		if(!H30_IMU_WaitFlag(SET, I2C_FLAG_RXNE))
		{
			goto error;
		}
		buf[index++] = I2C_ReceiveData(H30_IMU_I2C);
		len--;
	}

	if(!H30_IMU_WaitFlag(SET, I2C_FLAG_BTF))
	{
		goto error;
	}
	I2C_AcknowledgeConfig(H30_IMU_I2C, DISABLE);
	/* 关闭 ACK 后读取倒数第 3 字节，随后等待最后 2 字节进入移位/数据寄存器。 */
	buf[index++] = I2C_ReceiveData(H30_IMU_I2C);
	len--;

	if(!H30_IMU_WaitFlag(SET, I2C_FLAG_BTF))
	{
		goto error;
	}
	I2C_GenerateSTOP(H30_IMU_I2C, ENABLE);
	/* 产生 STOP 后读出最后两个字节，再恢复 ACK，保证下一次读取正常。 */
	buf[index++] = I2C_ReceiveData(H30_IMU_I2C);
	buf[index] = I2C_ReceiveData(H30_IMU_I2C);

	I2C_AcknowledgeConfig(H30_IMU_I2C, ENABLE);
	return 1;

error:
	/* 任意一步失败都释放总线并恢复 ACK，避免影响后续 I2C 访问。 */
	I2C_GenerateSTOP(H30_IMU_I2C, ENABLE);
	I2C_AcknowledgeConfig(H30_IMU_I2C, ENABLE);
	return 0;
}

/* I2C1 初始化：PB8/PB9 重映射为 SCL/SDA，开漏复用输出，400kHz。 */
static void H30_IMU_I2C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_DeInit(H30_IMU_I2C);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_Init(H30_IMU_I2C, &I2C_InitStructure);
	I2C_Cmd(H30_IMU_I2C, ENABLE);
}

/* TIM2 初始化为 5ms 周期中断，与主控制周期同步读取 IMU 偏航角速度。 */
static void H30_IMU_TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 49;
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM2, ENABLE);
}

/* H30 初始化：先配置 I2C，等待模块稳定后启动 TIM2 周期读取。 */
void H30_IMU_Init(void)
{
	H30_IMU_I2C_Init();
	delay_ms(500);
	H30_IMU_TIM2_Init();
}

/* TIM2 中断中的 IMU 回调：只读 gyro_z，给循迹直线修正和云台偏航补偿使用。 */
void H30_IMU_TIM_Callback(void)
{
	h30_imu_last_error = H30_IMU_ReadYawGyro() ? 0 : 1;
}

/* 快速读取 z 轴角速度，减少中断内 I2C 读取时间。 */
u8 H30_IMU_ReadYawGyro(void)
{
	u8 gyro_buf[H30_IMU_GYRO_LEN];

	if(!H30_IMU_ReadReg(H30_IMU_REG_GYRO, gyro_buf, H30_IMU_GYRO_LEN))
	{
		return 0;
	}

	/* gyro_z 位于陀螺仪数据的第 8~11 字节，乘比例系数转为浮点值。 */
	h30_imu_data.gyro_z = H30_IMU_ToS32(gyro_buf + 8) * H30_IMU_DATA_FACTOR;
	h30_imu_data_ready = 1;
	return 1;
}

/* 完整读取欧拉角、加速度、角速度和四元数，适合调试或低频状态显示。 */
u8 H30_IMU_ReadAll(void)
{
	u8 euler_buf[H30_IMU_EULER_LEN];
	u8 acc_buf[H30_IMU_ACC_LEN];
	u8 gyro_buf[H30_IMU_GYRO_LEN];
	u8 quaternion_buf[H30_IMU_QUATERNION_LEN];

	if(!H30_IMU_ReadReg(H30_IMU_REG_EULER, euler_buf, H30_IMU_EULER_LEN))
	{
		return 0;
	}
	if(!H30_IMU_ReadReg(H30_IMU_REG_ACC, acc_buf, H30_IMU_ACC_LEN))
	{
		return 0;
	}
	if(!H30_IMU_ReadReg(H30_IMU_REG_GYRO, gyro_buf, H30_IMU_GYRO_LEN))
	{
		return 0;
	}
	if(!H30_IMU_ReadReg(H30_IMU_REG_QUATERNION, quaternion_buf, H30_IMU_QUATERNION_LEN))
	{
		return 0;
	}

	/* 欧拉角：pitch、roll、yaw，每个量 4 字节小端有符号整数。 */
	h30_imu_data.pitch = H30_IMU_ToS32(euler_buf) * H30_IMU_DATA_FACTOR;
	h30_imu_data.roll = H30_IMU_ToS32(euler_buf + 4) * H30_IMU_DATA_FACTOR;
	h30_imu_data.yaw = H30_IMU_ToS32(euler_buf + 8) * H30_IMU_DATA_FACTOR;

	/* 三轴加速度。 */
	h30_imu_data.acc_x = H30_IMU_ToS32(acc_buf) * H30_IMU_DATA_FACTOR;
	h30_imu_data.acc_y = H30_IMU_ToS32(acc_buf + 4) * H30_IMU_DATA_FACTOR;
	h30_imu_data.acc_z = H30_IMU_ToS32(acc_buf + 8) * H30_IMU_DATA_FACTOR;

	/* 三轴角速度。 */
	h30_imu_data.gyro_x = H30_IMU_ToS32(gyro_buf) * H30_IMU_DATA_FACTOR;
	h30_imu_data.gyro_y = H30_IMU_ToS32(gyro_buf + 4) * H30_IMU_DATA_FACTOR;
	h30_imu_data.gyro_z = H30_IMU_ToS32(gyro_buf + 8) * H30_IMU_DATA_FACTOR;

	/* 四元数姿态数据。 */
	h30_imu_data.q0 = H30_IMU_ToS32(quaternion_buf) * H30_IMU_DATA_FACTOR;
	h30_imu_data.q1 = H30_IMU_ToS32(quaternion_buf + 4) * H30_IMU_DATA_FACTOR;
	h30_imu_data.q2 = H30_IMU_ToS32(quaternion_buf + 8) * H30_IMU_DATA_FACTOR;
	h30_imu_data.q3 = H30_IMU_ToS32(quaternion_buf + 12) * H30_IMU_DATA_FACTOR;

	h30_imu_data_ready = 1;
	return 1;
}

/* 预留处理接口，目前数据读取由 TIM2 回调直接完成。 */
void H30_IMU_Process(void)
{
}
