#include "control.h"
#include "ir_track.h"
#include "bluetooth_usart.h"

/* ============================================================================
 * MPU6050 / MPU6500 陀螺仪（软件 I2C，PB6/PB7）
 *   作用：读偏航角速度，填到 h30_imu_data.gyro_z（单位 度/秒，已减零偏），
 *         使 Gimbal_GetYawCompRpm() 的前馈逻辑自动生效。
 *   接线：SCL->PB6(BL_SCL)  SDA->PB7(BL_SDA)  VCC->3.3V  GND->GND  AD0->GND
 * ==========================================================================*/
#define MPU_SCL_PIN   GPIO_Pin_6
#define MPU_SDA_PIN   GPIO_Pin_7
#define MPU_SCL_H()   GPIO_SetBits(GPIOB, MPU_SCL_PIN)
#define MPU_SCL_L()   GPIO_ResetBits(GPIOB, MPU_SCL_PIN)
#define MPU_SDA_H()   GPIO_SetBits(GPIOB, MPU_SDA_PIN)
#define MPU_SDA_L()   GPIO_ResetBits(GPIOB, MPU_SDA_PIN)
#define MPU_SDA_IN()  GPIO_ReadInputDataBit(GPIOB, MPU_SDA_PIN)

#define MPU_I2C_ADDR        0x68
#define MPU_REG_SMPLRT_DIV  0x19
#define MPU_REG_CONFIG      0x1A
#define MPU_REG_GYRO_CFG    0x1B
#define MPU_REG_ACCEL_CFG   0x1C
#define MPU_REG_GYRO_ZOUT_H 0x47
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_WHO_AM_I    0x75

#define MPU_LSB_PER_DPS     16.4f

float g_mpu_gyro_z = 0.0f;
u8 g_mpu_id = 0;
u8 g_mpu_ok = 0;
static float mpu_gyro_z_bias = 0.0f;
static u8    mpu_ok = 0;
static u32   mpu_last_tick = 0;

static void MPU_Dly(void) { delay_us(4); }

static void MPU_I2C_Init(void)
{
	GPIO_InitTypeDef g;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	g.GPIO_Pin   = MPU_SCL_PIN | MPU_SDA_PIN;
	g.GPIO_Mode  = GPIO_Mode_Out_OD;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);

	MPU_SDA_H();
	MPU_SCL_H();
	delay_ms(5);
}

static void MPU_Start(void)
{
	MPU_SDA_H(); MPU_SCL_H(); MPU_Dly();
	MPU_SDA_L(); MPU_Dly();
	MPU_SCL_L(); MPU_Dly();
}

static void MPU_Stop(void)
{
	MPU_SDA_L(); MPU_SCL_L(); MPU_Dly();
	MPU_SCL_H(); MPU_Dly();
	MPU_SDA_H(); MPU_Dly();
}

static u8 MPU_WriteByte(u8 b)
{
	u8 i, nack;

	for(i = 0; i < 8; i++)
	{
		MPU_SCL_L(); MPU_Dly();
		if(b & 0x80) MPU_SDA_H(); else MPU_SDA_L();
		MPU_Dly();
		MPU_SCL_H(); MPU_Dly();
		b <<= 1;
	}

	MPU_SCL_L(); MPU_Dly();
	MPU_SDA_H(); MPU_Dly();
	MPU_SCL_H(); MPU_Dly();
	nack = MPU_SDA_IN();
	MPU_SCL_L(); MPU_Dly();

	return nack;
}

static u8 MPU_ReadByte(u8 ack)
{
	u8 i, b = 0;

	MPU_SDA_H();
	for(i = 0; i < 8; i++)
	{
		b <<= 1;
		MPU_SCL_L(); MPU_Dly();
		MPU_SCL_H(); MPU_Dly();
		if(MPU_SDA_IN()) b |= 1;
	}

	MPU_SCL_L(); MPU_Dly();
	if(ack) MPU_SDA_L(); else MPU_SDA_H();
	MPU_Dly();
	MPU_SCL_H(); MPU_Dly();
	MPU_SCL_L(); MPU_Dly();
	MPU_SDA_H();

	return b;
}

static u8 MPU_WriteReg(u8 reg, u8 val)
{
	u8 nack;

	MPU_Start();
	nack  = MPU_WriteByte((u8)(MPU_I2C_ADDR << 1));
	nack |= MPU_WriteByte(reg);
	nack |= MPU_WriteByte(val);
	MPU_Stop();

	return (nack == 0) ? 1 : 0;
}

static u8 MPU_ReadBuf(u8 reg, u8 *buf, u8 len)
{
	u8 i, nack;

	MPU_Start();
	nack = MPU_WriteByte((u8)(MPU_I2C_ADDR << 1));
	if(nack) { MPU_Stop(); return 0; }

	nack = MPU_WriteByte(reg);
	if(nack) { MPU_Stop(); return 0; }

	MPU_Start();
	nack = MPU_WriteByte((u8)((MPU_I2C_ADDR << 1) | 1));
	if(nack) { MPU_Stop(); return 0; }

	for(i = 0; i < len; i++)
	{
		buf[i] = MPU_ReadByte((i < (u8)(len - 1)) ? 1 : 0);
	}
	MPU_Stop();

	return 1;
}

void MPU6050_Init(void)
{
	u8  id = 0;
	u8  buf[2];
	int i;
	int sum = 0;

	MPU_I2C_Init();
	MPU_ReadBuf(MPU_REG_WHO_AM_I, &id, 1);
	g_mpu_id = id;

	if(id == 0x00 || id == 0xFF)
	{
		mpu_ok = 0;
		g_mpu_ok = 0;
		h30_imu_data.gyro_z = 0.0f;
		g_mpu_gyro_z = 0.0f;
		return;
	}

	MPU_WriteReg(MPU_REG_PWR_MGMT_1, 0x80);
	delay_ms(100);
	MPU_WriteReg(MPU_REG_PWR_MGMT_1, 0x00);
	delay_ms(50);
	MPU_WriteReg(MPU_REG_SMPLRT_DIV, 0x04);
	MPU_WriteReg(MPU_REG_CONFIG, 0x03);
	MPU_WriteReg(MPU_REG_GYRO_CFG, 0x18);
	MPU_WriteReg(MPU_REG_ACCEL_CFG, 0x10);
	MPU_WriteReg(MPU_REG_PWR_MGMT_1, 0x01);
	delay_ms(100);

	for(i = 0; i < 50; i++)
	{
		if(MPU_ReadBuf(MPU_REG_GYRO_ZOUT_H, buf, 2))
		{
			sum += (s16)(((u16)buf[0] << 8) | buf[1]);
		}
		delay_ms(2);
	}
	mpu_gyro_z_bias = (float)sum / 50.0f;

	mpu_ok = 1;
	g_mpu_ok = 1;
}

void MPU6050_Update(void)
{
	u8  buf[2];
	u32 now;

	if(!mpu_ok) return;

	now = User_GetSysTickTime();
	if((now - mpu_last_tick) < 5) return;
	mpu_last_tick = now;

	if(MPU_ReadBuf(MPU_REG_GYRO_ZOUT_H, buf, 2))
	{
		s16 raw = (s16)(((u16)buf[0] << 8) | buf[1]);
		g_mpu_gyro_z = ((float)raw - mpu_gyro_z_bias) / MPU_LSB_PER_DPS;
		h30_imu_data.gyro_z = g_mpu_gyro_z;
	}
}

/* 整数开方（避免引入 math 库） */
static u32 isqrt_u32(u32 v)
{
	u32 x = 0;
	u32 b = 1u << 30;

	while(b > v) b >>= 2;

	while(b)
	{
		if(v >= x + b)
		{
			v -= x + b;
			x = (x >> 1) + b;
		}
		else
		{
			x >>= 1;
		}
		b >>= 2;
	}

	return x;
}



/* 电机闭环输出和工作状态：模式选择与启动使能分开管理。 */
int Motor_Left = 0;
int Motor_Right = 0;
u8 Control_Work_Mode = CONTROL_MODE_SEARCH;
u8 Control_Work_Enable = 0;
volatile Motor_Parameter MotorA;
volatile Motor_Parameter MotorB;

/* 速度环 PI 参数和云台 IMU 偏航补偿系数。 */
float Velocity_KP = 400.0f;
float Velocity_KI = 400.0f;
float Gimbal_Yaw_Rpm_K = -1.0f / 6.0f;
volatile int16_t Gimbal_Joint1_Target_Rpm = 0;
volatile int16_t Gimbal_Joint2_Target_Rpm = 0;
volatile int32_t Gimbal_Joint2_Target_Angle = 0;   /* 0.1 度，可为负，不回绕 */
volatile u16 Vision_Fps = 0;
volatile u8 Vision_Online = 0;
/* 视觉误差方向、死区、限幅以及可调参数步进。 */
#define VISION_DIR_X               -1.0f  /* 图像 X 误差到偏航输出的方向符号，方向反时只改这里。 */
#define VISION_DIR_Y               1.0f  /* 图像 Y 误差到俯仰输出的方向符号，方向反时只改这里。 */
#define VISION_DEADZONE            2.0f  /* 目标中心附近的像素死区，用来抑制小幅抖动。 */
#define VISION_LOST_TICKS          40    /* 视觉连续丢帧计数阈值，超过后清空视觉 PD 输出。 */
/* 方案B：目标丢失后，继续用【最后看到的位置】瞄准这么久，再进搜索。
   400 拍 x 5ms = 2 秒。目标固定时，这等于"回到最后位置"。 */
#define TARGET_HOLD_TICKS         400
/* 收到 K230 第一帧后，等这么久才允许自动搜索（毫秒）。
   给 K230 一点时间稳定画面；期间如果有目标会正常跟踪。 */
#define VISION_WAIT_MS            3000
#define VISION_YAW_LIMIT_RPM        8.0f  /* 视觉偏航 PD 输出限幅，尚未叠加 IMU 补偿。 */
#define VISION_PITCH_LIMIT_RPM      5.0f /* 视觉俯仰 PD 输出限幅。 */
#define VISION_PITCH_STEP_LIMIT_X10 5.0f /* 俯仰角度每周期累加步进限幅，写入目标角前先限制。 */
#define GIMBAL_YAW_COMP_LIMIT_RPM  35.0f /* IMU z 轴角速度折算偏航补偿的限幅。 */
#define GIMBAL_YAW_LIMIT_RPM       100.0f /* 下发到云台一轴的最终偏航速度限幅。 */
#define VISION_FPS_SAMPLE_TICKS    200   /* 帧率统计窗口，200 个 5ms 周期约为 1 秒。 */

/* 当前视觉瞄准中心点固定为视觉模块坐标原点，先去掉路线标定补偿。 */
int VISION_CENTER_X = 120;
int VISION_CENTER_Y = 57;

/* ---- 激光动态补偿系数（自动适配距离） ----
   画面偏移(像素) = LASER_K x sqrt(目标面积)
   面积 正比于 1/距离^2，所以 偏移 正比于 1/距离 正比于 sqrt(面积)
   标定：
     1. 先把 LASER_K 设 0，在某个距离调 VISION_CENTER_X/Y 让激光打中
     2. 记下偏移 P（VISION_CENTER_X 的值）和 OLED 上的面积 A
     3. LASER_K = P / sqrt(A)
     4. 换距离验证，偏了就微调
   设 0 表示只用固定基准。 */
float LASER_K_X = -0.2848f;
float LASER_K_Y = -0.0062f;

/*
 * 视觉 PD 调参说明：
 * X 轴对应云台偏航，主要负责让目标左右居中；Y 轴对应云台俯仰，主要负责让目标上下居中。
 * Kp 决定追踪力度，值大响应快但容易抖动或过冲，值小则跟踪慢、目标回中心不够积极。
 * Kd 决定阻尼，主要抑制快速变化和过冲，值大可能动作发涩或放大噪声，值小则刹不住。
 */
float KP_VISION_X = 0.02f;  /* X 轴比例系数：调左右偏航追踪力度，左右跟不住先加它，抖动明显就减小。 */
float KD_VISION_X = 0.002f; /* X 轴微分系数：给左右偏航加阻尼，转动过冲/来回摆时适当加大。 */
float KP_VISION_Y = 0.008f;/* 轴比例系数：调上下俯仰追踪力度，上下回中心慢时加大。 */
float KD_VISION_Y = 0.002f;
/* 目标速度前馈：目标在画面里移动时提前给指令，减少滞后（0 = 关闭）。
   相当于额外的微分增益，但可以单独调，不影响 KD 的抗噪性能。
   建议从 0.01 开始试，抖动就减半。 */
float KF_VISION_X = 0.0f;
float KF_VISION_Y = 0.0f; /* Y 轴微分系数：给上下俯仰加阻尼，俯仰过冲或抖动时微调。 */

/*
 * 不同路段视觉系数：
 * 这些系数会作为 vision_segment_gain 参与视觉 PD 输出，用来适配不同路线段的车速、角度和目标偏移。
 * 某段目标跟踪偏软、偏慢时增大对应系数；某段云台抖动、抢得太猛或路线变化大时减小。
 * CD_VISION_FF 是 CD 段速度前馈系数，不是 PD 增益，主要用来按车速提前补偿横向偏移。
 */
float AC_VISION_K = 0.40f;  /* AC 段视觉整体增益：只影响 AC 段，常用于单独压住或增强 AC 段跟踪。 */
float CD_VISION_K = 0.40f;  /* CD 段视觉整体增益：只影响 CD 段，配合 CD_VISION_FF 一起调。 */
float DB_VISION_K = 0.40f;  /* DB 段视觉整体增益：只影响 DB 段，方便最后一段单独修正。 */
float BA_VISION_FF = 0.05f;
float CD_VISION_FF = 0.045f; /* CD 段偏航速度前馈：车越快补偿越大，CD 段横向总偏一边时重点调它。 */

/* Visual PD state. Search mode locks after the first valid target. */
static u16 vision_lost_ticks = VISION_LOST_TICKS + 1;
static u8 vision_prev_valid = 0;
static float prev_err_x = 0.0f;
static float prev_err_y = 0.0f;
static float vision_yaw_cmd = 0.0f;
static float vision_pitch_cmd = 0.0f;
static float vision_ff_yaw_cmd = 0.0f;
static float vision_segment_gain = 1.0f;
static u16 search_scan_ticks = 0;
static u16 last_vision_frame = 0;
static u8 search_target_locked = 0;
static int32_t visual_y_angle_offset = 0;   /* 上电读到的 Y 电机实际角度，作为 0 基准 */
static u16 visual_pitch_hold = 0;
static float pitch_frac_acc = 0.0f;   /* 俯仰指令的小数累加器，避免取整吞掉小误差 */           /* 上电静默期（拍数），期间不驱动俯仰 */
static float yaw_frac_acc = 0.0f;     /* X 轴指令的小数累加器（同理，避免取整死区） */


/* 小工具函数：统一做取绝对值、限幅和浮点四舍五入，避免控制输出越界。 */
static int abs_int(int value)
{
	return value < 0 ? -value : value;
}
static int limit_int(int value, int max, int min)
{
	if(value > max) return max;
	if(value < min) return min;
	return value;
}
static float limit_float(float value, float max, float min)
{
	if(value > max) return max;
	if(value < min) return min;
	return value;
}
static float abs_float(float value)
{
	return value < 0.0f ? -value : value;
}

/* 视觉接收频率在 5ms 的 TIM2 循环中统计。 */
static void Vision_Frequency_Update(void)
{
	static u16 sample_ticks = 0;
	static u16 last_frame_count = 0;
	u16 current_frame_count;
	u16 frame_delta;

	if(g_vision_frame_flag)
	{
		/* 已消费一次视觉新帧标志，清零等待下一帧。 */
		g_vision_frame_flag = 0;
	}

	sample_ticks++;
	if(sample_ticks >= VISION_FPS_SAMPLE_TICKS)
	{
		current_frame_count = g_vision_frame_count;
		if(current_frame_count >= last_frame_count)
		{
			frame_delta = current_frame_count - last_frame_count;
		}
		else
		{
			frame_delta = (u16)(0xffff - last_frame_count + current_frame_count + 1);
		}
		Vision_Fps = frame_delta;
		Vision_Online = (frame_delta > 0) ? 1 : 0;
		last_frame_count = current_frame_count;
		sample_ticks = 0;
	}
}

/* 用 IMU 的 z 轴角速度抵消车体偏航对云台的影响。 */
static float Gimbal_GetYawCompRpm(void)
{
	float yaw_comp = h30_imu_data.gyro_z * Gimbal_Yaw_Rpm_K;
	return limit_float(-yaw_comp, GIMBAL_YAW_COMP_LIMIT_RPM, -GIMBAL_YAW_COMP_LIMIT_RPM);
}

static void Visual_Gimbal_ClearState(void)
{
	vision_lost_ticks = VISION_LOST_TICKS + 1;
	vision_prev_valid = 0;
	prev_err_x = 0.0f;
	prev_err_y = 0.0f;
	vision_yaw_cmd = 0.0f;
	vision_pitch_cmd = 0.0f;
	vision_ff_yaw_cmd = 0.0f;
	Gimbal_Joint1_Target_Rpm = 0;
	Gimbal_Joint2_Target_Rpm = 0;
}

/* Gimbal output: yaw uses speed mode, pitch uses 0~359.9 deg single-angle mode. */
/* 上电读一次 Y 电机的实际多圈角度，当"0 度基准"。
   这样不管电机自己的零点在哪，上电都不会朝绝对零位猛甩。
   同时开启 1 秒静默期，让电机先停稳再开始跟。 */
void Visual_Gimbal_CaptureYOffset(void)
{
	BLDC_Motor1.data_ready = 0;
	BLDC_ReqFeedback(BLDC_ADDR_Y_AXIS, FB_MULTI_ANGLE);
	delay_ms(60);

	if(BLDC_Motor1.data_ready)
	{
		visual_y_angle_offset = BLDC_Motor1.multi_angle;
	}

	Gimbal_Joint2_Target_Angle = 0;
	visual_pitch_hold = CONTROL_FREQUENCY;
}

/* ---------------------------- 按键（PC3） ----------------------------
   按下按键才开始搜索；如果没按按键就已经检测到目标，直接进入跟踪。
   按键模块一般是"按下为低电平"；如果你的模块相反，把 KEY_PRESSED_LEVEL 改成 1。
 -------------------------------------------------------------------- */
#define KEY_PORT            GPIOC
#define KEY_PIN             GPIO_Pin_3
#define KEY_PRESSED_LEVEL   0

static u8 search_started = 0;
static u32 vision_first_frame_ms = 0;
static u8  vision_ever_locked = 0;   /* 是否曾经锁定过有效目标 */   /* 第一帧 K230 数据到达的时刻 */

void Key_Init_PC3(void)
{
	GPIO_InitTypeDef g;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	g.GPIO_Pin   = KEY_PIN;
	g.GPIO_Mode  = (KEY_PRESSED_LEVEL == 0) ? GPIO_Mode_IPU : GPIO_Mode_IPD;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY_PORT, &g);
}

static u8 Key_IsPressed(void)
{
	u8 v = (u8)GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN);
	return (v == KEY_PRESSED_LEVEL) ? 1 : 0;
}

/* 开机发一次 Y 位置（控制环未启动时用） */
void Visual_Gimbal_BootMove(void)
{
	BLDC_SetMultiAngle(BLDC_ADDR_Y_AXIS, visual_y_angle_offset + Gimbal_Joint2_Target_Angle);
}

static void Visual_Gimbal_Output(float yaw_rpm, float pitch_rpm)
{
	int32_t pitch_next;

	yaw_rpm = limit_float(yaw_rpm, GIMBAL_YAW_LIMIT_RPM, -GIMBAL_YAW_LIMIT_RPM);
	pitch_rpm = limit_float(pitch_rpm, VISION_PITCH_STEP_LIMIT_X10, -VISION_PITCH_STEP_LIMIT_X10);
	/* X 轴也要小数累加：否则 round(KP*err) 在误差小时恒为 0，
	   云台会在离瞄准点 40~50 像素处彻底停住（"靠近目标就慢下来"）。 */
	yaw_frac_acc += yaw_rpm;
	Gimbal_Joint1_Target_Rpm = (int16_t)yaw_frac_acc;
	yaw_frac_acc -= (float)Gimbal_Joint1_Target_Rpm;
	/* 小数累加：不够 1 单位的指令先攒着，攒够再走一步，
	   这样误差 1~2 像素时也能平滑收敛过去，不会被取整吞掉。 */
	pitch_frac_acc += pitch_rpm;
	Gimbal_Joint2_Target_Rpm = (int16_t)pitch_frac_acc;
	pitch_frac_acc -= (float)Gimbal_Joint2_Target_Rpm;
	/* 先按步进累加，再夹到软限位区间内，超出就不让走 */
	pitch_next = (int32_t)Gimbal_Joint2_Target_Angle + Gimbal_Joint2_Target_Rpm;
	if(pitch_next < GIMBAL_Y_ANGLE_MIN_X10) pitch_next = GIMBAL_Y_ANGLE_MIN_X10;
	if(pitch_next > GIMBAL_Y_ANGLE_MAX_X10) pitch_next = GIMBAL_Y_ANGLE_MAX_X10;
	Gimbal_Joint2_Target_Angle = pitch_next;
	BLDC_SetSpeed(BLDC_ADDR_X_AXIS, Gimbal_Joint1_Target_Rpm);
	delay_us(250);
	BLDC_SetMultiAngle(BLDC_ADDR_Y_AXIS, visual_y_angle_offset + Gimbal_Joint2_Target_Angle);
}

/* 搜索：按一下 PC3 按键后 X 轴才慢转；检测到目标立即由视觉 PD 接管。 */
static void Visual_Gimbal_SearchScan(float yaw_base_rpm)
{
	if(search_target_locked)
	{
		Visual_Gimbal_Output(0.0f, 0.0f);
		return;
	}

	if(!search_started && Key_IsPressed())
	{
		search_started = 1;
	}

	if(!search_started)
	{
		Visual_Gimbal_Output(yaw_base_rpm, 0.0f);
		return;
	}

	/* 还没收到过 K230 任何数据，或者收到第一帧后还没等够时间 -> 先别转 */
	if(vision_first_frame_ms == 0)
	{
		Visual_Gimbal_Output(yaw_base_rpm, 0.0f);
		return;
	}

	if((User_GetSysTickTime() - vision_first_frame_ms) < VISION_WAIT_MS)
	{
		Visual_Gimbal_Output(yaw_base_rpm, 0.0f);
		return;
	}

	search_scan_ticks++;

	/* pitch 传 0 -> Y 轴目标角不变，搜索期间 Y 锁死不动 */
	Visual_Gimbal_Output(yaw_base_rpm + 8.0f, 0.0f);
}


/* work_mode=SEARCH：寻靶；work_mode=RUN：巡线打靶并叠加补偿。 */
static void Visual_Gimbal_Process(u8 work_mode)
{
	u8 use_compensation = (work_mode == CONTROL_MODE_RUN) ? 1 : 0;
	float yaw_base_rpm = use_compensation ? Gimbal_GetYawCompRpm() : 0.0f;
	float err_x;
	float err_y;
	float diff_err_x;
	float diff_err_y;
	float cmd_yaw;
	float cmd_pitch;
	float aim_x;
	float aim_y;

	if(visual_pitch_hold > 0) visual_pitch_hold--;

	/* 按键每 5ms 采样一次（不管当前是跟踪还是等待），
	   这样在"守株待兔"期间按下的键也能记住。 */
	if(!search_started && Key_IsPressed())
	{
		search_started = 1;
	}

	/* 只有收到带【有效目标】的帧才清除丢失计时，
	   这样 K230 偶尔掉一帧（area=0）不会立刻重新搜索。 */
	if(g_vision_frame_count != last_vision_frame)
	{
		last_vision_frame = g_vision_frame_count;

		/* 记下第一帧到达的时刻，用于"等几秒再搜索" */
		if(vision_first_frame_ms == 0)
		{
			vision_first_frame_ms = User_GetSysTickTime();
		}

		if(g_follow_area != 0)
		{
			vision_lost_ticks = 0;
			vision_ever_locked = 1;      /* 记下"已经见过目标" */

			/* 每次锁定目标都清掉按键状态，
			   这样下次脱靶必须重新按一下才会搜索 */
			search_started = 0;
		}
	}

	if(vision_lost_ticks < 0xffff) vision_lost_ticks++;

	/* ★ 还没锁定过任何目标 -> 绝对不跑 PD。
	   否则 g_follow_x/y 还是 0，会被当成"目标在画面正中"，
	   误差 = 瞄准点-0，X 会莫名其妙转起来，Y 也会被推着动。 */
	if(!vision_ever_locked)
	{
		Visual_Gimbal_SearchScan(yaw_base_rpm);
		return;
	}

	/* 见过目标，但连续丢失太久 -> 重置，回去搜索 */
	if(vision_lost_ticks >= (VISION_LOST_TICKS + TARGET_HOLD_TICKS))
	{
		vision_ever_locked = 0;
		Visual_Gimbal_ClearState();
		Visual_Gimbal_SearchScan(yaw_base_rpm);
		return;
	}
	if(use_compensation == 0)
	{
		search_target_locked = 1;
	}
	else
	{
		search_scan_ticks = 0;
		search_target_locked = 0;
	}

	/* 激光动态补偿：偏移 = 静态基准 + K x sqrt(面积) */
	{
		float dyn = (float)isqrt_u32((u32)g_follow_area);

		if(dyn > 500.0f) dyn = 500.0f;

		aim_x = (float)VISION_CENTER_X + LASER_K_X * dyn;
		aim_y = (float)VISION_CENTER_Y + LASER_K_Y * dyn;
	}

	err_x = aim_x - (float)g_follow_x;
	err_y = aim_y - (float)g_follow_y;
	if(vision_prev_valid == 0)
	{
		prev_err_x = err_x;
		prev_err_y = err_y;
		vision_prev_valid = 1;
	}

	diff_err_x = err_x - prev_err_x;
	diff_err_y = err_y - prev_err_y;
	prev_err_x = err_x;
	prev_err_y = err_y;

	cmd_yaw = VISION_DIR_X * vision_segment_gain * (KP_VISION_X * err_x + KD_VISION_X * diff_err_x)
	        + VISION_DIR_X * KF_VISION_X * diff_err_x;   /* 目标速度前馈 */
	if(abs_float(err_x) <= VISION_DEADZONE) cmd_yaw = 0.0f;
	vision_yaw_cmd = limit_float(cmd_yaw, VISION_YAW_LIMIT_RPM, -VISION_YAW_LIMIT_RPM);

	cmd_pitch = VISION_DIR_Y * vision_segment_gain * (KP_VISION_Y * err_y + KD_VISION_Y * diff_err_y)
	          + VISION_DIR_Y * KF_VISION_Y * diff_err_y;   /* 目标速度前馈 */
	if(abs_float(err_y) <= VISION_DEADZONE) cmd_pitch = 0.0f;
	vision_pitch_cmd = limit_float(cmd_pitch, VISION_PITCH_LIMIT_RPM, -VISION_PITCH_LIMIT_RPM);

	/* 上电静默期：让电机先停稳，不做俯仰动作 */
	if(visual_pitch_hold > 0) vision_pitch_cmd = 0.0f;

	Visual_Gimbal_Output(yaw_base_rpm + vision_yaw_cmd +
	                     (use_compensation ? vision_ff_yaw_cmd : 0.0f),
	                     vision_pitch_cmd);
}
/* 根据左右电机目标 PWM 设置方向引脚和占空比，停车时同时关闭 H 桥输入。 */
void Motor_SetPwm(int motor_left, int motor_right)
{
	Motor_Left = limit_int(motor_left, PWM_MAX, -PWM_MAX);
	Motor_Right = limit_int(motor_right, PWM_MAX, -PWM_MAX);
	if(Control_Work_Mode == CONTROL_MODE_SEARCH || !Control_Work_Enable)
	{
		Motor_Left = 0;
		Motor_Right = 0;
		AIN1 = 0;
		AIN2 = 0;
		BIN1 = 0;
		BIN2 = 0;
		PWMA = 0;
		PWMB = 0;
		return;
	}
	if(Motor_Right > 0)      AIN1 = 1, AIN2 = 0;
	else if(Motor_Right < 0) AIN1 = 0, AIN2 = 1;
	else                    AIN1 = 0, AIN2 = 0;
	PWMA = abs_int(Motor_Right);
	if(Motor_Left > 0)      BIN1 = 1, BIN2 = 0;
	else if(Motor_Left < 0) BIN1 = 0, BIN2 = 1;
	else                   BIN1 = 0, BIN2 = 0;
	PWMB = abs_int(Motor_Left);
}




static void Control_StopWork(void)
{
	/* 停止当前动作，但保留当前模式选择；下一次单击仍启动这个模式。 */
	Control_Work_Enable = 0;
	search_scan_ticks = 0;
	search_target_locked = 0;
	Visual_Gimbal_ClearState();
	Visual_Gimbal_Output(0.0f, 0.0f);
}

static void Control_StartWork(void)
{
	/* 启动当前模式：SEARCH 寻靶，RUN 巡线打靶。 */
	Control_Work_Enable = 1;
	search_scan_ticks = 0;
	search_target_locked = 0;
	Visual_Gimbal_ClearState();
}

static void Control_ToggleWork(void)
{
	/* 单击在“当前模式启动”和“当前模式待启动”之间来回切换。 */
	if(Control_Work_Enable)
	{
		Control_StopWork();
	}
	else
	{
		Control_StartWork();
	}
}

static void Control_SwitchMode(void)
{
	/* 长按只切换模式并停止当前动作；切换后必须再单击启动。 */
	Control_StopWork();
	Control_Work_Mode = (Control_Work_Mode == CONTROL_MODE_SEARCH) ? CONTROL_MODE_RUN : CONTROL_MODE_SEARCH;
}

/* 按键处理：单击启停当前模式，双击不使用，长按切换寻靶/巡线打靶模式。 */
void Key_Process(void)
{
	u8 key = click_N_Double(50);
	if(key == 1)
	{
		Control_ToggleWork();
		return;
	}
	if(key == 2)
	{
		return;
	}
	if(Long_Press())
	{
		Control_SwitchMode();
	}
}
/* TIM2 main 5 ms loop: line inspection and PI always run; stop/run only selects output logic. */
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		/* 云台版：只跑视觉帧率统计 + 视觉 PD -> 云台
		   红外、编码器、车轮 PI、IMU 全部不需要 */
		Vision_Frequency_Update();
		Visual_Gimbal_Process(Control_Work_Mode);
	}
}

