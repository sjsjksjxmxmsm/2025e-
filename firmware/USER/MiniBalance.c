/* ============================================================================
 * 云台版主程序（只做：K230 视觉 + 二轴云台 + OLED 状态显示）
 * ----------------------------------------------------------------------------
 * 接线（江协 32 核心板）：
 *   云台  USART1 : PA9 (TX) -> 云台RX ，PA10(RX) <- 云台TX  （板上 H4 的 TXD1/RXD1）
 *   K230  USART3 : PB11(RX) <- K230 TX   （板上 GX_SDA，已验证）
 *
 *   ！！板载 CH340 的 TXD/RXD 并接在 PA9/PA10 上，
 *      接云台时【绝对不能插 USB 线】，供电请走 DC 输入或 5V 排针。
 *   OLED  SPI   : PB3=SCLK  PB4=DC  PB5=SDIN  PB8=RES
 * ==========================================================================*/

#include "stm32f10x.h"
#include "sys.h"

#define USE_OLED     1      /* 没有屏就改成 0 */

#if USE_OLED
static void OLED_ShowSignedInt(u8 x, u8 y, int value, u8 len)
{
	int v = value;

	if(v < 0) { OLED_ShowString(x, y, "-"); v = -v; }
	else      { OLED_ShowString(x, y, "+"); }

	OLED_ShowNumber(x + 8, y, (u32)v, len, 12);
}

static void Show_Status(void)
{
	OLED_ShowString(0, 0, "K230 GIMBAL");
	OLED_ShowString(0, 12, "N");
	OLED_ShowNumber(8, 12, g_vision_frame_count, 5, 12);

	/* 陀螺 Z 角速度（0.1°/s 为单位） */
	OLED_ShowString(48, 12, "G");
	OLED_ShowSignedInt(56, 12, (int)(g_mpu_gyro_z * 10.0f), 4);

	OLED_ShowString(0, 24, "X");
	OLED_ShowSignedInt(8, 24, g_follow_x, 4);
	OLED_ShowString(64, 24, "Y");
	OLED_ShowSignedInt(72, 24, g_follow_y, 3);

	OLED_ShowString(0, 36, "A");
	OLED_ShowNumber(8, 36, g_follow_area, 5, 12);

	/* MPU6050 诊断：ID 应为 68，OK 应为 1 */
	OLED_ShowString(0, 48, "ID");
	OLED_ShowNumber(16, 48, g_mpu_id, 3, 12);
	OLED_ShowString(40, 48, "OK");
	OLED_ShowNumber(56, 48, g_mpu_ok, 1, 12);

	OLED_Refresh_Gram();
}
#endif

static void Timer2_Init_5ms(void)
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

static void Gimbal_Init(void)
{
	BLDC_Disable(BLDC_ADDR_X_AXIS);
	delay_us(500);
	BLDC_Disable(BLDC_ADDR_Y_AXIS);
	delay_us(500);

	BLDC_SetAcc(BLDC_ADDR_X_AXIS,100);
	delay_us(500);
	BLDC_SetAcc(BLDC_ADDR_Y_AXIS,100);
	delay_us(500);

	BLDC_SetMode(BLDC_ADDR_X_AXIS,MODE_SPEED);
	delay_us(500);
	BLDC_SetMode(BLDC_ADDR_Y_AXIS,MODE_SPEED);
	delay_us(500);
	BLDC_SetSpeed(BLDC_ADDR_X_AXIS,0);
	delay_us(500);
	BLDC_SetSpeed(BLDC_ADDR_Y_AXIS,0);
	delay_us(500);

	BLDC_Enable(BLDC_ADDR_X_AXIS);
	delay_us(500);
	BLDC_Enable(BLDC_ADDR_Y_AXIS);
	delay_us(500);

	/* Y 轴用多圈位置模式（0x0001，手册明确写的那个）。
	   先把当前多圈角度清零当零点，这样"转到 0"就是原地不动，不会乱摆。 */
	BLDC_SetMode(BLDC_ADDR_Y_AXIS,MODE_MULTI_POS);
	delay_us(500);
	BLDC_SetSpeed(BLDC_ADDR_Y_AXIS, 25);   /* 位置模式下的轨迹速度，必须非零(官方例程 50) */
	delay_us(500);
	/* ★ 上电保护：读电机实际角度当基准 + 1 秒静默期
	   不再依赖 ClearMultiAngle，就算它不生效也不会甩。 */
	Visual_Gimbal_CaptureYOffset();
}

int main(void)
{
	u32 last_show;

	MY_NVIC_PriorityGroupConfig(2);
	JTAG_Set(JTAG_SWD_DISABLE);
	JTAG_Set(SWD_ENABLE);

	SysTick_Init(1000);
	delay_ms(1500);  /* 官方例程：等 1.5s 让电机启动 */

#if USE_OLED
	OLED_Init();
	OLED_Clear();
#endif

	DataScope_DP_Init();          /* 云台 USART3 */
	delay_us(1000);
	Gimbal_Init();
	Key_Init_PC3();                 /* PC3 按键：按下才开始搜索 */
	MPU6050_Init();                 /* 陀螺仪初始化 + 零偏校准 */

	/* ================= 开机动作（控制环启动前做完） =================
	   1. Y 轴向下转到 -90 度
	   2. 等它转到位
	   3. X 轴以 24rpm 转 2.5 秒（正好 360 度）扫描一圈
	   ==============================================================*/
	Gimbal_Joint2_Target_Angle = -1150;      /* Y 向下 90 度 */
	Visual_Gimbal_BootMove();               /* 发出位置指令，Y 开始转 */
	delay_ms(1500);                          /* 等 Y 转到位（约 0.5s + 余量） */

	/* ================= 进入正常跟踪 ================= */
	Bluetooth_USART2_Init();      /* K230 USART1 */

	Control_Work_Mode   = CONTROL_MODE_RUN;   /* RUN 模式才会启用陀螺前馈 */
	Control_Work_Enable = 1;

	Timer2_Init_5ms();

	last_show = User_GetSysTickTime();

	while(1)
	{
		MPU6050_Update();      /* 读陀螺仪，内部限速 200Hz */

#if USE_OLED
		if((User_GetSysTickTime() - last_show) >= 200)
		{
			last_show = User_GetSysTickTime();
			Show_Status();
		}
#else
		delay_ms(100);
#endif
	}
}
