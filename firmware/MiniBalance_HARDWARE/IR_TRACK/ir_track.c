#include "ir_track.h"
#include "control.h"
#include "h30_imu.h"

/* 四路红外巡线传感器原始电平，从左到右依次为 dh1~dh4。 */
u8 ir_dh1_state = 1;
u8 ir_dh2_state = 1;
u8 ir_dh3_state = 1;
u8 ir_dh4_state = 1;

/* 巡线差速参数：不同红外状态对应不同转向差速量，单位按 mm/s 参与计算。 */
float Turn90Angle  = 60.0f;
float TurnMaxAngle = 40.0f;
float TurnMidAngle = 15.0f;
float TurnMinAngle = 5.0f;
float BaseSpeed    = 460.0f;
float ForwardLimit = 50.0f;
float GyroStraightK = 0.95f;
float GyroStraightLimit = 15.0f;

/* 当前计算出的前进速度和转向差速，最终会换算成左右轮目标速度。 */
float base_speed_mm = 0.0f;
float turn_diff = 0.0f;
int Smooth = 0;

/* 当前所在路线段，用于和视觉瞄准中心、参数显示等模块同步。 */
volatile RoadSegment_t CurrentRoadSegment = ROAD_SEG_BA;
int turn_cnt = 0;
int TurnFinishCnt = 562;
static const u8 RoadSegmentName[4][3] = {
	"BA",
	"AC",
	"CD",
	"DB"
};

#define SMOOTH_PERIOD 100
#define SMOOTH_MIN_RATE 0.4f

/* 四路传感器按 bit3~bit0 组成状态码，0 表示压到黑线，1 表示白底。 */
typedef enum {
	STATE_CROSS       = 0,   /* 0000, all black */
	STATE_LEFT_90_A   = 1,   /* 0001 */
	STATE_LEFT_90_B   = 3,   /* 0011 */
	STATE_RIGHT_90_A  = 8,   /* 1000 */
	STATE_RIGHT_90_B  = 12,  /* 1100 */
	STATE_LEFT_BIG    = 7,   /* 0111 */
	STATE_RIGHT_BIG   = 14,  /* 1110 */
	STATE_LEFT_SMALL  = 11,  /* 1011 */
	STATE_RIGHT_SMALL = 13,  /* 1101 */
	STATE_STRAIGHT    = 9,   /* 1001 */
	STATE_LOST        = 15   /* 1111, all white */
} SensorState_t;

/* 本文件内部使用的小工具：取绝对值和限幅。 */
static float abs_float(float value)
{
	return value < 0.0f ? -value : value;
}

static float limit_float(float value, float max, float min)
{
	if(value > max) return max;
	if(value < min) return min;
	return value;
}

/* 出弯后用 Smooth 做一段平滑加速，避免刚转完弯速度突变。 */
static float smooth_speed_rate(void)
{
	float rate;

	if(Smooth <= 0) return 1.0f;
	if(Smooth > SMOOTH_PERIOD) Smooth = SMOOTH_PERIOD;

	rate = SMOOTH_MIN_RATE +
	       (1.0f - SMOOTH_MIN_RATE) *
	       (float)(SMOOTH_PERIOD - Smooth) / (float)SMOOTH_PERIOD;

	Smooth--;
	return rate;
}

/* 每完成一个 90 度转弯，路线段按 BA -> AC -> CD -> DB -> BA 循环切换。 */
static void road_segment_next(void)
{
	switch(CurrentRoadSegment)
	{
		case ROAD_SEG_BA:
			CurrentRoadSegment = ROAD_SEG_AC;
			break;

		case ROAD_SEG_AC:
			CurrentRoadSegment = ROAD_SEG_CD;
			break;

		case ROAD_SEG_CD:
			CurrentRoadSegment = ROAD_SEG_DB;
			break;

		default:
			CurrentRoadSegment = ROAD_SEG_BA;
			break;
	}
}

/* 重新从 BA 段开始计路段，一般在初始化或重新开始跑图时调用。 */
void IR_Track_ResetRoadSegment(void)
{
	CurrentRoadSegment = ROAD_SEG_BA;
}

/* 给 OLED/调参界面返回当前路线段名字。 */
const u8 *IR_Track_GetRoadSegmentName(void)
{
	RoadSegment_t segment = CurrentRoadSegment;

	if(segment > ROAD_SEG_DB) segment = ROAD_SEG_BA;
	return RoadSegmentName[segment];
}

/* 初始化红外巡线 GPIO：PA1、PC13、PC14、PC15 均使用上拉输入。 */
void IR_Track_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	IR_Track_ResetRoadSegment();
	IR_Track_Update();
}

/* 读取四个 GPIO 电平并缓存到全局状态变量。 */
void IR_Track_Update(void)
{
	ir_dh1_state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
	ir_dh2_state = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);
	ir_dh3_state = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);
	ir_dh4_state = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
}

/* 返回四路红外组合状态：dh1 为最高位，dh4 为最低位。 */
u8 IR_Track_Read(void)
{
	IR_Track_Update();

	return (ir_dh1_state << 3) |
	       (ir_dh2_state << 2) |
	       (ir_dh3_state << 1) |
	        ir_dh4_state;
}

/* 巡线主逻辑：根据红外状态计算差速，并写入左右轮目标速度。 */
void IR_Track_LineInspection(void)
{
	static int last_state = STATE_CROSS;
	float left_motor_speed;
	float right_motor_speed;
	float abs_turn;
	float smooth_rate;
	int sensor_state;

	sensor_state = IR_Track_Read();

	/* 只有巡线打靶模式且已经启动时，直角弯识别才会推进巡线流程。 */
	if((sensor_state == STATE_LEFT_90_A ||
	    sensor_state == STATE_RIGHT_90_A ||
	    sensor_state == STATE_LEFT_90_B ||
	    sensor_state == STATE_RIGHT_90_B) && turn_cnt == 0 && (Control_Work_Mode == CONTROL_MODE_RUN) && Control_Work_Enable)
	{
		turn_cnt = 1;
	}
	if(turn_cnt > 0)
	{
		turn_cnt++;
		if(turn_cnt < 100)
		{
			/* 转弯前先向前压一小段，让车身进入弯道位置。 */
			MotorA.Target_Encoder = 0.28f;
			MotorB.Target_Encoder = 0.28f;
			return ;
		}
		else if(turn_cnt < TurnFinishCnt &&
				sensor_state !=STATE_STRAIGHT
		        // sensor_state != STATE_LEFT_SMALL &&
		        // sensor_state != STATE_RIGHT_SMALL
			)
		{
			/* 未重新识别到直线前，原地/小半径持续转向。 */
			MotorA.Target_Encoder = -0.043f;
			MotorB.Target_Encoder = 0.043f;
			return ;
		}
		else
		{
			/* 转弯完成后清零计数，启动平滑加速，并切换到下一路段。 */
			turn_cnt = 0;
			Smooth = SMOOTH_PERIOD;
			road_segment_next();
		}
	}

	/* 普通巡线状态表：左偏给正差速，右偏给负差速。 */
	switch(sensor_state)
	{
		case STATE_CROSS:
			turn_diff = 0.0f;
			break;

		case STATE_LEFT_90_A:
		case STATE_LEFT_90_B:
			turn_diff = Turn90Angle;
			break;

		case STATE_RIGHT_90_A:
		case STATE_RIGHT_90_B:
			turn_diff = -Turn90Angle;
			break;

		case STATE_LEFT_BIG:
			turn_diff = TurnMaxAngle;
			break;

		case STATE_RIGHT_BIG:
			turn_diff = -TurnMaxAngle;
			break;

		case STATE_LEFT_SMALL:
			turn_diff = TurnMinAngle;
			break;

		case STATE_RIGHT_SMALL:
			turn_diff = -TurnMinAngle;
			break;

		case STATE_STRAIGHT:
			turn_diff = 0.0f;
			break;

		case STATE_LOST:
			/* 全白丢线时沿用上一次偏离方向，避免立刻失去修正方向。 */
			if(last_state == STATE_LEFT_SMALL)       turn_diff = TurnMidAngle;
			else if(last_state == STATE_RIGHT_SMALL) turn_diff = -TurnMidAngle;
			else if(last_state == STATE_LEFT_BIG)    turn_diff = TurnMaxAngle;
			else if(last_state == STATE_RIGHT_BIG)   turn_diff = -TurnMaxAngle;
			break;

		default:
			turn_diff = 0.0f;
			break;
	}

	if(sensor_state != STATE_LOST)
	{
		/* 只用有效状态更新 last_state，丢线时才能参考上一次方向。 */
		last_state = sensor_state;
	}

	if(turn_cnt == 0 &&
	   (sensor_state == STATE_STRAIGHT ||
	    sensor_state == STATE_LEFT_SMALL ||
	    sensor_state == STATE_RIGHT_SMALL))
	{
		/* 直线或小偏差状态下叠加陀螺仪补偿，抑制车体慢慢跑偏。 */
		float gyro_comp = limit_float(-h30_imu_data.gyro_z * GyroStraightK,
		                              GyroStraightLimit, -GyroStraightLimit);
		turn_diff += gyro_comp;
	}

	/* 转向越大，基础前进速度越低；超过 ForwardLimit 时只转向不前进。 */
	abs_turn = abs_float(turn_diff);
	if(abs_turn < ForwardLimit)
	{
		base_speed_mm = BaseSpeed - (BaseSpeed * (abs_turn / ForwardLimit));
	}
	else
	{
		base_speed_mm = 0.0f;
	}

	smooth_rate = smooth_speed_rate();
	base_speed_mm *= smooth_rate;

	/* 差速模型：左轮 = 前进 - 转向，右轮 = 前进 + 转向，单位换成 m/s。 */
	left_motor_speed = 0.001f * (base_speed_mm - turn_diff);
	right_motor_speed = 0.001f * (base_speed_mm + turn_diff);

	MotorA.Target_Encoder = left_motor_speed;
	MotorB.Target_Encoder = right_motor_speed;
}

/* 兼容旧函数名，实际仍调用新的巡线主逻辑。 */
void IRDM_line_inspection(void)
{
	IR_Track_LineInspection();
}
