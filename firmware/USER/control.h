#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#define PWM_MAX              6900
#define CONTROL_FREQUENCY    200.0f
#define ENCODER_MULTIPLES    4.0f
#define WHEEL_PERIMETER      0.204203519f
#define MOTOR_GEAR_RATIO     28.0f
#define ENCODER_RESOLUTION   13.0f
#define WHEEL_SPACING        0.1610f
#define SPEED_STEP           0.05f
#define SPEED_MAX            0.60f
#define CONTROL_MODE_RUN     0
#define CONTROL_MODE_SEARCH  1

/*
 * Y 轴（俯仰）软限位，单位 0.1 度，可以写负数：
 *     0     = 电机自己的零点
 *   -120    = -12.0 度（对应单圈角度里的 348.0 度，也就是寻靶位置）
 * 做法：目标角只在这个区间内累加，超出就被夹住，不会再一直转圈。
 * 拿到机械极限的方法见调试说明，把两个数改成你的实际值即可。
 */
#define GIMBAL_Y_ANGLE_MIN_X10      (-36000)      /* -60.0 度，最低能到哪 */
#define GIMBAL_Y_ANGLE_MAX_X10      ( 36000)      /* +60.0 度，最高能到哪 */
#define GIMBAL_Y_SEARCH_ANGLE_X10   (0)      /* 寻靶时抬到 348.0 度 = -12.0 度 */

#if (GIMBAL_Y_ANGLE_MAX_X10 <= GIMBAL_Y_ANGLE_MIN_X10)
#error "GIMBAL_Y_ANGLE_MAX_X10 must be greater than GIMBAL_Y_ANGLE_MIN_X10"
#endif
#define LEFT_ENCODER_SIGN    1.0f
#define RIGHT_ENCODER_SIGN  1.0f
#define LEFT_PWM_SIGN       -1
#define RIGHT_PWM_SIGN       -1
typedef struct
{
	float Current_Encoder;
	float Motor_Pwm;
	float Target_Encoder;
} Motor_Parameter;
typedef struct
{
	int A;
	int B;
} Encoder_Original;
extern volatile Motor_Parameter MotorA;
extern volatile Motor_Parameter MotorB;
extern u8 Control_Work_Mode;
extern u8 Control_Work_Enable;
extern float Velocity_KP ;
extern float Velocity_KI ;
extern float Gimbal_Yaw_Rpm_K;
extern float KP_VISION_X;
extern float KD_VISION_X;
extern float KP_VISION_Y;
extern float KD_VISION_Y;
extern float AC_VISION_K;
extern float CD_VISION_K;
extern float DB_VISION_K;
extern float BA_VISION_FF;
extern float CD_VISION_FF;

extern int VISION_CENTER_X;
extern int VISION_CENTER_Y;
extern volatile u16 Vision_Fps;
extern volatile u8 Vision_Online;
extern volatile int16_t Gimbal_Joint1_Target_Rpm;
extern volatile int16_t Gimbal_Joint2_Target_Rpm;
extern volatile int32_t Gimbal_Joint2_Target_Angle;   /* 有符号、不回绕，0.1 度 */
void Visual_Gimbal_BootMove(void);
void Key_Init_PC3(void);
void MPU6050_Init(void);
void MPU6050_Update(void);
extern float g_mpu_gyro_z;
extern u8 g_mpu_id;
extern u8 g_mpu_ok;
void Visual_Gimbal_CaptureYOffset(void);   /* 上电读实际角度当基准 */
void Motor_SetPwm(int motor_left, int motor_right);
void Motor_ClosedLoop_5ms(void);
void Control_SetTargetSpeed(float speed);
void Control_AddTargetSpeed(float delta);
#endif
