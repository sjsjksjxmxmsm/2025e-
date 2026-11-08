#ifndef __H30_IMU_H
#define __H30_IMU_H

#include "sys.h"

typedef struct
{
	float acc_x;
	float acc_y;
	float acc_z;

	float gyro_x;
	float gyro_y;
	float gyro_z;

	float pitch;
	float roll;
	float yaw;

	float q0;
	float q1;
	float q2;
	float q3;
} H30_IMU_Data;

extern volatile u8 h30_imu_data_ready;
extern volatile u8 h30_imu_last_error;
extern volatile u16 h30_imu_sample_tick;
extern volatile u16 h30_imu_sample_seconds;
extern H30_IMU_Data h30_imu_data;

void H30_IMU_Init(void);
void H30_IMU_TIM_Callback(void);
void H30_IMU_Process(void);
u8 H30_IMU_ReadAll(void);
u8 H30_IMU_ReadYawGyro(void);

#endif
