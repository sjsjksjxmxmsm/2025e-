#ifndef __IR_TRACK_H
#define __IR_TRACK_H

#include "sys.h"

/* Raw GPIO levels, from left to right: PA1, PC15, PC14, PC13. */
extern u8 ir_dh1_state;
extern u8 ir_dh2_state;
extern u8 ir_dh3_state;
extern u8 ir_dh4_state;
extern int turn_cnt;
extern int TurnFinishCnt;
typedef enum {
	ROAD_SEG_BA = 0,
	ROAD_SEG_AC,
	ROAD_SEG_CD,
	ROAD_SEG_DB
} RoadSegment_t;

extern float Turn90Angle;
extern float TurnMaxAngle;
extern float TurnMidAngle;
extern float TurnMinAngle;
extern float BaseSpeed;
extern float ForwardLimit;
extern float GyroStraightK;
extern float GyroStraightLimit;
extern float base_speed_mm;
extern float turn_diff;
extern volatile RoadSegment_t CurrentRoadSegment;

void IR_Track_Init(void);
void IR_Track_Update(void);
u8 IR_Track_Read(void);
void IR_Track_LineInspection(void);
void IRDM_line_inspection(void);
void IR_Track_ResetRoadSegment(void);
const u8 *IR_Track_GetRoadSegmentName(void);

#endif
