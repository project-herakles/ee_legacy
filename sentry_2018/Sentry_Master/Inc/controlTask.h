#ifndef _CONTROLTASK_H_
#define _CONTROLTASK_H_
#include "stm32f4xx.h"
typedef enum
{
	PREPARE_STATE,
	NORMAL_STATE,
	STOP_STATE,
}workState_e;

typedef enum
{
	FRIC_STATIC,
	FRIC_ACCELERATING,
	FRIC_MAX_SPEED,
}frictionState_e;

#define PREPARE_TIME_TICK_MS 4000
#define CHASSIS_SPEED_ATTENUATION (1.0f)
#define SPEED_OUTPUT_ATTENUATION (80.0f)


void CM_Control(void);
void workStateFSM(void);
void setWorkState(workState_e workState);
workState_e getWorkState(void);
void contorlTaskInit(void);
void Control_Loop(void);
void Gimbal_Control(void);
uint32_t getCurrentTimeTick(void);
#endif
