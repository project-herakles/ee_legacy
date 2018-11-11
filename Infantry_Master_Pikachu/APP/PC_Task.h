#ifndef __PC_TASK_
#define __PC_TASK_
#include <stdint.h>

typedef enum
{
	PC_SOF,
	PC_LENGTH,
	PC_CMD,
	PC_DATA,
}PC_Rx_State_e;

typedef struct
{
	uint8_t Length;
	uint8_t Cmd;
	uint8_t Data[8];
}PC_Rx_Struct_t;

typedef struct
{
	float yaw_angle;
	float pitch_angle;
}Gimbal_CMD_t;

typedef struct
{
	uint8_t fire;
	float freq;
}Shoot_CMD_t;

typedef struct
{
	Gimbal_CMD_t Gimbal;
	Shoot_CMD_t shoot;
}PC_Command_t;

typedef enum{
	SUCCESSFUL,
	UNSUCCESSFUL,
}Receive_Info_e;

void COM_Receive_PC(PC_Rx_Struct_t* rx);
void pc_handler(uint8_t byte);
void COM_Upload_PC(void);



#endif
