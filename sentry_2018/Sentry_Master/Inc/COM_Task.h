#ifndef _COM_TASK_H_
#define _COM_TASK_H_
#include <stdint.h>

#define UPLOAD_FRAMELENGTH 9
typedef enum 
{
	SOF,
	LENGTH_LOW,
	LENGTH_HIGH,
	SEQ,
	CRC8,
	CMD_LOW,
	CMD_HIGH,
	DATA,
	CRC16_LOW,
	CRC16_HIGH,
}Judge_RxState_e;

typedef struct
{
	uint16_t Length;
	uint8_t Seq;
	uint8_t CRC8;
	uint16_t Cmd;
	uint8_t Data[20];
	uint16_t CRC16;
}Judge_RxStruct;

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
	uint8_t update;
}Gimbal_CMD_t;

typedef struct
{
	uint8_t dir;
}Chassis_CMD_t;

typedef struct
{
	uint8_t fire;
	float freq;
}Shoot_CMD_t;

typedef struct
{
	Gimbal_CMD_t Gimbal;
	Chassis_CMD_t Chassis;
	Shoot_CMD_t shoot;
}PC_Command_t;

typedef struct
{
	uint16_t remainHP;
	uint8_t armorType;
	uint8_t hurtType;
}Judge_Data_t;

typedef struct
{
	int8_t end;
	Judge_Data_t judge;
	int16_t real_yaw;
	int16_t real_pitch;
}PC_Upload_data_t;

typedef struct
{
	uint16_t stageRemainTime;
	uint8_t gameProgress;
	uint8_t robotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
}extGameRobotState_t; //ID 0x001

typedef struct
{
	uint8_t armorType:4;
	uint8_t hurtType:4;
}extRobotHurt_t;

typedef struct
{
	uint8_t bulletType;
	uint8_t bulletFreq;
	float bulletSpeed;
}extShootData_t;

typedef struct
{
	float chassisVolt;
	float chassisCurrent;
	float chassisPower;
	float chassisPowerBuffer;
	uint16_t shooterHeat0;
	uint16_t shooterHeat1;
}extPowerHeatData_t;

typedef struct
{
	uint8_t cardType;
	uint8_t cardIdx;
}extRfidDetect_t;

typedef struct
{
	uint8_t winner;
}extGameResult_t;

typedef struct
{
	uint16_t buffMusk;
}extBuffMusk_t;

typedef struct 
{
	float x;
	float y;
	float z;
	float yaw;
}extGameRobotPos_t;

typedef struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}extShowData_t;

uint8_t CRC8_check(Judge_RxStruct* rxStruct);
uint8_t CRC16_check(Judge_RxStruct* rx);
void COM_Judge_Process(Judge_RxStruct* rx);
void JudgeRx_Reset(Judge_RxState_e* state,Judge_RxStruct* rxStruct);
void COM_Receive_PC(PC_Rx_Struct_t* rx);
void COM_Receive_PC_handler(uint8_t byte);
void COM_Upload_PC(void);


#endif
