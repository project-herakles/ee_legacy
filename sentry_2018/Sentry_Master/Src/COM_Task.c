#include "COM_Task.h"
#include "convert.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include "sensor.h"
#include "can.h"
#include "controlTask.h"

//Process the message received, rx is the valid data in order
extGameRobotState_t robotState;
extRobotHurt_t robotHurt;
extShootData_t shootData;
extPowerHeatData_t PHData;
extRfidDetect_t robotRFID;
extGameResult_t gameResult;
extBuffMusk_t robotBuff;
extGameRobotPos_t robotPos;

PC_Command_t PC_cmd;
Judge_Data_t Judge_data;
PC_Upload_data_t PC_upload_data = {0};
static PC_Rx_State_e PC_Rx_State = PC_SOF;
static uint8_t PC_Rx_Counter = 0;
static PC_Rx_Struct_t PC_Rx_Struct;
const uint8_t PC_HEADER = 0xCE;

extern BumpSensors_t bSensors;
extern UART_HandleTypeDef huart4;
extern Encoder GMYawEncoder;
extern Encoder GMPitchEncoder;

void PcRx_Reset(PC_Rx_State_e* state,PC_Rx_Struct_t* rx)
{
	*state = PC_SOF;
	rx->Cmd = 0x00;
	rx->Length = 0x00;
}

void COM_Receive_PC(PC_Rx_Struct_t* rx)
{
	switch(rx->Cmd)
	{
		case 0x01:
		{
			PC_cmd.Chassis.dir = rx->Data[0];
		}break;
		case 0x02:
		{
			PC_cmd.Gimbal.yaw_angle = byte2float(rx->Data);
			PC_cmd.Gimbal.pitch_angle = byte2float(rx->Data+4);
			PC_cmd.Gimbal.update = 1; // yaw & pitch as incremental value. Use update flag to ensure one increment per cycle.
		}break;
		case 0x03:
		{
			PC_cmd.shoot.fire = rx->Data[0];
		}break;
		default:
		{
		}
	}
}

void COM_Receive_PC_handler(uint8_t byte)
{
	switch(PC_Rx_State)
	{
		case PC_SOF:
		{
			if(byte==0xCE) PC_Rx_State = PC_LENGTH;
		}break;
		case PC_LENGTH:
		{
			if(byte<=0x08) 
			{
				PC_Rx_Struct.Length = byte;
				PC_Rx_State = PC_CMD;
			}
			else
				PcRx_Reset(&PC_Rx_State,&PC_Rx_Struct);
		}break;
		case PC_CMD:
		{
			if(byte<=0x03) 
			{
				PC_Rx_Struct.Cmd = byte;
				PC_Rx_State = PC_DATA;
			}
			else
				PcRx_Reset(&PC_Rx_State,&PC_Rx_Struct);
		}break;
		case PC_DATA:
		{
			PC_Rx_Struct.Data[PC_Rx_Counter++] = byte;
			if(PC_Rx_Counter == PC_Rx_Struct.Length)
			{
				COM_Receive_PC(&PC_Rx_Struct);
				PC_Rx_Counter = 0;
				PcRx_Reset(&PC_Rx_State,&PC_Rx_Struct);
			}
		}break;
		default:
		{
			PcRx_Reset(&PC_Rx_State,&PC_Rx_Struct);
			PC_Rx_Counter = 0;
		}
	}
}
void COM_Upload_DataProcess(void)
{
	if(bSensors.reachLeft || bSensors.reachRight)
	{
		(bSensors.reachLeft) ? (PC_upload_data.end=1) : (PC_upload_data.end=-1);
	}
	else
	{
		PC_upload_data.end = 0;
	}
	PC_upload_data.judge.armorType = robotHurt.armorType;
	PC_upload_data.judge.hurtType = robotHurt.hurtType;
	PC_upload_data.real_yaw = (int16_t)GMYawEncoder.ecd_angle;
	PC_upload_data.real_pitch = (int16_t)GMPitchEncoder.ecd_angle;
}

static uint8_t watch_tx0;
static uint8_t watch_tx1;
void COM_Upload_PC(void)
{
	COM_Upload_DataProcess();
	uint8_t tx[UPLOAD_FRAMELENGTH];
	tx[0] = PC_HEADER; 
	tx[1] = PC_upload_data.end;
	uint16_t2byte(PC_upload_data.judge.remainHP,tx+2);
	tx[4] = PC_upload_data.judge.hurtType;
	int16_t2byte(PC_upload_data.real_yaw,tx+5);
	int16_t2byte(PC_upload_data.real_pitch,tx+7);
	watch_tx0 = tx[5];
	watch_tx1 = tx[6];
	int i;
	for(i=0;i<UPLOAD_FRAMELENGTH;i++)
	{
		huart4.Instance->DR = tx[i];
		while(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_TC)==RESET);
	}
}

void COM_Judge_Process(Judge_RxStruct* rx)
{
	switch(rx->Cmd)
	{
		case 1: //robotState
		{
//			robotState.stageRemainTime = (rx->Data[1]<<8)|rx->Data[0];
//			robotState.gameProgress = rx->Data[2];
//			robotState.robotLevel = rx->Data[3];
//			robotState.remainHP = (rx->Data[5]<<8)|rx->Data[4];
//			robotState.maxHP = (rx->Data[7]<<8)|rx->Data[6];
//			memcpy(&robotState,rx->Data,20);
		}break;
		case 2: //robotHurt
		{
			robotHurt.armorType = rx->Data[0]>>4;
			robotHurt.hurtType = rx->Data[0];
		}break;
//		case 3: //shootData
//		{
//			shootData.bulletType = rx->Data[0];
//			shootData.bulletSpeed = rx->Data[1];
//			shootData.bulletSpeed = byte2float(rx->Data+2);
//		}break;
//		case 4: //PowerHeatData
//		{
//			PHData.chassisVolt = byte2float(rx->Data);
//			PHData.chassisCurrent = byte2float(rx->Data+4);
//			PHData.chassisPower = byte2float(rx->Data+8);
//			PHData.chassisPowerBuffer = byte2float(rx->Data+12);
//			PHData.shooterHeat0 = (rx->Data[17]<<8)|rx->Data[16];
//			PHData.shooterHeat1 = (rx->Data[19]<<8)|rx->Data[18];
//		}break;
//		case 5: //RFID data
//		{
//			robotRFID.cardType = rx->Data[0];
//			robotRFID.cardIdx = rx->Data[1];
//		}break;
//		case 6:
//		{
//			gameResult.winner = rx->Data[0];
//		}break;
//		case 7:
//		{
//			robotBuff.buffMusk = (rx->Data[1]<<8)|rx->Data[0];
//		}break;
//		case 8:
//		{
//			robotPos.x = byte2float(rx->Data);
//			robotPos.y = byte2float(rx->Data+4);
//			robotPos.z = byte2float(rx->Data+8);
//			robotPos.yaw = byte2float(rx->Data+12);
//		}break;
		default:
		{
		}
	}
}

uint8_t CRC8_check(Judge_RxStruct* rx)
{
	return 1;//to be developed
}

uint8_t CRC16_check(Judge_RxStruct* rx)
{
	return 1;//to be developed
}

void JudgeRx_Reset(Judge_RxState_e* state,Judge_RxStruct* rxStruct)
{
	(*state) = SOF;
	rxStruct->Length = 0;
	rxStruct->Seq = 0;
	rxStruct->CRC8 = 0;
	rxStruct->Cmd = 0;
	rxStruct->CRC16 = 0;
}


