#include "main.h"
#include "convert.h"

PC_Command_t PC_cmd;

static PC_Rx_State_e PC_Rx_State = PC_SOF;
static uint8_t PC_Rx_Counter = 0;
static PC_Rx_Struct_t PC_Rx_Struct;

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
			PC_cmd.Gimbal.yaw_angle = byte2float(rx->Data);
			PC_cmd.Gimbal.pitch_angle = byte2float(rx->Data+4);
		}break;
		case 0x02:
		{
			PC_cmd.shoot.fire = rx->Data[0];
		}break;
		default:
		{
		}
	}
}
static uint8_t byte_s = 0;
void pc_handler(uint8_t byte)
{
	
	byte_s = byte;
	//RED_LED_ON();
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


