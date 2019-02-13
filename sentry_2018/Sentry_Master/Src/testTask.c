#include "testTask.h"
#include "gpio.h"
#include "tim.h"
#include "can.h"
#include "main.h"

extern uint8_t canTxMsg[8];
extern UART_HandleTypeDef huart1;
extern uint8_t RemoteBuffer[18];


void Test_Task(void)
{

	GREEN_LED_TOGGLE();
	RED_LED_OFF();
	//CAN_SendMsg(&hcan1,canTxMsg);
}
