#include "sensor.h"
#include "stm32f4xx_hal_gpio.h"

BumpSensors_t bSensors = {0,0};

void BumpSensorInit(void)
{
	__HAL_RCC_GPIOI_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
}

void UpdateBumpSensors(void)
{
	(GET_LEFT_BUMP()==RESET) ? (bSensors.reachLeft=1):(bSensors.reachLeft=0);
	(GET_RIGHT_BUMP()==RESET) ? (bSensors.reachRight=1):(bSensors.reachRight=0);
}


