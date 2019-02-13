#ifndef __SENSOR_H_
#define __SENSOR_H_

#include "gpio.h"
#include "stm32f4xx_hal_gpio.h"

#define GET_LEFT_BUMP() HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_2)
#define GET_RIGHT_BUMP() HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7)

typedef struct
{
	uint8_t reachLeft;
	uint8_t reachRight;
}BumpSensors_t;

void BumpSensorInit(void);
void UpdateBumpSensors(void);

#endif
