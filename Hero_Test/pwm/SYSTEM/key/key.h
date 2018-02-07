#ifndef _KEY_H_
#define _KEY_H_

#include "stm32f4xx_gpio.h"
#include "delay.h"

void KEY_Configuration(void);
#define SWITCH_IS_PRESSED GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_10)
#define FIRE_IS_PRESSED GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_10)

#endif
