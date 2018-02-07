#ifndef _LED_H_
#define _LED_H_
#include "stm32f4xx_gpio.h"
#include "delay.h"

void LED_Configuration(void);
void Green_Flash(uint16_t flash);
void Green_Twinkle(uint16_t times);
#define RED_OFF GPIO_SetBits(GPIOE,GPIO_Pin_7)
#define RED_ON GPIO_ResetBits(GPIOE,GPIO_Pin_7)
#define GREEN_OFF GPIO_SetBits(GPIOF,GPIO_Pin_14)
#define GREEN_ON GPIO_ResetBits(GPIOF,GPIO_Pin_14)
extern int flashed;
#endif
