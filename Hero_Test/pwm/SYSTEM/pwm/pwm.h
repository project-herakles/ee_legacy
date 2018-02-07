#ifndef _PWM_H_
#define _PWM_H_
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void PWM_Configuration(void);
void PWM_OutputPulse(uint16_t pulse);

#endif
