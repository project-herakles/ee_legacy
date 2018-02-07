#include "led.h"
#include "stm32f4xx.h"

void LED_Init(void)
{
	
 GPIO_InitTypeDef GPIO_InitStructure;
	
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);	

	//F9
 GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
 GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_SetBits(GPIOE,GPIO_Pin_7);
	
	//F10
 GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
 GPIO_Init(GPIOF,&GPIO_InitStructure);	
GPIO_SetBits(GPIOF,GPIO_Pin_1);

}


