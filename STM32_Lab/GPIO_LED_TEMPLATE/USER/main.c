//#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"



int main(void)
{
 delay_init(168);
	
 LED_Init();
	
	while(1){
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
			//GPIO_SetBits(GPIOF,GPIO_Pin_14);
		GPIO_SetBits(GPIOF, GPIO_Pin_1);
		delay_ms(500);
		
			GPIO_ResetBits(GPIOE,GPIO_Pin_7);
			//GPIO_ResetBits(GPIOF,GPIO_Pin_14);
			GPIO_ResetBits(GPIOF, GPIO_Pin_1);
	
	   delay_ms(500);
	
	
	}

}

