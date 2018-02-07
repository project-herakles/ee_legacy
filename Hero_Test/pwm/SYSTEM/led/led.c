#include "led.h"

void LED_Configuration(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_7;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOE,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOF,&gpio);
	
	RED_OFF;
	GREEN_OFF;
}
void Green_Twinkle(uint16_t times){
	GREEN_ON;
	delay_ms(200);
	GREEN_OFF;
	delay_ms(180+times*20);
}
void Green_Flash(uint16_t flash){
	uint16_t times = flash;
	while(times!=0){
		Green_Twinkle(times);
		times--;
	}
	GREEN_OFF;
	delay_ms(4000-(100*times));
}
