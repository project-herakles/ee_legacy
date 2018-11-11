#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "button.h"
	

int main(void)
{
	
	LED_Init();
	Button_Init();
	while(1){
		if (!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10)){
			
			RED_LED_TOGGLE();
			GREEN_LED_TOGGLE();
		}
	}

}


