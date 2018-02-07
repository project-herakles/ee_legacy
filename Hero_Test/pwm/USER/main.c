#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "stm32f4xx_tim.h"
#include "pwm.h"
#include "led.h"
#include "key.h"

//ALIENTEK ̽����STM32F407������ ʵ��0
//STM32F4����ģ��-�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK

int flashed = 0;

void ESC_Init(){
	PWM_OutputPulse(3000);//Throttle full
	delay_ms(4000);
	PWM_OutputPulse(1000);//Throttle zero
	delay_ms(1000);
}

int main(void)
{
	int gear = 1;
	int shake = 0;
	PWM_Configuration();
	LED_Configuration();
	KEY_Configuration();
	delay_init(168);
	ESC_Init();
	
	
	while(1){
		Green_Flash(gear);
		if(SWITCH_IS_PRESSED == 0){
			shake = 1;
			delay_ms(20);
		}
	 if(SWITCH_IS_PRESSED == 0 && shake == 1){
		  gear++;
			shake = 0;
			if(gear>6)
				gear = 1;
			PWM_OutputPulse(600+400*gear);
			RED_ON;
			delay_ms(200);
			RED_OFF;
		}
		shake = 0;
	}
}	

/*
�ֲ��н��⵽����15��ʱ���main.cԴ�����£�
#include "stm32f4xx.h"

//ALIENTEK ̽����STM32F407������ ʵ��0
//STM32F4����ģ��-�⺯���汾
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK
  
void Delay(__IO uint32_t nCount);

void Delay(__IO uint32_t nCount)
{
  while(nCount--){}
}

int main(void)
{

  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  while(1){
		GPIO_SetBits(GPIOF,GPIO_Pin_9|GPIO_Pin_10);
		Delay(0x7FFFFF);
		GPIO_ResetBits(GPIOF,GPIO_Pin_9|GPIO_Pin_10);
		Delay(0x7FFFFF);
	
	}
}
*/


