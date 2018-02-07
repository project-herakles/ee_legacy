#ifndef  __LED_H

#define __LED_H




void LED_Init(void);

#define GREEN_LED_TOGGLE()      GPIO_ToggleBits(GPIOF, GPIO_Pin_14)
#define RED_LED_TOGGLE()        GPIO_ToggleBits(GPIOE, GPIO_Pin_7)

#endif

