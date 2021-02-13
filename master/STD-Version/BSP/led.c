#include "led.h"

void Led_Init(void)
{
	GPIO_InitTypeDef gpio;	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;	
	gpio.GPIO_Mode = GPIO_Mode_OUT;										
	gpio.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;  		
	GPIO_Init(GPIOC,&gpio);	
	
	Green_Off;
	Red_Off;
	Blue_Off;
	Orange_Off;
}
