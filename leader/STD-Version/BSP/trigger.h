#ifndef __TRIGGER_H
#define __TRIGGER_H

#define Trigger_On  GPIO_SetBits(GPIOB,GPIO_Pin_10)
#define Trigger_Off GPIO_ResetBits(GPIOB,GPIO_Pin_10)
#include "system.h"

void TRIGGER_Init(void);


#endif
