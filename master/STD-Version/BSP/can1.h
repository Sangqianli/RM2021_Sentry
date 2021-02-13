#ifndef __CAN1_H
#define __CAN1_H

#include "system.h"

void CAN1_Init(void);
void CAN1_Send(uint32_t Equipment_ID,int16_t Data0,int16_t Data1,int16_t Data2,int16_t Data3);

#endif

