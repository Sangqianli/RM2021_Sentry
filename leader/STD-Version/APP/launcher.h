#ifndef __LAUNCHER_H
#define __LAUNCHER_H
#include "system.h"

extern PVM_TypeDef Launcher_Dial;
extern PPM_TypeDef An_Bullet_PPM;
extern int Friction_Open;
extern int Dail_Open;

void Launcher_Init(void);
void Launcher_task(void);
void Dial_Remote_An(void);
void Friction_Control(void);
void Launcher_task(void);

























#endif
