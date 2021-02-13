#ifndef __MODE_H
#define __MODE_H

#include "system.h"

enum robot_mode
{
    prepare_Mode=0,
    control_Mode
};

extern int RobotMonitor;
void Mode_Init(void);
void Mode_switch(void);
void Recover_Slowly(void);
void Robot_Monitor(void);



#endif
