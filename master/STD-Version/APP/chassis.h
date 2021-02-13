#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "system.h"

#define Encoder_CM 20.41
#define Pathway_CM 260
#define Mileage_total (Pathway_CM/Encoder_CM)*2400

#define Normal_Speed -3000
#define Swerve_Ratio 0.1

#define Left_PES PDin(12)			/*白线，右手边光电开关*/
#define Right_PES PDin(13)		/*黑线，左手边光电开关*/



extern PVM_TypeDef Chassis;

extern float  rotate_ratio;



void Chassis_Init(void);

void Chassis_Data(void);

void Get_Switch_Status(void);

void Cruise_Normal(void);

void Chassis_task(void);



int32_t Get_Position(void);


#endif

