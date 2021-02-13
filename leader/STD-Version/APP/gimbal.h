#ifndef __GIMBAL__H
#define __GIMBAL__H

#include <system.h>


#define cnt_per_round  8191
#define deduction_ratio 1
#define cnt_per_round_out (cnt_per_round * deduction_ratio)
#define ATTACK_SWITCH_RAMP  200
typedef enum			/*云台模式*/
{
    SCOUT=1,			//侦察模式
    ATTACK		//打击模式
} CRUISE_Mode;

extern float yaw_PPM_ratio;
extern float yaw_PVM_ratio;
extern float pitch_PVM_ratio;
extern float pitch_PPM_ratio;

extern float yaw_error;
extern float yaw_round;
extern float pitch_base;


extern PPM_TypeDef Gimbal_yaw_PPM;
extern PPM_TypeDef Gimbal_pitch_PPM;
extern PVM_TypeDef Gimbal_yaw_PVM;
extern PVM_TypeDef Gimbal_pitch_PVM;
extern CRUISE_Mode Cruise_mode;
void Gimbal_Init(void);
void Gimbal_Remote_Data(void);
void Gimbal_Measure_Data(void);

void Scout(void);
void Gimbal_task(void);
void Gimbal_Follow(void);

#endif
