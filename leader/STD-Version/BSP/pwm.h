#ifndef __PWM_H__
#define __PWM_H__

#include "system.h"
extern int PWM3_Target;
void PWM1_Init(void);
void PWM3_Init(void);
void Friction_PWM(int16_t pwm1,int16_t pwm2);
void Feeding_Bullet_PWM(int16_t pwm1);
void Magazine_Output_PWM(int16_t pwm);

#define PWM1  TIM3->CCR2
#define PWM2  TIM3->CCR1

#endif

