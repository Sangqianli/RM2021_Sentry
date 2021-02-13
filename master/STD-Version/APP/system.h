#ifndef __SYSTEM_H
#define __SYSTEM_H

#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "sys.h"
#include "usart.h"

#include "myiic.h"
#include "bmi.h"
#include "spi2.h"
#include "bmi2.h"
#include "bmi270.h"
#include "bmi2_common.h"

#include "can1.h"
#include "can2.h"
#include "led.h"
#include "usart2.h"
#include "control.h"
#include "crc.h"
#include "pwm.h"
#include "usart5.h"
#include "time1.h"
#include "io.h"
#include "anoc.h"
#include "laser.h"


#include "pid.h"
#include "chassis.h"
#include "gimbal.h"
#include "mode.h"
#include "launcher.h"

#include "judge.h"
#include "vision.h"
#include "kalman.h"
#include "visual_process.h"
extern volatile uint32_t sysTickUptime;

extern struct  system_time
{
    uint32_t ms_1;

    uint32_t ms_3;

    uint32_t ms_2;
} TimeFlag;

extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据
extern float pitch,roll,yaw;		//欧拉角
extern short accx,accy,accz;
extern int SystemMonitor;

#define abs(x) ((x)>0? (x):(-(x)))

#define Remote_Mode  RC_Ctl.rc.s2 == 3
#define Check_Mode RC_Ctl.rc.s2 == 2
#define Cruise_Mode RC_Ctl.rc.s2 == 1


float myDeathZoom(float center,float width,float input);
float RampFloat(float step,float target,float current);
float constrain(float amt, float low, float high);
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);
uint32_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint32_t millis(void);
void System_Init(void);
void Loop(void);
void Stop(void);
#endif


