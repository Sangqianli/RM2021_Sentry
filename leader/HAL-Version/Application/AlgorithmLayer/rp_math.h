#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "pid.h"
#include "kalman.h"
/*数据结构*/
typedef struct
{
    uint16_t nowLength;
    uint16_t queueLength;
    float queueTotal;
    //长度
    float queue[100];
    //指针
    float aver_num;//平均值

    float Diff;//差分值

    uint8_t full_flag;
} QueueObj;
/*****************************/


/* 数值函数 */
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define abs(x) 					((x)>0? (x):(-(x)))

int16_t RampInt(int16_t final, int16_t now, int16_t ramp);
float RampFloat(float step,float target,float current);
float DeathZoom(float input, float center, float death);
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data);
void Clear_Queue(QueueObj* queue);
#endif

