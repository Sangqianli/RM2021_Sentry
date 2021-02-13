#ifndef _PID_H
#define _PID_H



#include "stdint.h"
typedef struct _PID_TypeDef
{


    float target;							//目标值

    float kp;
    float ki;
    float kd;

    float   measure;					//测量值
    float   err;							//误差
    float   last_err;      		//上次误差
    float   previous_err;  //上上次误差

    float pout;
    float iout;
    float dout;

    float output;						//本次输出
    float last_output;			//上次输出

    float MaxOutput;				//输出限幅
    float IntegralLimit;		//积分限幅
} PID_TypeDef;

typedef struct
{
    PID_TypeDef PID_PVM;
    int16_t speed_get;
    int16_t speed_set;

} PVM_TypeDef;

typedef struct
{
    PID_TypeDef PID_PPM;
    int16_t position_round;
    uint16_t position_get_now;
    uint16_t position_get_last;
    int16_t position_set;
    int16_t position;

    float yaw_get;
    float yaw_base;
    float yaw_target;
} PPM_TypeDef;





int pid_calculate(PID_TypeDef* pid);

void 	pid_clear(PID_TypeDef* pid);



















#endif
