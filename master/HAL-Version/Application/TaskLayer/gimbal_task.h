#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rp_math.h"
#include "system_task.h"

/* Exported macro ------------------------------------------------------------*/
typedef struct
{
    uint16_t nowLength;
    //长度
    float queue[200];
    //指针
    bool is_queue_full ; 	/*队列满标志*/
} Record_queue;
/* Exported types ------------------------------------------------------------*/
typedef struct Gimbal {
    pid_ctrl_t   YAW_PPM;
    pid_ctrl_t	 YAW_PVM;
    pid_ctrl_t   PITCH_PPM;
    pid_ctrl_t	 PITCH_PVM;
    float        Yaw_taget;
    float        Pitch_taget;
    float        RealYaw_speed;
    Record_queue Gimbal_queue;
} Gimbal_t;
/* Exported functions --------------------------------------------------------*/
extern Gimbal_t Gimbal_process;
void Update_Gimbal_Angle_Queue(int16_t now_angle);
float Get_Queue_Angle(uint8_t time_dis);
void Gimbal_Init(void);
void StartGimbalTask(void const * argument);
#endif
