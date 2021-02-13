#ifndef _VISUALPROCESS_H
#define _VISUALPROCESS_H
#include "system.h"


#define CONVER_SCALE  21.1f//20.86


typedef struct
{
    float YawGet_KF;
    float YawTarget_KF;

    float PitchGet_KF;
    float PitchTarget_KF;

    float DistanceGet_KF;
    float DistanceTarget_KF;
} Visual_TypeDef;

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
//队列对象
void Vision_task(void);
void Vision_Kalman_Init(void);

float Get_Target_Speed(uint8_t queue_len,float angle);
float Get_Target_Accel(uint8_t queue_len,float speed);
float Get_Distance_Tendency(uint8_t queue_len,float dis);
void Clear_Queue(QueueObj* queue);
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data);

extern Visual_TypeDef VisualProcess;

extern float Yaw_speed,Pitch_speed;
extern float predict_angle_raw;
extern float target_speed_raw,target_accel_raw;
extern  float YawTarget_now,PitchTarget_now;
extern uint32_t vision_this_time,vision_last_time;
extern uint16_t vision_update_fps;
extern float update_cloud_yaw,update_cloud_pitch;
extern bool predict_flag;
extern float use_predic;





#endif
