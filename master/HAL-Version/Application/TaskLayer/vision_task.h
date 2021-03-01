#ifndef __VISION_TASK_H
#define __VISION_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rp_math.h"
#include "system_task.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
    float YawGet_KF;
    float YawTarget_KF;

    float PitchGet_KF;
    float PitchTarget_KF;

    float DistanceGet_KF;
    float DistanceTarget_KF;
} Visual_TypeDef;


typedef struct {
    QueueObj speed_queue;
    QueueObj accel_queue;
    QueueObj dis_queue;
    Visual_TypeDef  data_kal;
    float predict_angle;
    float feedforwaurd_angle;
    float speed_get;
    float accel_get;
    float distend_get;
    float offset_yaw;
    float offset_pitch;
} Vision_process_t;

extern Vision_process_t Vision_process;
/* Exported functions --------------------------------------------------------*/
void Vision_Init(void);
void StartVisionTask(void const * argument);
#endif
