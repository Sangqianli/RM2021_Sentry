#ifndef __FIRE_TASK_H
#define __FIRE_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rp_math.h"
#include "system_task.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct Fire {
    pid_ctrl_t PPM;
    pid_ctrl_t PVM;
    bool       Stuck_flag;
    bool       Friction_ready;
    float      Speed_target;
    int16_t    Friction_target;
} Fire_t;
/* Exported functions --------------------------------------------------------*/
void Fire_Init(void);
void StartFireTask(void const * argument);
#endif
