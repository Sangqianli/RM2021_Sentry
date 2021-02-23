#ifndef __SYSTEM_TASK_H
#define __SYSTEM_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "control_task.h"
#include "monitor_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "fire_task.h"
#include "vision_task.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void StartSystemTask(void const * argument);
void Application_Init(void);
#endif
