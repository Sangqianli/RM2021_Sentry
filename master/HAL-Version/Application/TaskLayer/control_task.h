#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "fire_task.h"

/* Exported macro ------------------------------------------------------------*/
#define	KEYBOARD_STOP_FIRE	1
#define	KEYBOARD_DIERCT_RUN	2
#define	KEYBOARD_BACK_SCAN  3
#define KEYBOARD_STAY_LEFT  4
/* Exported types ------------------------------------------------------------*/
extern int16_t NormalData_0x200[4];
extern int16_t NormalData_0x2FF[4];
extern int16_t NormalPwm[2];
extern int16_t Mode_Data;
/* Exported functions --------------------------------------------------------*/

#endif
