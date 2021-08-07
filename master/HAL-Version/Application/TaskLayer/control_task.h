#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "fire_task.h"

/* Exported macro ------------------------------------------------------------*/
#define	KEYBOARD_STOP_FIRE	11
#define	KEYBOARD_DIERCT_RUN	22
#define	KEYBOARD_BACK_SCAN  33
#define KEYBOARD_STAY_LEFT  44
/* Exported types ------------------------------------------------------------*/
extern int16_t NormalData_0x200[4];
extern int16_t NormalData_0x2FF[4];
extern int16_t NormalPwm[2];
extern int16_t Mode_Data;
/* Exported functions --------------------------------------------------------*/

#endif
