#ifndef __PID_H
#define __PID_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void pid_clear(pid_ctrl_t *pid);
void pid2_clear(pid2_ctrl_t *pid);
void pid_calculate(pid_ctrl_t *pid);
void pid2_calculate(pid2_ctrl_t *pid);

#endif
