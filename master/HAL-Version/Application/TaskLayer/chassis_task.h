#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "system_task.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct Chassis {
	pid_ctrl_t	 PVM;
	float        Speed_taget;
	int32_t		 Mileage_atrip;//轨道长度
	uint8_t		 init_flag;
	int8_t       Derection_flag;//1为左，-1为右
	uint8_t      rotate_ratio;//遥控灵敏度
	bool         swerve_judge;//反弹是否完成的判断
    bool         swerve_flag;//反弹流程标志位
} Chassis_t;
/* Exported functions --------------------------------------------------------*/
extern Chassis_t Chassis_process;
void   Chassis_Init(void);
void   StartChassisTask(void const * argument);
#endif
